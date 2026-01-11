using System;
using System.Collections.Generic;
using System.Linq;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino.DocObjects;

namespace Hagoromo.DataStructure
{
    public class AllBeamElem : GH_Component
    {
        public AllBeamElem()
          : base("All Beam Elemnt Info", "AllBeamElem",
              "Gather all element information and display codes",
              "Hagoromo", "Input")
        {
        }

        private List<Point3d> _nodes = new List<Point3d>();
        private List<Line> _lines = new List<Line>();
        private List<string> _elementCodes = new List<string>();
        private List<string> _nodeCodes = new List<string>();
        private Dictionary<string, object> _nodeDictionary = new Dictionary<string, object>();
        private Dictionary<string, object> _elementDictionary = new Dictionary<string, object>();
        private double _textSize = 5.0;

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Elemdata", "e", "List of Elemdata", GH_ParamAccess.list);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("AllElemdata", "E", "Combined element data", GH_ParamAccess.list);
            pManager.AddTextParameter("Contents", "C", "Contents of AllElemdata", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var elemdataList = new List<GH_Elemdata>();

            // Retrieve inputs
            if (!DA.GetDataList(0, elemdataList) || elemdataList.Count == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "No element data provided.");
                return;
            }

            _nodes.Clear();
            _lines.Clear();

            // Collect nodes and lines
            foreach (var ghElemData in elemdataList)
            {
                if (!ghElemData.IsValid || ghElemData.Value == null)
                    continue;

                var elemData = ghElemData.Value;
                _lines.AddRange(elemData.Lines);

                foreach (var line in elemData.Lines)
                {
                    _nodes.Add(line.From);
                    _nodes.Add(line.To);
                }
            }

            // Remove duplicate nodes
            var uniqueNodes = new HashSet<Point3d>(_nodes);
            _nodes = uniqueNodes.ToList();

            // Initialize arrays
            double[,] nodeArray = new double[_nodes.Count, 3];
            int totalLines = elemdataList.Sum(e => e.Value?.Lines.Count ?? 0);
            object[,] elementArray = new object[totalLines, 5];

            // Populate node array
            for (int i = 0; i < _nodes.Count; i++)
            {
                nodeArray[i, 0] = _nodes[i].X;
                nodeArray[i, 1] = _nodes[i].Y;
                nodeArray[i, 2] = _nodes[i].Z;
            }

            // Populate element array
            int elementCounter = 0;
            foreach (var ghElemData in elemdataList)
            {
                if (!ghElemData.IsValid || ghElemData.Value == null)
                    continue;

                var elemData = ghElemData.Value;
                int lineCounter = 0;

                foreach (var line in elemData.Lines)
                {
                    var startIndex = _nodes.IndexOf(line.From);
                    var endIndex = _nodes.IndexOf(line.To);

                    if (startIndex == -1 || endIndex == -1)
                    {
                        AddRuntimeMessage(GH_RuntimeMessageLevel.Error, $"Node not found for line: {line}");
                        continue;
                    }

                    elementArray[elementCounter, 0] = elemData.SectId; // SECT
                    elementArray[elementCounter, 1] = new int[] { startIndex + 1, endIndex + 1 }; // NODE
                    elementArray[elementCounter, 2] = elemData.Constraint?.ToArray() ?? new bool[0]; // ENDS
                    elementArray[elementCounter, 3] =(elemData.RotInfo[lineCounter] as List<double>)?.ToArray(); //ROTS
                    elementArray[elementCounter, 4] = elemData.CMQ?.ToArray() ?? new double[0]; // CMQS

                    lineCounter++;
                    elementCounter++;
                }
            }

            // Combine data into the wrapper
            var combinedData = new AllElemdata(nodeArray, elementArray);

            // Generate output strings for nodes and elements
            var nodeText = string.Join("\n", Enumerable.Range(0, _nodes.Count).Select(i =>
                $"N{i + 1} X:{nodeArray[i, 0]:F2}, Y:{nodeArray[i, 1]:F2}, Z:{nodeArray[i, 2]:F2}"
            ));

            var elementText = string.Join("\n", Enumerable.Range(0, elementCounter).Select(i =>
            {
                var nodes = (int[])elementArray[i, 1];
                var ends = (bool[])elementArray[i, 2];
                var rots = (double[])elementArray[i, 3];
                new List<string>(); var cmqs = (double[])elementArray[i, 4];

                return $"E{i + 1000} SId:{elementArray[i,0]},NODE:[{nodes[0]},{nodes[1]}]," +
                       $"ENDS:[{string.Join(",", ends)}], ROTS:[{string.Join(",", rots.Select(c => c.ToString("F2")))}]," +
                       $"CMQS:[{string.Join(",", cmqs.Select(c => c.ToString("F2")))}]";
            }));

            // Set outputs
            DA.SetData(0, new GH_AllElemdata(combinedData));
            DA.SetData(1, $"{nodeText}\n{elementText}");
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);

                // Display node codes
                for (int i = 0; i < _nodes.Count; i++)
                {
                    Point3d node = _nodes[i];
                    string label = $"{i}"; // Use offset for proper labeling
                    Plane nodePlane = new Plane(node, Vector3d.ZAxis);

                    args.Display.Draw3dText(
                        label,
                        System.Drawing.Color.Green,
                        nodePlane,
                        _textSize,
                        "Arial",
                        false,
                        false,
                        TextHorizontalAlignment.Right,
                        TextVerticalAlignment.Top
                    );
                }

                // Display element codes
                for (int i = 0; i < _lines.Count; i++)
                {
                    Line line = _lines[i];
                    string label = $"{i + 1000}"; // Use offset for proper labeling
                    Point3d midPoint = line.PointAt(0.5); // Midpoint of the line
                    Plane linePlane = new Plane(midPoint, Vector3d.ZAxis);

                    args.Display.Draw3dText(
                        label,
                        System.Drawing.Color.Blue,
                        linePlane,
                        _textSize,
                        "Arial",
                        false,
                        false,
                        TextHorizontalAlignment.Center,
                        TextVerticalAlignment.Middle
                    );
                }
        }

        public override BoundingBox ClippingBox
        {
            get
            {
                // Include all nodes and midpoints of lines to ensure they are included in the viewport
                var boundingBox = BoundingBox.Empty;
                foreach (var node in _nodes)
                    boundingBox.Union(node);
                foreach (var line in _lines)
                    boundingBox.Union(line.PointAt(0.5)); // Midpoint of lines for text
                return boundingBox;
            }
        }

        public override bool IsPreviewCapable
        {
            get { return true; } // Ensure preview is always enabled
        }

        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return null;
            }
        }

        public override Guid ComponentGuid
        {
            get { return new Guid("6A9922FE-4377-458B-8D2F-60E7F516DECE"); }
        }
    }
}
