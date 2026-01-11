using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Grasshopper;
using Grasshopper.Kernel.Types;
using Rhino.Display;
using Rhino.DocObjects;

namespace Hagoromo
{
    public class Codes : GH_Component
    {
        public Codes()
          : base("Codes", "Codes",
              "Define node and element code",
              "Hagoromo", "test")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddLineParameter("Lines", "L", "The lines to calculate nodes", GH_ParamAccess.list);
            pManager.AddNumberParameter("Size", "s", "The size of text shown", GH_ParamAccess.item, 10.0);
            pManager.AddBooleanParameter("Display Nodes", "D", "True to show node codes, false to show line codes", GH_ParamAccess.item, true);
        }

        
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            //pManager.AddPointParameter("Points", "P", "Non duplicate points", GH_ParamAccess.list);
            //pManager.AddIntegerParameter("Codes", "C", "List of start and end node indices for each line", GH_ParamAccess.tree);
        }
        

        private List<Point3d> _nodes = new List<Point3d>();  // Store unique nodes
        private List<Line> _lines = new List<Line>();        // Store lines
        private double _size = 10.0;                         // Store text size
        private Grasshopper.DataTree<int> _lineCodes = new Grasshopper.DataTree<int>();
        private bool _displayNodes = true;                   // Boolean to toggle display
        

        // Custom equality comparer for Point3d to handle duplicates
        public class Point3dEqualityComparer : IEqualityComparer<Point3d>
        {
            public bool Equals(Point3d p1, Point3d p2)
            {
                return p1.DistanceTo(p2) < 1e-6; // Allow a small tolerance for floating-point comparisons
            }

            public int GetHashCode(Point3d obj)
            {
                return obj.X.GetHashCode() ^ obj.Y.GetHashCode() ^ obj.Z.GetHashCode();
            }
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // Clear existing data to prevent accumulation
            _nodes.Clear();
            _lines.Clear();
            _lineCodes = new Grasshopper.DataTree<int>();


            // Step 1: Retrieve the list of lines, size, and display toggle from the input
            DA.GetDataList("Lines", _lines);
            if (!DA.GetData("Size", ref _size))
            {
                _size = 10; // Default size if not specified
            }
            DA.GetData("Display Nodes", ref _displayNodes);

            // Step 2: Collect all unique nodes (start and end points of lines)
            HashSet<Point3d> nodeSet = new HashSet<Point3d>(new Point3dEqualityComparer());
            foreach (Line line in _lines)
            {
                nodeSet.Add(line.From);
                nodeSet.Add(line.To);
            }
            _nodes = new List<Point3d>(nodeSet);

            // Step 3: Create a node index map for each line
            for (int i = 0; i < _lines.Count; i++)
            {
                Line line = _lines[i];

                int startIndex = _nodes.IndexOf(line.From);
                int endIndex = _nodes.IndexOf(line.To);

                var path = new Grasshopper.Kernel.Data.GH_Path(i);
                _lineCodes.AddRange(new int[] { startIndex, endIndex }, path);
            }

            // Step 4: Output the results
            //DA.SetDataList(0, _nodes);
            //DA.SetDataTree(1, _lineCodes);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);

            if (_displayNodes)
            {
                // Display node codes
                for (int i = 0; i < _nodes.Count; i++)
                {
                    Point3d node = _nodes[i];
                    string label = (i + 100).ToString();        // Node index as string
                    Plane nodePlane = new Plane(node, Vector3d.ZAxis);

                    // Draw the text in the viewport
                    args.Display.Draw3dText(
                        label,                          // The text to display
                        System.Drawing.Color.Green,     // Text color
                        nodePlane,                      // Position of the text
                        _size,                          // Text height
                        "Arial",                        // Font
                        false,                          // Bold
                        false,                          // Italic
                        TextHorizontalAlignment.Right,  // Horizontal alignment
                        TextVerticalAlignment.Top       // Vertical alignment
                    );
                }
            }
            else
            {
                // Display line codes
                for (int i = 0; i < _lines.Count; i++)
                {
                    Line line = _lines[i];
                    string label = (i + 1000).ToString();        // Line index as string
                    Point3d midPoint = line.PointAt(0.5); // Midpoint of the line
                    Plane linePlane = new Plane(midPoint, Vector3d.ZAxis);

                    // Draw the text in the viewport
                    args.Display.Draw3dText(
                        label,                          // The text to display
                        System.Drawing.Color.Blue,      // Text color
                        linePlane,                      // Position of the text
                        _size,                          // Text height
                        "Arial",                        // Font
                        false,                          // Bold
                        false,                          // Italic
                        TextHorizontalAlignment.Center, // Horizontal alignment
                        TextVerticalAlignment.Middle    // Vertical alignment
                    );
                }
            }
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
            get { return new Guid("9CB14BDA-324D-4E68-82DC-22ED7B69BE59"); }
        }
    }
}
