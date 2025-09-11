using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Hagoromo.DataStructure;
using Rhino.Display;
using Rhino.DocObjects;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;


namespace Hagoromo.Display
{
    public class ShowInput : GH_Component
    {
        private double _szSect = 0;
        private double _szNode = 0;
        private double _szElem = 0;
        private double _szICon = 0;
        private double _szVCon = 0;
        private double _szEC = 0;
        private double _szCMQ = 0;

        private List<System.Drawing.Color> _elemColors = new List<System.Drawing.Color> 
        { System.Drawing.Color.DeepSkyBlue,System.Drawing.Color.Green, System.Drawing.Color.Orange, System.Drawing.Color.Brown,
         System.Drawing.Color.Aqua, System.Drawing.Color.DarkMagenta,  System.Drawing.Color.DarkGreen,
         System.Drawing.Color.Cornsilk,  System.Drawing.Color.DarkKhaki} ;

        private int[] _sectIDs = new int[] { };

        public ShowInput()
          : base("Show Input", "ShowInput",
              "Show input data. Size 0: not displayed ",
              "Hagoromo", "Display")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Alldata", "A", "Combined data package", GH_ParamAccess.item);
            pManager.AddNumberParameter("Size of SectID", "szSect", "The size of text shown", GH_ParamAccess.item, 0);
            pManager.AddNumberParameter("Size of NodeID", "szNode", "The size of text shown", GH_ParamAccess.item, 0);
            pManager.AddNumberParameter("Size of ElemID", "szElem", "The size of text shown", GH_ParamAccess.item, 0);
            pManager.AddNumberParameter("Size of ICon", "szICon", "The size of text shown", GH_ParamAccess.item, 0);
            pManager.AddNumberParameter("Size of VCon", "szVCon", "The size of text shown", GH_ParamAccess.item, 0);
            pManager.AddNumberParameter("Size of Beam End Conditions", "szEC", "The size of text shown", GH_ParamAccess.item, 0);
            pManager.AddNumberParameter("Size of CMQ", "szCMQ", "The size of text shown", GH_ParamAccess.item, 0);
        }


        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
        }


        private List<Point3d> _nodes = new List<Point3d>();  // Store unique nodes
        private List<Line> _lines = new List<Line>();        // Store lines

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_Alldata goo = null;

            // 入力スロット0から GH_Alldata を取得
            if (!DA.GetData(0, ref goo)) return;
            // nullチェック
            if (goo == null || goo.Value == null)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Alldata is null or invalid.");
                return;
            }
            // Alldata 型に変換
            Alldata alldata = goo.Value;
            fAlldata falldata = new fAlldata(alldata);

            int count = alldata.ElementArray.GetLength(0);
            Line[] elems = new Line[count];
            Point3d[] points = ConvertToPoints(falldata.fNodeXYZArray);
            _sectIDs = new int[count];

            for (int i = 0; i < count; i++)
            {
                int nodeid_s = (int)fAlldata.GetElemItem(falldata.fElemArray, i, 1, 0) - 1;
                int nodeid_e = (int)fAlldata.GetElemItem(falldata.fElemArray, i, 1, 1) - 1;
                elems[i] = new Line(points[nodeid_s], points[nodeid_e]);
                _sectIDs[i] = (int)fAlldata.GetElemItem(falldata.fElemArray, i, 0, 0);
            }

            

            DA.GetData(1, ref _szSect); 
            DA.GetData(2, ref _szNode);
            DA.GetData(3, ref _szElem);
            DA.GetData(4, ref _szICon);
            DA.GetData(5, ref _szVCon);
            DA.GetData(6, ref _szEC);
            DA.GetData(7, ref _szCMQ);

            _nodes = points.ToList();
            _lines = elems.ToList();
            
        }

        public Point3d[] ConvertToPoints(double[,] array)
        {
            int rowCount = array.GetLength(0);
            int colCount = array.GetLength(1);

            if (colCount < 3)
                throw new ArgumentException("各行には少なくとも X, Y, Z の3列が必要です。");

            List<Point3d> points = new List<Point3d>();

            for (int i = 0; i < rowCount; i++)
            {
                double x = array[i, 0];
                double y = array[i, 1];
                double z = array[i, 2];
                points.Add(new Point3d(x, y, z));
            }
            Point3d[] point3ds = points.ToArray();
            return point3ds;
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);

            if (_szNode != 0)
            {
                // Display node codes
                for (int i = 0; i < _nodes.Count; i++)
                {
                    Point3d node = _nodes[i];
                    string label = (i).ToString();        // Node index as string
                    Plane nodePlane = new Plane(node, Vector3d.ZAxis);

                    // Draw the text in the viewport
                    args.Display.Draw3dText(
                        label,                          // The text to display
                        System.Drawing.Color.Blue,     // Text color
                        nodePlane,                      // Position of the text
                        _szNode,                          // Text height
                        "Times New Roman",                        // Font
                        false,                          // Bold
                        false,                          // Italic
                        TextHorizontalAlignment.Right,  // Horizontal alignment
                        TextVerticalAlignment.Top       // Vertical alignment
                    );
                }
            }

            if (_szElem != 0)
            {
                // Display line codes
                for (int i = 0; i < _lines.Count; i++)
                {
                    Line line = _lines[i];
                    string label = (i).ToString();        // Line index as string
                    Point3d midPoint = line.PointAt(0.5); // Midpoint of the line
                    Plane linePlane = new Plane(midPoint, Vector3d.ZAxis);

                    // Draw the text in the viewport
                    args.Display.Draw3dText(
                        label,                          // The text to display
                        System.Drawing.Color.Magenta,      // Text color
                        linePlane,                      // Position of the text
                        _szElem,                          // Text height
                        "Times New Roman",                        // Font
                        false,                          // Bold
                        false,                          // Italic
                        TextHorizontalAlignment.Center, // Horizontal alignment
                        TextVerticalAlignment.Middle    // Vertical alignment
                    );
                }
            }

            
            if (_szSect != 0)
            {
                for (int i = 0; i < _lines.Count; i++)
                {
                    Line line = _lines[i];
                    string label = (_sectIDs[i]).ToString();        // Line index as string
                    Point3d midPoint = line.PointAt(0.7); // Midpoint of the line
                    Plane linePlane = new Plane(midPoint, Vector3d.ZAxis);

                    // Draw the text in the viewport
                    args.Display.Draw3dText(
                        label,                          // The text to display
                        _elemColors[(_sectIDs[i] - 1)%9],      // Text color
                        linePlane,                      // Position of the text
                        _szSect,                          // Text height
                        "Times New Roman",                        // Font
                        false,                          // Bold
                        false,                          // Italic
                        TextHorizontalAlignment.Center, // Horizontal alignment
                        TextVerticalAlignment.Middle    // Vertical alignment
                    );
                    args.Display.DrawLine(line, _elemColors[(_sectIDs[i] - 1)%9], 2); // 最後の引数は線の太さ
                }
            }
            
        }

        //public override bool IsPreviewCapable => true;

        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return null;
            }
        }

        public override Guid ComponentGuid
        {
            get { return new Guid("C87ECD80-FB3B-42A3-AE98-12D82D750F48"); }
        }
    }
}