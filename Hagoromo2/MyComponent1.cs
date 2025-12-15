using Grasshopper.Kernel;
using Rhino.Display;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Drawing;
using Hagoromo.DataStructure;

namespace Hagoromo.Visualization
{
    public class DisplayStructuralConditions : GH_Component, IGH_PreviewObject
    {
        private List<Point3d> _supportPoints = new List<Point3d>();
        private List<Vector3d> _gravityVectors = new List<Vector3d>();
        private Vector3d _globalGravity = Vector3d.Zero;

        public DisplayStructuralConditions()
          : base("Display Structural Conditions", "DisplayStruct",
              "Visualizes supports and loads from Alldata structure.",
              "Hagoromo", "Visualization")
        { }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Alldata", "D", "Structured input data for visualization", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            // Visual only — no output
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_Alldata ghData = null;
            if (!DA.GetData(0, ref ghData)) return;
            if (ghData == null || !ghData.IsValid) return;

            Alldata data = ghData.Value;
            _supportPoints.Clear();
            _gravityVectors.Clear();

            // 👇 例として: NodeArray を使ってノード位置取得
            for (int i = 0; i < data.NodeArray.GetLength(0); i++)
            {
                // 想定: NodeArray[row, 0] = X, [1] = Y, [2] = Z, [3] = サポートタイプ（0=自由,1=固定など）
                double x = Convert.ToDouble(data.NodeArray[i, 0]);
                double y = Convert.ToDouble(data.NodeArray[i, 1]);
                double z = Convert.ToDouble(data.NodeArray[i, 2]);
                int supportType = Convert.ToInt32(data.NodeArray[i, 3]);

                Point3d nodePos = new Point3d(x, y, z);
                if (supportType > 0)
                {
                    _supportPoints.Add(nodePos); // 固定またはピンなど支持条件あり
                }

                _gravityVectors.Add(data.GravityArray != null && data.GravityArray.Length == 3
                    ? new Vector3d(data.GravityArray[0], data.GravityArray[1], data.GravityArray[2])
                    : Vector3d.Zero);
            }

            _globalGravity = _gravityVectors.Count > 0 ? _gravityVectors[0] : Vector3d.Zero;
        }

        public override bool IsPreviewCapable => true;

        public void DrawViewportWires(GH_PreviewWireArgs args)
        {
            foreach (Point3d pt in _supportPoints)
            {
                args.Pipeline.DrawDot(pt, "🔗su", Color.SteelBlue, Color.White);
            }

            foreach (Point3d pt in _supportPoints)
            {
                args.Pipeline.DrawArrow(new Line(pt, pt + _globalGravity * 0.5), Color.OrangeRed, 5, 0);
            }
        }

        public override BoundingBox ClippingBox => new BoundingBox(_supportPoints);

        protected override Bitmap Icon => null;


        public override Guid ComponentGuid
        {
            get { return new Guid("F6F9D764-2A65-4889-89C4-1386BFBF68CB"); }
        }
    }
}