using Eto.Drawing;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Hagoromo.DataStructure
{
    public class PointsAndElems : GH_Component
    {
        public PointsAndElems()
          : base("PointsAndElems", "PtElem",
              "get points and elems in order of alldata.",
              "Hagoromo", "Linear")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Alldata", "A", "Combined data package", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("Nodes", "Nodes", "Nodes", GH_ParamAccess.list);
            pManager.AddLineParameter("Elements", "Elems", "Elements", GH_ParamAccess.list);
        }

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

            for (int i = 0; i < alldata.ElementArray.GetLength(0); i++)
            {
                int nodeid_s = (int)fAlldata.GetElemItem(falldata.fElemArray, i, 1, 0) - 1;
                int nodeid_e = (int)fAlldata.GetElemItem(falldata.fElemArray, i, 1, 1) - 1;
                elems[i] = new Line(points[nodeid_s], points[nodeid_e]);
            }

            DA.SetDataList(0, points);
            DA.SetDataList(1, elems);
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

        public override Guid ComponentGuid
        {
            get { return new Guid("CCC0E16E-5749-47F6-BF2F-21EE6D5D1878"); }
        }
    }
}