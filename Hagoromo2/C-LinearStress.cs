using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Hagoromo.DataStructure;
using Hagoromo.MathTools;
using Hagoromo.StiffnessTools;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Drawing;


namespace Hagoromo.Solver
{
    
    public class LinearDisp : GH_Component
    {
        public Alldata previousalldata = new Alldata();
        double[] U = new double[] { };
        public Vector3d previousgravity_vec = new Vector3d { };

        public LinearDisp()
          : base("Linear Analysis Disp", "LDisp",
              "displacement by linear analysis.重力は両節点に半分ずつ分配しモーメントは考えない。" +
                "変位の単位は10^Lp mm、角度の単位はrad",
              "Hagoromo", "Linear")
        {
        }

        
        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Alldata", "A", "Combined data package", GH_ParamAccess.item);
            pManager.AddNumberParameter("Scale", "S", "display scale of disp", GH_ParamAccess.item, 1);
            pManager.AddVectorParameter("体積力の加速度ベクトルの和", "g", "体積力(重力)加速度ベクトル modified gを入れる。", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddLineParameter("Deformed", "D", "Deformed frame lines", GH_ParamAccess.list);
            pManager.AddColourParameter("Colors", "C", "Line colors based on displacement", GH_ParamAccess.list);
            pManager.AddVectorParameter("DispVec", "V", "Displacement vector in order of nodeid", GH_ParamAccess.list);
            pManager.AddNumberParameter("AllDisp", "AD", "xyzとその軸周りの回転角", GH_ParamAccess.tree);
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



            double scale = 1.0; // 初期値を設定（オプション）

            if (!DA.GetData(1, ref scale))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Scale value could not be retrieved.");
                return;
            }

            Vector3d gravity_vec = Vector3d.Zero;
            if (!DA.GetData(2, ref gravity_vec))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Gravity value could not be retrieved.");
                return;
            }



            fAlldata falldata = new fAlldata(previousalldata);
            double[] gravity = new double[3];
            if (previousalldata != alldata || previousgravity_vec != gravity_vec)
            {
                falldata = new fAlldata(alldata);
                gravity = new double[3] { gravity_vec.X, gravity_vec.Y, gravity_vec.Z };
                U = SolveLinear.SolveDisp(falldata,gravity);
                previousalldata = alldata;
                previousgravity_vec = gravity_vec;
            }

            DataTree<double> treeU = new DataTree<double>();
            for (int i = 0; i < U.Length; i += 6)
            {
                GH_Path path = new GH_Path(i / 6); // 各6個セットに対してブランチを作成

                for (int j = 0; j < 6; j++)
                {
                    treeU.Add(U[i + j], path);
                }
            }



            List<Vector3d> disps_ = new List<Vector3d>();
            for (int i = 0; i < U.Length; i += 6)
            {
                // 範囲外参照を防ぐためのチェック（U.Length が 3 の倍数である前提なら不要）
                /*if (i + 2 >= U.Length) break;*/

                Vector3d v = new Vector3d(U[i], U[i + 1], U[i + 2]);
                disps_.Add(v);
            }
            Vector3d[] disps = disps_.ToArray();
            Point3d[] points = ConvertToPoints(falldata.fNodeXYZArray);
            for (int i = 0; i < points.Length; i++)
            {
                points[i] += disps[i] * scale;
            }

            int count = alldata.ElementArray.GetLength(0);
            Line[] deformed = new Line[count];
            Color[] colors = new Color[count];

            for (int i = 0; i < alldata.ElementArray.GetLength(0); i++)
            {
                int nodeid_s = (int)fAlldata.GetElemItem(falldata.fElemArray, i, 1, 0) - 1;
                int nodeid_e = (int)fAlldata.GetElemItem(falldata.fElemArray, i, 1, 1) - 1;
                deformed[i] = new Line(points[nodeid_s], points[nodeid_e]);
                double mag = 0.5 * (disps[nodeid_s].Length + disps[nodeid_e].Length);
                colors[i] = DisplacementColor(mag);
            }

            DA.SetDataList(0, deformed);
            DA.SetDataList(1, colors);
            DA.SetDataList(2, disps);
            DA.SetDataTree(3, treeU);
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
        private Color DisplacementColor(double magnitude)
        {
            double t = Math.Min(1.0, magnitude / 10.0); // スケーリング
            int r = (int)(255 * t);
            int b = 255 - r;
            return Color.FromArgb(r, 0, b);
        }

        // Point3d 比較（Hash-based）
        private class PointEqualityComparer : IEqualityComparer<Point3d>
        {
            public bool Equals(Point3d a, Point3d b) =>
                a.DistanceToSquared(b) < 1e-8;

            public int GetHashCode(Point3d pt) =>
                pt.X.GetHashCode() ^ pt.Y.GetHashCode() ^ pt.Z.GetHashCode();
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
            get { return new Guid("25E811D4-5682-4A1B-8759-7F08CC270F79"); }
        }
    }
}