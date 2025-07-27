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
    
    public class LinearAnalysis : GH_Component
    {
        public Alldata previousalldata = new Alldata();
        double[] U = new double[] { };
        double[,] p = new double[,] { };
        public Vector3d previousgravity_vec = new Vector3d { };

        public LinearAnalysis()
          : base("Linear Analysis", "LA",
              "solved by linear analysis.重力は両節点に半分ずつ分配しモーメントは考えない。" +
                "Uは変位の単位は10^Lp mm、角度の単位はrad。pは部材座標系でのxyz方向の応力とその軸まわりのモーメントの部材始端6個の後終端6個の順,単位は10^Fp N,10^(Fp+Lp) Nmm",
              "Hagoromo", "Linear")
        {
        }

        
        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Alldata", "A", "Combined data package", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("AllDisplacement", "U", "xyzとその軸周りの回転角", GH_ParamAccess.tree);
            pManager.AddNumberParameter("AllStress", "p", "xyzとその軸周りのモーメント", GH_ParamAccess.tree);
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
            fAlldata falldata = new fAlldata(previousalldata);

            if (previousalldata != alldata)
            {
                falldata = new fAlldata(alldata);
                U = SolveLinear.SolveDisp(falldata);
                p = SolveLinear.SolveStress(falldata, U);
                previousalldata = alldata;
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

            DataTree<double> treeP = new DataTree<double>();

            int rowCount = p.GetLength(0);
            int colCount = p.GetLength(1);

            for (int i = 0; i < rowCount; i++)
            {
                GH_Path path = new GH_Path(i); // 各行に対してブランチを作成

                for (int j = 0; j < colCount; j++)
                {
                    treeP.Add(p[i, j], path);
                }
            }


            DA.SetDataTree(0, treeU);
            DA.SetDataTree(1, treeP);
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
            get { return new Guid("6EE81898-0344-4F82-B08B-8EF0D6EBDFC8"); }
        }
    }
}



