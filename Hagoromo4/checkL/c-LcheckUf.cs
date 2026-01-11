using System;

using Grasshopper.Kernel;
using Hagoromo.DataStructure;
using Hagoromo.SolverL;
using Hagoromo.MathTools;

namespace Hagoromo.CheckL
{
    public class LCheckUf : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public LCheckUf()
          : base("LCheckUf", "LCheckUf",
              "debug用でした。用済み。",
              "Hagoromo", "debug")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Alldata", "A", "Combined data package", GH_ParamAccess.item);
            //pManager.AddIntegerParameter("index", "I", "elem index", GH_ParamAccess.item, 0);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddTextParameter("VectorText", "Vec", "Vector as string", GH_ParamAccess.list);
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
            double[] pf = SolveLinear.GetFalsePart(falldata.fNodeVConArray, falldata.fNodeIConArray);
            double[,] k_ff = LinearK.MakeModifiedGlobalK(falldata).Item1;
            //double[] k_ffd = MatrixUtils.ExtractLowerTriangle(k_ff);
            double[,] inv_kff = MatrixUtils.InverseMatrix(k_ff);
            //double[] array = SLEsSolvers.SolveByLDL(k_ffd, pf);/*U_f*/
            double[] array = MatrixUtils.Multiply(inv_kff, pf);/*U_f*/

            string arrayStr = "";
            for (int i = 0; i < array.Length; i++)
            {
                arrayStr += array[i].ToString("F8").PadLeft(15); // 小数点8桁、右寄せ
                arrayStr += "\n"; // 1行ずつ表示（縦に並べる）
            }
            DA.SetData(0, arrayStr);


        }

        /// <summary>
        /// Provides an Icon for the component.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                //You can add image files to your project resources and access them like this:
                // return Resources.IconForThisComponent;
                return null;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("ED7D80C9-0C84-46D3-976B-D3AD8472B27B"); }
        }
    }
}