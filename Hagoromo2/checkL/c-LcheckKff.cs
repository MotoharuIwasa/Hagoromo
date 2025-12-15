using System;

using Grasshopper.Kernel;
using Hagoromo.DataStructure;
using Hagoromo.SolverL;

namespace Hagoromo.CheckL
{
    public class LCheckKff : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public LCheckKff()
          : base("LCheckKff", "LCheckKff",
              "グローバルマトリクスの左上",
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
            pManager.AddTextParameter("MatrixText", "MT", "Matrix as string", GH_ParamAccess.list);
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
            double[,] matrix = LinearK.MakeModifiedGlobalK(falldata).Item1;
            //double[,] matrix = { { 1, 2, 3 }, { 4, 5, 6 } };
            string matrixStr = "";
            int rows = matrix.GetLength(0);
            int cols = matrix.GetLength(1);

            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < cols; j++)
                {
                    matrixStr += matrix[i, j].ToString("F8").PadLeft(15); // 右寄せで10桁分
                }
                matrixStr += "\n";
            }

            DA.SetData(0, matrixStr);

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
            get { return new Guid("60D4AEE9-6A9C-43FD-9EE3-2AB1899A93D2"); }
        }
    }
}