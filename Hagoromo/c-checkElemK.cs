using System;
using System.Collections.Generic;
using System.Drawing;

using Grasshopper.Kernel;
using Rhino.Geometry;
using Hagoromo.DataStructure;
using Hagoromo.MathTools;
using Hagoromo.StiffnessTools;
using Grasshopper.Kernel.Types.Transforms;

namespace Hagoromo
{
    public class CheckElemK : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public CheckElemK()
          : base("CheckElemK", "CheckKe",
              "座標変換前の要素剛性マトリクス",
              "Hagoromo", "Linear")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Alldata", "A", "Combined data package", GH_ParamAccess.item);
            pManager.AddIntegerParameter("index", "I", "elem index", GH_ParamAccess.item, 0);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddTextParameter("MatrixText", "MT", "Matrix as string", GH_ParamAccess.item);
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

            int index = 0;
            if (!DA.GetData(1, ref index))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Scale value could not be retrieved.");
                return;
            }

            // Alldata 型に変換
            Alldata alldata = goo.Value;
            fAlldata falldata = new fAlldata(alldata);
            double[,] matrix = LinearK.MakeKe(falldata,index);
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
            get { return new Guid("0F6CA5B6-990F-47A8-B4C3-2EEB75412D49"); }
        }
    }
}