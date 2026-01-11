using System;
using System.Collections.Generic;
using System.Drawing;

using Grasshopper.Kernel;
using Rhino.Geometry;
using Hagoromo.DataStructure;
using Hagoromo.SolverL;
using Hagoromo.MathTools;
using Grasshopper.Kernel.Types.Transforms;

namespace Hagoromo.CheckL
{
    public class LCheckPf : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public LCheckPf()
          : base("LCheckPf", "LCheckPf",
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
            double[] array = SolveLinear.GetFalsePart(falldata.fNodeVConArray, falldata.fNodeIConArray);
            //double[,] matrix = { { 1, 2, 3 }, { 4, 5, 6 } };

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
            get { return new Guid("AA192CE2-220D-4D9A-AD9F-9613BA36DBBA"); }
        }
    }
}