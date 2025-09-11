using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Hagoromo
{
    public class LinearBeam1 : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public LinearBeam1()
          : base("LinearBeam1", "Nickname",
              "軸、曲げ、サンブナンねじりのみ考慮(Hoganと同じ)の線形解析",
              "Category", "Subcategory")
        {
        }

        
        
        //cmqの考慮
        //brepからelem情報を作成するコンポーネントを作る。coord angleなどを自動で出せるようにしたい。
        //自重の曲げ考慮
        //剛性マトリクスの作成を効率よく、連立一次方程式の解き方





        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {

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
            get { return new Guid("F6B3F263-6011-418C-ABF6-A61889AE225D"); }
        }
    }
}