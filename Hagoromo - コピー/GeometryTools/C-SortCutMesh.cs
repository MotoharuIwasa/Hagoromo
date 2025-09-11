using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace Hagoromo.GeometryTools
{
    public class SortCutMesh : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public SortCutMesh()
          : base("SortCutMesh", "SortCutMesh",
              "SortCutMesh",
              "Hagoromo", "Geometry")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "CutMesh or Mesh", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "CutMesh or Mesh", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            object input = null;
            if (!DA.GetData(0, ref input)) return;

            CutMesh cutMesh = null;

            if (input is IGH_Goo goo)
            {
                // Mesh へのキャストを試す
                if (goo.CastTo(out Mesh m))
                {
                    cutMesh = new CutMesh(m);
                }
                // CutMesh へのキャストを試す
                else if (goo.CastTo(out CutMesh cm))
                {
                    cutMesh = cm;
                }
            }

            if (cutMesh == null)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Mesh または CutMesh を入力してください");
                return;
            }

            // 以降 cutMesh が確実に利用可能
            CutMesh sortMesh = cutMesh.Sort();
            GH_CutMesh gH_CutMesh = new GH_CutMesh(sortMesh);
            DA.SetData(0,gH_CutMesh);
        }

        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                //You can add image files to your project resources and access them like this:
                // return Resources.IconForThisComponent;
                return null;
            }
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("5A328EE8-C0A0-40C3-8FFA-39BCEC056E41"); }
        }
    }
}