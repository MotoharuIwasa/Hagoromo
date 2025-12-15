using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace Hagoromo.GeometryTools
{
    public class MbridgeCM : GH_Component
    {
        public MbridgeCM()
          : base("MbridgeCM", "MbridgeCM",
              "If input is Mesh, outputs CutMesh — and vice versa",
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

            if (input is IGH_Goo goo)
            {
                // Mesh へのキャストを試す
                if (goo.CastTo(out Mesh mesh))
                {
                    CutMesh cutMesh = new CutMesh(mesh);
                    DA.SetData(0, new GH_CutMesh(cutMesh));
                }

                // CutMesh へのキャストを試す
                else if (goo.CastTo(out CutMesh cutMesh))
                {
                    Rhino.Geometry.Mesh newMesh = cutMesh.ConvertToMesh();
                    DA.SetData(0, newMesh);
                }
            }
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
            get { return new Guid("523CBF95-0AE9-4450-BA5F-43B8243B38E0"); }
        }
    }
}