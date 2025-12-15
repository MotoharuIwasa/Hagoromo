using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino.Render.ChangeQueue;
using System;
using System.Collections.Generic;
using Hagoromo.GeometryTools;

namespace Hagoromo.DevelopableMesh
{
    public class GaussianCrv : GH_Component
    {
        public GaussianCrv()
          : base("GaussianCurvature", "GCurvature",
              "Description",
              "Hagoromo", "Optimization")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Triangulated Mesh", "M", "mesh to develop", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("a", "a", "a", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Rhino.Geometry.Mesh mesh = null;
            if (!DA.GetData(0, ref mesh)) return;
            List<int> internalVertexIndices = MeshDataTools.TopoInternalVertIndices(mesh);

            //ガウス曲率の2乗のリストを作成
            List<double> crvSum = new List<double> { };
            int count = internalVertexIndices.Count;
            for (int i = 0; i < count; i++)
            {
                double curvatureTwo = CurvatureTools.CurvatureTwo(mesh, internalVertexIndices[i]);
                crvSum.Add(curvatureTwo);
            }
            DA.SetDataList(0, crvSum);
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
            get { return new Guid("2FA3CFF1-C5E3-4453-9323-3EC9F5A53EFB"); }
        }
    }
}