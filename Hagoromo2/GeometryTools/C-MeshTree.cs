using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Hagoromo.GeometryTools
{
    public class MeshTree : GH_Component
    {
        public MeshTree()
          : base("MeshTree", "MeshTree",
              "Description",
              "Hagoromo", "Optimization")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Triangulated Developable Mesh", "M", "mesh to develop", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddLineParameter("Cut Lines", "C", "cut lines", GH_ParamAccess.list);
            pManager.AddCurveParameter("Mesh Tree", "T", "cut mesh tree", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Rhino.Geometry.Mesh mesh = null;
            if (!DA.GetData(0, ref mesh)) return;
            int[][] newConnectedFaces = MeshCutTools.MeshToOpenTree(mesh);

            List<Line> cutLines = MeshCutTools.MeshTreeToCurves(mesh, newConnectedFaces);
            List<Curve> treeCurves = MeshCutTools.MeshTreeCentersToCurves(mesh, newConnectedFaces);
            DA.SetDataList(0, cutLines);
            DA.SetDataList(1, treeCurves);
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
            get { return new Guid("6F61723A-5656-4DD0-8DBC-2189E3ACCC00"); }
        }
    }
}