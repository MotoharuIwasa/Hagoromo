using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Grasshopper.Kernel.Parameters;
using Rhino.Geometry;

namespace Hagoromo.GeometryTools
{
    public class OpenTreeMesh : GH_Component
    {
        public OpenTreeMesh()
          : base("OpenTreeMesh", "OpenTreeMesh",
              "OpenTreeMesh",
              "Hagoromo", "Geometry")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Triangulated Developable Mesh", "M", "mesh to develop", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("CutMesh", "CM", "cut mesh", GH_ParamAccess.item);
            pManager.AddCurveParameter("CutLines", "L", "cut lines", GH_ParamAccess.list);
            pManager.AddCurveParameter("MeshGraph", "T", "mesh graph", GH_ParamAccess.list);

        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Rhino.Geometry.Mesh mesh = null;
            if (!DA.GetData(0, ref mesh)) return;
            int[][] newConnectedFaces = MeshCutTools.MeshToOpenTree(mesh);
            List<int> cutEdgeIndices = MeshCutTools.CutEdgeIndices(mesh, newConnectedFaces);
            List<Line> cutLines = new List<Line>();
            for (int i = 0; i < cutEdgeIndices.Count; i++)
            {
                cutLines.Add(mesh.TopologyEdges.EdgeLine(cutEdgeIndices[i]));
            }

            CutMesh cutMesh = MeshCutTools.CutMeshWithEdgeIndices(mesh, cutEdgeIndices);
            GH_CutMesh ghCutMesh = new GH_CutMesh(cutMesh);
            List<Curve> treeCurves = MeshCutTools.MeshTreeCentersToCurves(mesh, newConnectedFaces);
            DA.SetData(0, ghCutMesh);
            DA.SetDataList(1, cutLines);
            DA.SetDataList(2, treeCurves);
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
            get { return new Guid("28AB11AB-853A-4747-813B-0249AFC971FE"); }
        }
    }
}