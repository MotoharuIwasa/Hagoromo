using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using static Hagoromo.GeometryTools.CutMeshCalcTools;

namespace Hagoromo.GeometryTools
{
    public class MeshCutWithCutLines : GH_Component
    {
        public MeshCutWithCutLines()
          : base("MeshCut with CutLines", "MeshCutbyLines",
              "Cut (Cut)Mesh with CutLines",
              "Hagoromo", "Geometry")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "CutMesh or Mesh", GH_ParamAccess.item);
            pManager.AddCurveParameter("Open Cut Lines (Polylines)", "L", "CutLines", GH_ParamAccess.list);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("TriangulatedCutMesh", "CM", "CutMesh", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Curve> curves = new List<Curve>();
            if (!DA.GetDataList(1, curves)) return;

            object input = null;
            if (!DA.GetData(0, ref input)) return;

            CutMesh cutMesh = new CutMesh();
            if (input is IGH_Goo goo)
            {
                // Mesh へのキャストを試す
                if (goo.CastTo(out Mesh mesh))
                {
                    cutMesh = new CutMesh(mesh);
                }
                // CutMesh へのキャストを試す
                else if (goo.CastTo(out CutMesh cutMesh2))
                {
                    cutMesh = cutMesh2.Clone();
                }

                List<int> edgeIndices = CrvToEdgeIndices(cutMesh, curves);
                List<Polyline> cutLines = new List<Polyline>();
                CutMesh newMesh = MeshCutTools.CutMeshWithEdgeIndices(cutMesh, edgeIndices);
                DA.SetData(0, new GH_CutMesh(newMesh));
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
            get { return new Guid("F53ADF39-2D5B-466F-97A2-C7564EFB361E"); }
        }
    }
}