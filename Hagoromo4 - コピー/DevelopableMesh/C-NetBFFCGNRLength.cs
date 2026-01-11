using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Hagoromo.GeometryTools;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using static Hagoromo.GeometryTools.CutMeshCalcTools;

namespace Hagoromo.DevelopableMesh
{
    public class NetBFFLength2 : GH_Component
    {
        public NetBFFLength2()
          : base("NetBFFLength2", "NetBFFLength2",
              "穴あり対応BFFした後に長さ保存のCGNR法",
              "Hagoromo", "Optimization")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "Mesh or CutMesh", GH_ParamAccess.item);
            pManager.AddCurveParameter("Feature Lines (Polylines)", "L", "Lines", GH_ParamAccess.list);
            pManager[1].Optional = true;
            pManager.AddIntegerParameter("OuterBoundaryVertCount", "C", "OuterBoundaryVertCount", GH_ParamAccess.item, -1);
            pManager[2].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("cutMesh", "CM", "CM", GH_ParamAccess.item);
            pManager.AddCurveParameter("Feature Lines (Polylines)", "L", "Lines", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            int outerBoundaryVertCount = -1;
            DA.GetData(2, ref outerBoundaryVertCount);
            object input = null;
            CutMesh cutMesh = new CutMesh();
            if (!DA.GetData(0, ref input)) return;
            if (!(input is IGH_Goo goo2)) return;
            if (input is IGH_Goo goo)
            {
                // Mesh へのキャストを試す
                if (goo.CastTo(out Rhino.Geometry.Mesh m))
                {
                    cutMesh = new CutMesh(m);
                    cutMesh = cutMesh.Sort();
                }
                else if (goo.CastTo(out CutMesh mesh))
                {
                    cutMesh = mesh.Sort();
                }
                else { return; }
            }

            List<Curve> curves = new List<Curve>();
            DA.GetDataList(1, curves);
            List<int> edgeIndices = CrvToEdgeIndices(cutMesh, curves);

            CutMesh newMesh = NetTools.NetBFFandCGNRLength(cutMesh, outerBoundaryVertCount);
            GH_CutMesh ghCutMesh = new GH_CutMesh(newMesh);
            List<Line> lines = new List<Line>();
            foreach (int edgeIndex in edgeIndices)
            {
                lines.Add(newMesh.GetEdgeLine(edgeIndex));
            }
            DA.SetData(0, new GH_CutMesh(newMesh));
            DA.SetDataList(1, lines);
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

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("BEA607A5-AA2E-4C50-9046-66CD8570E6F6"); }
        }
    }
}