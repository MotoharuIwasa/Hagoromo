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
    //もうこのコンポーネント不要。NetBFFをNetBFFF2に置き換え終わったらプロジェクトから除外する。
    public class NetBFF : GH_Component
    {
        public NetBFF()
          : base("NetBFF", "NetBFF",
              "Net by BFF",
              "Hagoromo", "Optimization")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "Mesh or CutMesh", GH_ParamAccess.item);
            pManager.AddCurveParameter("Feature Lines (Polylines)", "L", "Lines", GH_ParamAccess.list);
            pManager[1].Optional = true;

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
            
            CutMesh newMesh = cutMesh.Clone();
            cutMesh = cutMesh.Sort();
            newMesh.Vertices = NetTools.NetBFF(cutMesh);
            newMesh.DuplicatedVertIndices = Enumerable.Range(0, cutMesh.Vertices.Count)
                              .Select(i => new List<int> { i })
                              .ToList();
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
                return null;
            }
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("5B335F69-A190-45D5-875B-BD4DEA66A9EA"); }
        }
    }
}