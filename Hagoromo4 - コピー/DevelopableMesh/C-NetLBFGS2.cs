using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Hagoromo.GeometryTools;
using Hagoromo.MathTools;
using Rhino.Geometry;
using Rhino.Input.Custom;
using Rhino.Render.ChangeQueue;
using System;
using System.Collections.Generic;
using System.Linq;
using static Hagoromo.DevelopableMesh.NetTools;
using static Hagoromo.GeometryTools.CutMeshCalcTools;

namespace Hagoromo.DevelopableMesh
{
    public class NetLBFGS2 : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public NetLBFGS2()
          : base("NetLBFGS2", "NetLBFGS2",
              "TutteMeshのあと長さ保存、符号付面積>0制約最適化。最初falseでやってw調整しておおまかOKになったらそのwのままtrueにするといい展開図得られる。",
              "Hagoromo", "Optimization")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "Mesh or CutMesh", GH_ParamAccess.item);
            pManager.AddCurveParameter("Feature Lines (Polylines)", "L", "Lines", GH_ParamAccess.list);
            pManager[1].Optional = true;
            pManager.AddNumberParameter("w0", "w0", "length preservation weight", GH_ParamAccess.item, 1.0);
            pManager.AddNumberParameter("w1", "w1", "avoid face flip weight", GH_ParamAccess.item, 1.0);
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
            double[] w = new double[] { 1.0, 1.0 };
            DA.GetData(2, ref w[0]);
            DA.GetData(3, ref w[1]);

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
                else if (goo.CastTo(out CutMesh mesh2))
                {
                    cutMesh = mesh2.Sort();
                }
                else { return; }
            }

            List<Curve> curves = new List<Curve>();
            DA.GetDataList(1, curves);
            List<int> edgeIndices = CrvToEdgeIndices(cutMesh, curves);
            List<Line> lines = new List<Line>();

            CutMesh newMesh = CutMeshNetLBFGS2(cutMesh, w[0], w[1]);
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

        public override Guid ComponentGuid
        {
            get { return new Guid("805BC0D3-0307-44A1-AC24-AA4E081CAD1E"); }
        }
    }
}