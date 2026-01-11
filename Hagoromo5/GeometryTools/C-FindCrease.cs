using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using Hagoromo.DevelopableMesh;

namespace Hagoromo.GeometryTools
{
    public class FindCrease : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent4 class.
        /// </summary>
        public FindCrease()
          : base("FindCrease", "FindCrease",
              "FindCrease",
              "Hagoromo", "Geometry")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "CutMesh or Mesh", GH_ParamAccess.item);
            pManager.AddNumberParameter("curvature border", "w", "curvature border", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("creases", "c", "creases", GH_ParamAccess.list);
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
                    cutMesh = cm.Clone();
                }
            }

            if (cutMesh == null)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Mesh または CutMesh を入力してください");
                return;
            }

            double border = 0;
            DA.GetData(1, ref border);

            double[] principle = CutChoiceTools.BiggerAbsPrinciple(cutMesh);
            double[] edgeCurvature = new double[cutMesh.Edges.Count];
            int j = 0;
            foreach (int[] edge in cutMesh.Edges)
            {
                edgeCurvature[j] = principle[edge[0]] + principle[edge[1]];
                j++;
            }
            List<Line> lines = new List<Line>();
            for (int i = 0; i < cutMesh.Edges.Count; i++)
            {
                if (edgeCurvature[i] > border) { lines.Add(cutMesh.GetEdgeLine(i)); }
            }

            DA.SetDataList(0, lines);
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
            get { return new Guid("8CC3AB60-0D11-4B5D-BBDB-CC811EADA68D"); }
        }
    }
}