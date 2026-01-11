using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Hagoromo.GeometryTools;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Hagoromo.DevelopableMesh
{
    public class PrincipleCurvature : GH_Component
    {
        public PrincipleCurvature()
          : base("PrincipleCurvature", "Principle",
              "Bigger Principle Curvature absolution",
              "Hagoromo", "Optimization")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "Mesh or CutMesh", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Principle Curvature", "C", "PC", GH_ParamAccess.list);
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

            // 以降 cutMesh が確実に利用可能

            double[,] principle = CutMeshCalcTools.PrincipleCurvature(cutMesh);
            double[] big = new double[principle.GetLength(0)];
            for (int i = 0; i <  principle.GetLength(0); i++)
            {
                double a = Math.Abs(principle[i,0]);
                double b = Math.Abs(principle[i,1]);
                big[i]  = Math.Max(a, b);
            }
            double[] bigger = CutChoiceTools.BiggerAbsPrinciple(cutMesh);
            DA.SetDataList(0, bigger);
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
            get { return new Guid("C74ECF16-3006-409D-8854-F8021334931A"); }
        }
    }
}