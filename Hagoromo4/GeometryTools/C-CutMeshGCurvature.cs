using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using System;
using static Hagoromo.GeometryTools.CutMeshCalcTools;

namespace Hagoromo.GeometryTools
{
    public class CutMeshGCurvature : GH_Component
    {
        public CutMeshGCurvature()
          : base("CutMeshGCurvature", "CutMeshGCurvature",
              "CutMeshGCurvature",
              "Hagoromo", "Geometry")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "Mesh or CutMesh", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("CutMesh GCurvature", "GC", "CutMesh GCurvature", GH_ParamAccess.list);
            pManager.AddNumberParameter("CutMesh GCurvature", "GCMap", "CutMesh GCurvature", GH_ParamAccess.list);
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
            double[] gc = GaussianCurvature(cutMesh);
            double[] gcMap = GaussianMap(cutMesh, 5);
            DA.SetDataList(0, gc);
            DA.SetDataList(1, gcMap);
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
            get { return new Guid("ED902EC7-5633-4036-9DEC-85945DB5A11E"); }
        }
    }
}