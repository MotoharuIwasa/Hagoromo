using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Hagoromo.GeometryTools;
using Hagoromo.MathTools;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Collections;
using Rhino.Render.ChangeQueue;
using System;
using System.Collections.Generic;
using System.Linq;
using static Hagoromo.GeometryTools.CutMeshCalcTools;
using static Hagoromo.DevelopableMesh.DevelopableTools;

namespace Hagoromo.DevelopableMesh
{
    public class DevelopCutMesh2 : GH_Component
    {
        public DevelopCutMesh2()
          : base("Develop CutMesh2", "DevelopCM2",
              "CGによる可展面近似。ミラー、切込みに対応している。すぐ発散したりする",
              "Hagoromo", "Optimization")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated CutMesh", "(C)M", "mesh to develop", GH_ParamAccess.item);
            pManager.AddIntegerParameter("iterations", "i", "iteration", GH_ParamAccess.item);
            pManager.AddBooleanParameter("xyMirror", "xy", "xyMirror", GH_ParamAccess.item,false);
            pManager.AddBooleanParameter("yzMirror", "yz", "yzMirror", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("zxMirror", "zx", "zxMirror", GH_ParamAccess.item, false);
            pManager.AddCurveParameter("outerBoundaryCurves", "outerB", "outerBoundaryCurves", GH_ParamAccess.list);
            int fixCrvIndex = pManager.AddCurveParameter("fixPointsCrv", "fixCrv", "fixPointsCrv", GH_ParamAccess.list);
            pManager[6].Optional = true;
            pManager.AddNumberParameter("pace", "alpha", "pace", GH_ParamAccess.item, 0.1);
        }


        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            //pManager.AddMeshParameter("Developed Mesh", "M", "developed planer mesh", GH_ParamAccess.item);
            pManager.AddGenericParameter("Modified CutMesh", "(C)M", "modified CutMesh", GH_ParamAccess.item);
            //pManager.AddIntegerParameter("a", "a", "a", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            object input = null;
            if (!DA.GetData(0, ref input)) return;

            CutMesh cutMesh = null;

            if (input is IGH_Goo goo)
            {
                // Mesh へのキャストを試す
                if (goo.CastTo(out Rhino.Geometry.Mesh m))
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
            cutMesh = cutMesh.Sort();
            int iterations = 0;
            DA.GetData(1, ref iterations);
            bool xyMirror = false;
            DA.GetData(2, ref xyMirror);
            bool yzMirror = false;
            DA.GetData(3, ref yzMirror);
            bool zxMirror = false;
            DA.GetData(4, ref zxMirror);
            double alpha = 0;
            DA.GetData(7, ref alpha);

            List<Curve> outerCrvs = new List<Curve>();
            DA.GetDataList(5, outerCrvs);
            List<int> sortedOuterVertIndices = CrvToVertIndices(cutMesh, outerCrvs);

            List<Curve> fixCrvs = new List<Curve>();
            bool hasInput = DA.GetDataList(6, fixCrvs);
            List<int> sortedFixVertIndices = CrvToVertIndices(cutMesh, fixCrvs);

            // 以降 cutMesh が確実に利用可能
            CutMesh newMesh = CGDevCutMeshConsiderOther(cutMesh, iterations, alpha, xyMirror, yzMirror, zxMirror, sortedOuterVertIndices, sortedFixVertIndices);
            DA.SetData(0, new GH_CutMesh(newMesh));
            //DA.SetDataList(0, sortedOuterVertIndices);
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
            get { return new Guid("F35A696E-2EC2-405E-8105-19610755110F"); }
        }
    }
}