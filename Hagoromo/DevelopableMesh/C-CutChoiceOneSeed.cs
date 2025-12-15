using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Hagoromo.GeometryTools;
using Hagoromo.MathTools;
using MathNet.Numerics;
using MathNet.Numerics.Random;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Collections;
using Rhino.Render.ChangeQueue;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Hagoromo.DevelopableMesh
{
    public class CutChoiceOneSeed : GH_Component
    {
        public CutChoiceOneSeed()
          : base("OneSeedCut", "OneSeedCut",
              "Description",
              "Hagoromo", "Optimization")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated CutMesh", "(C)M", "mesh to develop", GH_ParamAccess.item);
            pManager.AddNumberParameter("maxLength", "L", "max length", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Branch", "Branch", "branch", GH_ParamAccess.item, false);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated CutMesh", "(C)M", "mesh to develop", GH_ParamAccess.item);
            pManager.AddCurveParameter("Cut Choices", "CL", "cut edges", GH_ParamAccess.list);
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

            double maxLength = 0;
            if (!DA.GetData(1, ref maxLength)) return;
            bool branch = false;
            if (!DA.GetData(2, ref branch)) return;

            List<int> cutChoices = new List<int>();
            if (branch) { cutChoices = CutChoiceTools.AvoidLoopMultiBranchesOneSeed(cutMesh, maxLength); }
            else { cutChoices = CutChoiceTools.AvoidLoopOneBranchOneSeed(cutMesh, maxLength); }
            CutMesh newMesh = MeshCutTools.CutMeshWithEdgeIndices(cutMesh, cutChoices);
            DA.SetData(0, new GH_CutMesh(newMesh));
            List<Line> cutEdges = new List<Line>();
            foreach (int i in cutChoices)
            {
                cutEdges.Add(cutMesh.GetEdgeLine(i));
            }
            DA.SetDataList(1, cutEdges);
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
            get { return new Guid("B6466813-99B5-4320-9EAF-02726CEF8BFB"); }
        }
    }
}