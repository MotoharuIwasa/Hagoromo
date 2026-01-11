using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using Hagoromo.DevelopableMesh;
using System.Linq;
using static Hagoromo.GeometryTools.CutMeshCalcTools;

namespace Hagoromo.GeometryTools
{
    public class GetMainTree : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent5 class.
        /// </summary>
        public GetMainTree()
          : base("GetMainTree", "GetMainTree",
              "GetMainTree in CutMesh",
              "Hagoromo", "Geometry")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "CutMesh or Mesh", GH_ParamAccess.item);
            pManager.AddCurveParameter("Open Cut Lines (Polylines)", "L", "CutLines", GH_ParamAccess.list);
            pManager.AddBooleanParameter("IsTopological", "TP", "topological:true, length base:false", GH_ParamAccess.item,false);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Cut Choices", "CL", "cut edges", GH_ParamAccess.list);
        }
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Curve> curves = new List<Curve>();
            if (!DA.GetDataList(1, curves)) return;

            List<Polyline> cutLines = new List<Polyline>();

            foreach (var crv in curves)
            {
                if (crv.TryGetPolyline(out Polyline pl))
                {
                    cutLines.Add(pl);
                }
                else
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "One of the input curves is not a polyline.");
                }
            }

            object input = null;
            if (!DA.GetData(0, ref input)) return;

            if (input is IGH_Goo goo)
            {
                CutMesh cutMesh = new CutMesh();
                // Mesh へのキャストを試す
                if (goo.CastTo(out Mesh mesh))
                {
                    cutMesh = new CutMesh(mesh);
                    cutMesh = cutMesh.Sort();
                }

                // CutMesh へのキャストを試す
                else if (goo.CastTo(out CutMesh cutMesh2))
                {
                    cutMesh = cutMesh2.Sort();
                }



                List<int> edgeIndices = CrvToEdgeIndices(cutMesh, curves);
                bool mode = false;
                DA.GetData(2, ref mode);
                List<int> mainVert = CutChoiceTools.GetLongestTree(cutMesh, edgeIndices, mode);
                List<int> mainEdges = CutMeshCalcTools.PolyEdge(cutMesh, mainVert,false);
                List<Line> mainTree = new List<Line>();
                foreach (int i in mainEdges)
                {
                    mainTree.Add(cutMesh.GetEdgeLine(i));
                }
                DA.SetDataList(0, mainTree);
                
            }
        }

        /// <summary>
        /// Provides an Icon for the component.
        /// </summary>
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
            get { return new Guid("3EA25089-38CB-49B4-86FD-6A27DA60F80F"); }
        }
    }
}