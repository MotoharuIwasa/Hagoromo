using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace Hagoromo.GeometryTools
{
    public class CutMeshEdges : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public CutMeshEdges()
          : base("CutMeshEdges", "CutMeshEdges",
              "CutMeshEdges",
              "Hagoromo", "Geometry")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "CutMesh or Mesh", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Mesh All Edges", "AE", "all edges", GH_ParamAccess.list);
            pManager.AddCurveParameter("Mesh Naked Edges", "NE", "naked edges", GH_ParamAccess.list);
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



            List<Line> allEdges = new List<Line>();
            for (int i = 0; i < cutMesh.Edges.Count; i++)
            {
                allEdges.Add(cutMesh.GetEdgeLine(i));
            }

            List<Line> nakedEdges = new List<Line>();
            List<int> boundaryIndices = cutMesh.BoundaryEdgeIndices();
            for (int i = 0; i < boundaryIndices.Count; i++)
            {
                nakedEdges.Add(cutMesh.GetEdgeLine(boundaryIndices[i]));
            }
            DA.SetDataList(0, allEdges);
            DA.SetDataList(1, nakedEdges);
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
            get { return new Guid("AFE35A7A-3707-40DC-A102-C6B3F11C8AFC"); }
        }
    }
}