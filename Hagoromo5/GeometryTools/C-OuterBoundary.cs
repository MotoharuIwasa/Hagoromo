using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;

using Rhino.Geometry;
using System;
using System.Collections.Generic;

using Grasshopper.Kernel.Types;

namespace Hagoromo.GeometryTools
{
    public class OuterBoundary : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public OuterBoundary()
          : base("OuterBoundary", "OuterBoundary",
              "OuterBoundary",
              "Hagoromo", "Geometry")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "Mesh or CutMesh", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("outer edges", "E", "outer edges", GH_ParamAccess.list);
            pManager.AddIntegerParameter("outerVertsIndices", "V", "outer verts indices", GH_ParamAccess.list);
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
            var outer = CutMeshCalcTools.FindOuterBoundaryVerts(cutMesh);
            List<int> edges = outer.OuterEdges;
            DA.SetDataList(1, outer.OuterVerts);

            List<Line> nakedEdges = new List<Line>();
            for (int i = 0; i < edges.Count; i++)
            {
                nakedEdges.Add(cutMesh.GetEdgeLine(edges[i]));
            }
            DA.SetDataList(0, nakedEdges);

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
            get { return new Guid("0DB03E3C-F911-4C0D-A456-193C17907DFA"); }
        }
    }
}