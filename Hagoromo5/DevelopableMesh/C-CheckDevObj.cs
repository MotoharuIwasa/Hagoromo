using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Hagoromo.GeometryTools;
using Hagoromo.MathTools;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using static Hagoromo.DevelopableMesh.CutChoiceTools;
using static Hagoromo.GeometryTools.CutMeshCalcTools;
using static Hagoromo.GeometryTools.MeshCutTools;

namespace Hagoromo.DevelopableMesh
{
    public class CheckDevObj : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public CheckDevObj()
          : base("CheckDevObj", "CheckDevObj",
              "CheckDevObj",
              "Hagoromo", "Optimization")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "Mesh or CutMesh", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "Mesh or CutMesh", GH_ParamAccess.item);
            pManager.AddNumberParameter("obj", "obj", "obj", GH_ParamAccess.item);
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

            double threshold = 0.01; // 形状に合わせて調整
            List<double> gaussMap = GaussianMap(cutMesh, 5).ToList();
            List<List<int>> clusters = FindSeparatedPeaksTwoPass(cutMesh, gaussMap, threshold);
            List<int> represents = clusters.Select(c => c[0]).ToList();

            List<double> edgeCost = new List<double>();
            for (int j = 0; j < cutMesh.Edges.Count; j++)
            {
                int[] edge = cutMesh.Edges[j];
                edgeCost.Add(cutMesh.GetEdgeLine(j).Length / (Math.Abs(gaussMap[edge[0]]) + Math.Abs(gaussMap[edge[1]])) + 0.0000001);
                //edgeCost.Add(cutMesh.GetEdgeLine(j).Length);
            }
            List<List<int>> path = new List<List<int>>();
            foreach (int vert in represents) { path.Add(FindShortestPathToBoundary(cutMesh, edgeCost, vert)); }
            for (int i = 0; i < represents.Count; i++)
            {
                for (int j = i + 1; j < represents.Count; j++)
                {
                    path.Add(FindShortestPathEdges(cutMesh, edgeCost, represents[i], represents[j]));
                }
            }
            List<double> edgeLength = new List<double>();
            for (int i = 0; i < cutMesh.Edges.Count; i++)
            {
                edgeLength.Add(cutMesh.GetEdgeLine(i).Length);
            }

            int pathCount = path.Count;
            if (pathCount == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "カットパスが見つかりませんでした。閾値やメッシュを確認してください。");
                return;
            }
            bool[] isCut = new bool[path.Count];
            for (int i = 0; i < isCut.Length; i++)
            {
                isCut[i] = false;
            }

            double[] a = TotalLengthAndObj(cutMesh, path, edgeLength, isCut);
            HashSet<int> edges = new HashSet<int>();
            for (int i = 0; i < isCut.Length; i++)
            {
                if (!isCut[i]) continue;
                List<int> route = path[i];
                edges.UnionWith(route);
            }
            CutMesh cutMesh2 = CutMeshWithEdgeIndices(cutMesh, edges.ToList());
            DA.SetData(0, new GH_CutMesh(cutMesh2));
            DA.SetData(1, a[1]);
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
            get { return new Guid("F9304987-ABB6-4A32-A6B4-A45BB9343168"); }
        }
    }
}