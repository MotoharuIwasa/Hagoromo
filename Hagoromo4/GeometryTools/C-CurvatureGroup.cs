using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;

using Rhino.Geometry;
using System;
using System.Collections.Generic;

using Grasshopper.Kernel.Types;
using static Hagoromo.GeometryTools.CutMeshCalcTools;
using static Hagoromo.DevelopableMesh.CutChoiceTools;
using System.Linq;

namespace Hagoromo.GeometryTools
{
    public class CurvatureGroup : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public CurvatureGroup()
          : base("CurvatureGroup", "CurvatureGroup",
              "CurvatureGroup",
              "Hagoromo", "Geometry")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "Mesh or CutMesh", GH_ParamAccess.item);
            pManager.AddIntegerParameter("i", "i", "i", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("Points", "V", "vertices", GH_ParamAccess.list);
            pManager.AddGenericParameter("TriangulatedCutMesh", "CM", "CutMesh", GH_ParamAccess.item);
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
            int i = 0;
            DA.GetData(1, ref i);

            double threshold = 0.01; // 形状に合わせて調整
            List<double> gaussMap = GaussianMap(cutMesh, 5).ToList();
            List<List<int>> clusters = FindSeparatedPeaksTwoPass(cutMesh, gaussMap, threshold);
            List<int> cluster = clusters[i % clusters.Count];
            List<Point3d> points = new List<Point3d>();
            foreach ( int vert in cluster) { points.Add(cutMesh.Vertices[vert]); }
            DA.SetDataList(0, points);

            //int startIndex = FindCentralVertexIndex(cutMesh,clusters[0]);
            //int endIndex = FindCentralVertexIndex(cutMesh, clusters[1]);
            int startIndex = clusters[0][0];
            int endIndex = clusters[1][0]; ;
            int thirdIndex = clusters[0][0]; ;

            List<double> edgeCost = new List<double>();
            for (int j = 0; j < cutMesh.Edges.Count; j++)
            {
                int[] edge = cutMesh.Edges[j];
                edgeCost.Add(cutMesh.GetEdgeLine(j).Length / (Math.Abs(gaussMap[edge[0]]) + Math.Abs(gaussMap[edge[1]])) + 0.0000001);
                //edgeCost.Add(cutMesh.GetEdgeLine(j).Length);
            }
            //List<int> path = FindShortestPathEdges(cutMesh, edgeCost, thirdIndex, endIndex);
            List<int> path = FindShortestPathToBoundary(cutMesh, edgeCost, startIndex);
            List<int> path2 = FindShortestPathToBoundary(cutMesh, edgeCost, endIndex);
            //List<int> path2 = FindShortestPathEdges(cutMesh, edgeCost, startIndex, thirdIndex);
            path.AddRange(path2);
            CutMesh newMesh = MeshCutTools.CutMeshWithEdgeIndices(cutMesh, path);
            DA.SetData(1, new GH_CutMesh(newMesh));
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
            get { return new Guid("DF5D95FE-6F59-453A-814D-8820D64A51C4"); }
        }
    }
}