using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Hagoromo.MathTools;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using static Hagoromo.DevelopableMesh.CutChoiceTools;
using static Hagoromo.GeometryTools.MeshCutTools;
using static Hagoromo.GeometryTools.CutMeshCalcTools;

namespace Hagoromo.GeometryTools
{
    public class ClusterCut : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public ClusterCut()
          : base("ClusterCut", "ClusterCut",
              "ClusterCut",
              "Hagoromo", "Optimization")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "Mesh or CutMesh", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "Mesh or CutMesh", GH_ParamAccess.list);
            pManager.AddNumberParameter("TotalLength", "L", "total length", GH_ParamAccess.list);
            pManager.AddNumberParameter("DevObj", "O", "development objective", GH_ParamAccess.list);
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
            for (int i = 0;  i < cutMesh.Edges.Count ; i++)
            {
                edgeLength.Add(cutMesh.GetEdgeLine(i).Length);
            }

            int pathCount = path.Count;
            if (pathCount == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "カットパスが見つかりませんでした。閾値やメッシュを確認してください。");
                return;
            }
            bool[] isCut = new bool[pathCount];
            var optimizer = new LargeScaleHLOptimizer<CutChoiceVariable>();

            //設計変数の設定
            var configs = new Dictionary<string, VariableConfig>
            {
                // bool[] 用の設定
                { "isCut", VariableConfig.CreateBinary(pathCount) },
            };

            // ClusterCut.cs 内の myObjective
            Func<CutChoiceVariable, double[]> myObjective = (sol) =>
            {
                // ガード1: インスタンスや配列自体が null の場合
                if (sol == null || sol.isCut == null)
                    return new double[] { 1e10, 1e10 };

                // ガード2: 配列の長さが想定（pathCount）と一致しない場合
                // HeuristicLabの初期化時に長さ0の配列が渡されるケースへの対策
                if (sol.isCut.Length != path.Count)
                    return new double[] { 1e10, 1e10 };

                try
                {
                    double[] objs = TotalLengthAndObj(cutMesh, path, edgeLength, sol.isCut);
                    return objs ?? new double[] { 1e10, 1e10 };
                }
                catch (Exception)
                {
                    // 内部で計算エラーが起きても止まらないようにする
                    return new double[] { 1e10, 1e10 };
                }
            };



            //実行
            var results = optimizer.Run(
                objectiveFunc: myObjective,
                configs: configs,
                popSize: 3,
                generations: 3
            );

            //結果の表示
            var answers = ResultReview.GetBalancedSolutions(results, 10);
            List<GH_CutMesh> cutMeshList = new List<GH_CutMesh>();
            List<double> lengthObj = new List<double>();
            List<double> devObj = new List<double>();
            foreach (var answer in answers)
            {
                bool[] flag = answer.Solution.isCut;
                HashSet<int> edges = new HashSet<int>();
                for (int i = 0; i < flag.Length; i++)
                {
                    if (!flag[i]) continue;
                    List<int> route = path[i];
                    edges.UnionWith(route);
                }
                CutMesh cutMesh2 = CutMeshWithEdgeIndices(cutMesh, edges.ToList());
                cutMeshList.Add(new GH_CutMesh(cutMesh2));
                lengthObj.Add(answer.Objectives[0]);
                devObj.Add(answer.Objectives[1]);
            }
            DA.SetDataList(0, cutMeshList);
            DA.SetDataList(1, lengthObj);
            DA.SetDataList(2, devObj);
        }

        public class CutChoiceVariable
        {
            [Variable]
            public bool[] isCut { get; set; }
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
            get { return new Guid("EBE668C6-F3C9-4E83-9853-F70EF732AB1D"); }
        }
    }
}