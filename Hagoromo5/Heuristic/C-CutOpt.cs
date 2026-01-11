using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using HeuristicLab.Algorithms.NSGA2;
using HeuristicLab.Common;
using HeuristicLab.Core;
using HeuristicLab.Data;
using HeuristicLab.Encodings.RealVectorEncoding;
using HeuristicLab.Optimization;
using HeuristicLab.ParallelEngine;
using HeuristicLab.SequentialEngine;
using Rhino.Geometry;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Threading;

using static Hagoromo.DevelopableMesh.CutChoiceTools;
using static Hagoromo.GeometryTools.CutMeshCalcTools;
using static Hagoromo.GeometryTools.MeshCutTools;

namespace Hagoromo.GeometryTools
{
    public class ClusterCutNSGAII : GH_Component
    {
        public ClusterCutNSGAII()
          : base("ClusterCut NSGA-II", "CutOpt",
                 "NSGA-IIを用いてカットパスの最適な組み合わせを選択します",
                 "Hagoromo", "Optimization")
        { }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "M", "Mesh or CutMesh", GH_ParamAccess.item);
            pManager.AddIntegerParameter("PopSize", "Pop", "個体数", GH_ParamAccess.item, 100);
            pManager.AddIntegerParameter("Generations", "Gen", "世代数", GH_ParamAccess.item, 50);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("CutMeshes", "Meshes", "最適化されたメッシュ候補", GH_ParamAccess.list);
            pManager.AddNumberParameter("TotalLength", "L", "カットの総延長", GH_ParamAccess.list);
            pManager.AddNumberParameter("DevObj", "O", "展開図の重なり/歪み指標", GH_ParamAccess.list);
            pManager.AddNumberParameter("Seams", "S", "枝分かれ、切込み本数", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // 1. 入力データの取得と下準備
            object input = null;
            if (!DA.GetData(0, ref input)) return;
            int popSize = 100; DA.GetData(1, ref popSize);
            int generations = 50; DA.GetData(2, ref generations);

            CutMesh cutMesh = null;
            if (input is IGH_Goo goo)
            {
                if (goo.CastTo(out Mesh m)) cutMesh = new CutMesh(m);
                else if (goo.CastTo(out CutMesh cm)) cutMesh = cm.Clone();
            }
            if (cutMesh == null) return;

            // --- グラフ解析とパス生成（定数として扱う） ---
            double threshold = 0.01;
            List<double> gaussMap = GaussianMap(cutMesh, 5).ToList();
            List<List<int>> clusters = FindSeparatedPeaksTwoPass(cutMesh, gaussMap, threshold);
            List<int> represents = clusters.Select(c => c[0]).ToList();
            List<int> negativeRepIndexInReps = new List<int>();
            int repsCount = represents.Count;
            for (int j = 0; j < represents.Count; j++)
            {
                if (gaussMap[represents[j]] <= 0) { negativeRepIndexInReps.Add(j); }
            }

            //negPairIndicesInIsCutは各マイナスのrepを含むpairがisCutで何番目のindexに対応するのかまとめたもの
            int[,] negPairIndicesInIsCut = new int[negativeRepIndexInReps.Count, repsCount];
            for (int j = 0; j < negativeRepIndexInReps.Count; j++)
            {
                int id = negativeRepIndexInReps[j];
                negPairIndicesInIsCut[j, 0] = id;
                for (int i = 0; i < id; i++) 
                {
                    int s = 0;
                    for (int k = 0; k < i + 1; k++) { s += repsCount - k; }
                    negPairIndicesInIsCut[j, i + 1] = id - 1 - i + s;
                }
                int sum = 0;
                for (int i = 0; i < id + 1; i++) { sum += repsCount - i; }
                for (int i = 0; i < repsCount - 1 -id; i++) { negPairIndicesInIsCut[j, id + 1 + i] = sum + i; }

            }

            List<double> edgeCost = new List<double>();
            for (int j = 0; j < cutMesh.Edges.Count; j++)
            {
                int[] edge = cutMesh.Edges[j];
                edgeCost.Add(cutMesh.GetEdgeLine(j).Length / (Math.Abs(gaussMap[edge[0]]) + Math.Abs(gaussMap[edge[1]])) + 0.0000001);
            }

            List<List<int>> pathList = new List<List<int>>();
            foreach (int vert in represents) pathList.Add(FindShortestPathToBoundary(cutMesh, edgeCost, vert));
            for (int i = 0; i < represents.Count; i++)
            {
                for (int j = i + 1; j < represents.Count; j++)
                {
                    pathList.Add(FindShortestPathEdges(cutMesh, edgeCost, represents[i], represents[j]));
                }
            }

            int pathCount = pathList.Count;
            if (pathCount == 0) return;

            List<double> edgeLength = new List<double>();
            for (int i = 0; i < cutMesh.Edges.Count; i++) edgeLength.Add(cutMesh.GetEdgeLine(i).Length);

            // 2. 最適化の設定 (HeuristicLab)
            var bounds = new DoubleMatrix(pathCount, 2);
            for (int i = 0; i < pathCount; i++)
            {
                bounds[i, 0] = 0.0; // Min (False相当)
                bounds[i, 1] = 0.6; // Max (True相当)
            }

            var encoding = new RealVectorEncoding("Variables", pathCount);
            encoding.Bounds = bounds;

            // 自作のProblemクラスをインスタンス化
            var problem = new CutPathProblem(cutMesh, pathList, edgeLength, negPairIndicesInIsCut, encoding);

            var nsga2 = new NSGA2 { Problem = problem };
            nsga2.Engine = new ParallelEngine(); 
            //nsga2.Engine = new SequentialEngine(); // Grasshopper内ではSequentialが安定
            nsga2.PopulationSize.Value = popSize;
            nsga2.MaximumGenerations.Value = generations;

            // 3. 実行
            nsga2.Prepare();





            nsga2.Start(CancellationToken.None);
            while (nsga2.ExecutionState == ExecutionState.Started) Thread.Sleep(10);

            // 4. 結果の抽出
            var paretoMatrix = nsga2.Results["Pareto Front"].Value as DoubleMatrix;
            var archiveRaw = nsga2.Results["Pareto Archive"].Value as ItemArray<IScope>;

            if (paretoMatrix == null || archiveRaw == null) return;

            // バランスの良い解を選ぶためのリストを作成
            var rawResults = new List<ParetoSolution>();
            for (int i = 0; i < paretoMatrix.Rows; i++)
            {
                var scope = archiveRaw[i] as Scope;
                if (scope == null) continue;
                var rv = scope.Variables["Variables"].Value as RealVector;
                if (rv == null) continue;

                rawResults.Add(new ParetoSolution
                {
                    Objectives = new double[] { paretoMatrix[i, 0], paretoMatrix[i, 1], paretoMatrix[i,2] },
                    Variables = rv.ToArray()
                });
            }

            // バランスの良い10個を抽出
            var balanced = GetBalancedIndices(rawResults, 100);

            List<GH_CutMesh> outMeshes = new List<GH_CutMesh>();
            List<double> outL = new List<double>();
            List<double> outO = new List<double>();
            List<double> outS = new List<double>();

            foreach (var sol in balanced)
            {
                bool[] isCut = sol.Variables.Select(v => v >= 0.5).ToArray();
                HashSet<int> edgeIndices = new HashSet<int>();
                for (int i = 0; i < isCut.Length; i++)
                {
                    if (isCut[i]) edgeIndices.UnionWith(pathList[i]);
                }

                var finalMesh = CutMeshWithEdgeIndices(cutMesh, edgeIndices.ToList());
                outMeshes.Add(new GH_CutMesh(finalMesh));
                outL.Add(sol.Objectives[0]);
                outO.Add(sol.Objectives[1]);
                outS.Add(sol.Objectives[2]);
            }

            DA.SetDataList(0, outMeshes);
            DA.SetDataList(1, outL);
            DA.SetDataList(2, outO);
            DA.SetDataList(3, outS);
        }

        // --- 内部クラス: 問題定義 ---
        public class CutPathProblem : MultiObjectiveBasicProblem<RealVectorEncoding>
        {
            private CutMesh _mesh;
            private List<List<int>> _paths;
            private List<double> _edgeLengths;
            private int[,] _negPairIndicesInIsCut;

            public CutPathProblem(CutMesh m, List<List<int>> p, List<double> el, int[,] negPairIndicesInIsCut, RealVectorEncoding encoding)
            {
                _mesh = m; _paths = p; _edgeLengths = el; _negPairIndicesInIsCut = negPairIndicesInIsCut;
                this.Encoding = encoding;
            }

            public override bool[] Maximization => new[] { false, false, false }; // 両方最小化

            public override double[] Evaluate(Individual individual, IRandom random)
            {
                // RealVector を bool[] に変換
                var rv = individual.RealVector();
                bool[] isCut = new bool[rv.Length];
                for (int i = 0; i < rv.Length; i++) isCut[i] = rv[i] >= 0.5;

                try
                {
                    // ユーザー定義の目的関数を呼び出し
                    double[] qualities = TotalLengthAndObj2(_mesh, _paths, _edgeLengths, _negPairIndicesInIsCut, isCut);

                    // 結果をIndividualに保存（後で抽出するため）
                    individual["Objectives"] = new DoubleArray(qualities);
                    return qualities;
                }
                catch
                {
                    return new double[] { 1e10, 1e10, 1e10 };
                }
            }

            public override IDeepCloneable Clone(Cloner cloner)
            {
                return new CutPathProblem(_mesh, _paths, _edgeLengths, _negPairIndicesInIsCut, Encoding);
            }
        }

        // --- ヘルパー: バランス抽出ロジック ---
        private class ParetoSolution { public double[] Objectives; public double[] Variables; }

        private List<ParetoSolution> GetBalancedIndices(List<ParetoSolution> src, int count)
        {
            if (src.Count <= count) return src;

            // 目的関数の正規化
            int objCount = src[0].Objectives.Length;
            double[] mins = new double[objCount];
            double[] maxs = new double[objCount];
            for (int i = 0; i < objCount; i++)
            {
                mins[i] = src.Min(s => s.Objectives[i]);
                maxs[i] = src.Max(s => s.Objectives[i]);
            }

            var selected = new List<int> { 0 };
            while (selected.Count < count && selected.Count < src.Count)
            {
                int bestIdx = -1; double maxMinDist = -1;
                for (int i = 0; i < src.Count; i++)
                {
                    if (selected.Contains(i)) continue;
                    double minDist = selected.Min(sIdx => {
                        double sum = 0;
                        for (int j = 0; j < objCount; j++)
                        {
                            double r = maxs[j] - mins[j];
                            double d = (r == 0) ? 0 : (src[i].Objectives[j] - src[sIdx].Objectives[j]) / r;
                            sum += d * d;
                        }
                        return Math.Sqrt(sum);
                    });
                    if (minDist > maxMinDist) { maxMinDist = minDist; bestIdx = i; }
                }
                if (bestIdx == -1) break;
                selected.Add(bestIdx);
            }
            return selected.Select(i => src[i]).ToList();
        }

        public override Guid ComponentGuid
        {
            get { return new Guid("D59289C0-DBE4-4F22-B4B1-9C097B9DF156"); }
        }
    }
}
