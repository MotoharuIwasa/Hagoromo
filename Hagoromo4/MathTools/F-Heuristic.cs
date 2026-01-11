using HeuristicLab.Algorithms.NSGA2;
using HeuristicLab.Collections;
using HeuristicLab.Common;
using HeuristicLab.Core;
using HeuristicLab.Data;
using HeuristicLab.Encodings.RealVectorEncoding;
using HeuristicLab.Optimization;
using HeuristicLab.ParallelEngine;
using HeuristicLab.Parameters;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;

namespace Hagoromo.MathTools
{
    [AttributeUsage(AttributeTargets.Property)]
    public class VariableAttribute : Attribute { }

    public class VariableConfig
    {
        public int Length { get; set; } = 1;
        public double[] Min { get; set; }
        public double[] Max { get; set; }
        public bool IsBinary { get; set; } // bool[] 用のフラグ

        // double[] 用の設定
        public static VariableConfig CreateUniform(int length, double min, double max)
        {
            return new VariableConfig
            {
                Length = length,
                IsBinary = false,
                Min = Enumerable.Repeat(min, length).ToArray(),
                Max = Enumerable.Repeat(max, length).ToArray()
            };
        }

        // bool[] 用の設定
        public static VariableConfig CreateBinary(int length)
        {
            return new VariableConfig
            {
                Length = length,
                IsBinary = true,
                Min = Enumerable.Repeat(0.0, length).ToArray(),
                Max = Enumerable.Repeat(1.0, length).ToArray()
            };
        }
    }

    public class LargeScaleHLOptimizer<T> where T : class, new()
    {
        private class PropertyMeta
        {
            public PropertyInfo Info;
            public int Offset;
            public int Count;
            public bool IsBinary;
        }

        public List<(T Solution, double[] Objectives)> Run(
            Func<T, double[]> objectiveFunc,
            Dictionary<string, VariableConfig> configs,
            T initialValue = null,
            int popSize = 100,
            int generations = 100)
        {
            var metaList = new List<PropertyMeta>();
            int currentOffset = 0;

            foreach (var p in typeof(T).GetProperties())
            {
                if (p.GetCustomAttribute<VariableAttribute>() == null) continue;
                if (!configs.ContainsKey(p.Name)) continue;

                var config = configs[p.Name];
                metaList.Add(new PropertyMeta
                {
                    Info = p,
                    Offset = currentOffset,
                    Count = config.Length,
                    IsBinary = config.IsBinary
                });
                currentOffset += config.Length;
            }

            int totalVectorSize = currentOffset;
            var bounds = new DoubleMatrix(totalVectorSize, 2);
            foreach (var meta in metaList)
            {
                var config = configs[meta.Info.Name];
                for (int i = 0; i < meta.Count; i++)
                {
                    bounds[meta.Offset + i, 0] = config.Min[i];
                    bounds[meta.Offset + i, 1] = config.Max[i];
                }
            }

            var encoding = new RealVectorEncoding();
            encoding.Length = totalVectorSize;
            encoding.Bounds = bounds;

            int objCount = objectiveFunc(new T()).Length;
            var problem = new GenericLargeScaleProblem(objectiveFunc, encoding, metaList, objCount);

            var nsga2 = new NSGA2 { Problem = problem };
            nsga2.PopulationSize.Value = popSize;
            nsga2.MaximumGenerations.Value = generations;
            nsga2.Engine = new ParallelEngine();

            // アルゴリズムを準備（ここで初期集団がランダム生成される）
            nsga2.Prepare();

            // --- 初期値の注入 ---
            if (initialValue != null)
            {
                double[] initialVector = ExtractVector(initialValue, metaList, totalVectorSize);

                // IDE0019 対策：'as' と 'if != null' を 'is' パターンマッチングに統合
                // CS0246 対策：具体的な型名ではなく IList インターフェースで受ける
                if (nsga2.Results["Population"].Value is IList population && population.Count > 0)
                {
                    // 最初の個体を取得（Individual型は既に認識されているはず）
                    if (population[0] is Individual firstIndividual)
                    {
                        var realVector = firstIndividual.RealVector();
                        for (int i = 0; i < initialVector.Length; i++)
                        {
                            realVector[i] = initialVector[i];
                        }
                    }
                }
            }

            nsga2.Start();

            while (nsga2.ExecutionState == ExecutionState.Started) System.Threading.Thread.Sleep(50);

            var results = new List<(T Solution, double[] Objectives)>();

            if (nsga2.Results.ContainsKey("ParetoFront"))
            {
                var paretoFront = nsga2.Results["ParetoFront"].Value as IEnumerable;
                if (paretoFront != null)
                {
                    foreach (var indObj in paretoFront)
                    {
                        if (indObj is Individual ind)
                        {
                            // 設計変数の復元
                            T solInstance = ConvertIndividualToT(ind, metaList);

                            // 目的関数の復元（CS1061対策：インデクサと型チェックを使用）
                            double[] objectives = null;
                            var objData = ind["Objectives"]; // インデクサによる安全なアクセス

                            if (objData is DoubleArray da) objectives = da.ToArray();
                            else if (objData is IEnumerable<double> enumDouble)
                            {
                                // DoubleVector や double[] の両方に対応できる汎用的な書き方
                                objectives = enumDouble.ToArray();
                            }

                            results.Add((solInstance, objectives));
                        }
                    }
                }
            }

            return results;
        }

        private T ConvertIndividualToT(Individual ind, List<PropertyMeta> metaList)
        {
            var values = ind.RealVector(); // values は DoubleVector 型
            T instance = new T();
            foreach (var meta in metaList)
            {
                if (meta.IsBinary)
                {
                    var arr = new bool[meta.Count];
                    for (int i = 0; i < meta.Count; i++)
                        arr[i] = values[meta.Offset + i] >= 0.5;
                    meta.Info.SetValue(instance, arr);
                }
                else if (meta.Info.PropertyType == typeof(double[]))
                {
                    var arr = new double[meta.Count];
                    // Array.Copy ではなく DoubleVector の機能を使ってコピー
                    for (int i = 0; i < meta.Count; i++)
                    {
                        arr[i] = values[meta.Offset + i];
                    }
                    meta.Info.SetValue(instance, arr);
                }
                else if (meta.Info.PropertyType == typeof(double))
                {
                    meta.Info.SetValue(instance, values[meta.Offset]);
                }
            }
            return instance;
        }

        private double[] ExtractVector(T instance, List<PropertyMeta> metaList, int totalSize)
        {
            var vector = new double[totalSize];
            foreach (var meta in metaList)
            {
                var val = meta.Info.GetValue(instance);
                if (meta.IsBinary)
                {
                    var arr = (bool[])val;
                    for (int i = 0; i < meta.Count; i++)
                        vector[meta.Offset + i] = arr[i] ? 1.0 : 0.0; // trueなら1.0
                }
                else if (meta.Info.PropertyType == typeof(double[]))
                {
                    var arr = (double[])val;
                    Array.Copy(arr, 0, vector, meta.Offset, meta.Count);
                }
                else if (meta.Info.PropertyType == typeof(double))
                {
                    vector[meta.Offset] = (double)val;
                }
            }
            return vector;
        }

        private class GenericLargeScaleProblem : MultiObjectiveBasicProblem<RealVectorEncoding>
        {
            private readonly Func<T, double[]> _objectiveFunc;
            private readonly List<PropertyMeta> _metaList;
            private readonly bool[] _maximization;

            public GenericLargeScaleProblem(Func<T, double[]> objFunc, RealVectorEncoding encoding, List<PropertyMeta> meta, int objCount)
            {


                
                _objectiveFunc = objFunc;
                _metaList = meta;
                _maximization = new bool[objCount];
                EncodingParameter.Value = encoding;
                

                /*
                _objectiveFunc = objFunc;
                _metaList = meta;
                this.Encoding = encoding;
                _maximization = new bool[objCount];
                */
            }

            public override bool[] Maximization => _maximization;

            // GenericLargeScaleProblem クラス内の Evaluate メソッドを修正
            public override double[] Evaluate(Individual individual, IRandom random)
            {
                // 追加: 個体が RealVector を持っていない、またはサイズが合わない場合のガード
                var values = individual.RealVector();
                if (values == null || values.Length == 0)
                    return Enumerable.Repeat(double.MaxValue, _maximization.Length).ToArray();

                T instance = new T();

                foreach (var meta in _metaList)
                {
                    // 追加: オフセットが配列の範囲内かチェック (これが -1 エラーの直接原因の可能性)
                    if (meta.Offset < 0 || meta.Offset >= values.Length) continue;

                    if (meta.IsBinary)
                    {
                        var arr = new bool[meta.Count];
                        for (int i = 0; i < meta.Count; i++)
                        {
                            // インデックスの安全確認
                            int idx = meta.Offset + i;
                            if (idx < values.Length) arr[i] = values[idx] >= 0.5;
                        }
                        meta.Info.SetValue(instance, arr);
                    }
                    else if (meta.Info.PropertyType == typeof(double[]))
                    {
                        var arr = new double[meta.Count];
                        for (int i = 0; i < meta.Count; i++)
                        {
                            int idx = meta.Offset + i;
                            if (idx < values.Length) arr[i] = values[idx];
                        }
                        meta.Info.SetValue(instance, arr);
                    }
                    else if (meta.Info.PropertyType == typeof(double))
                    {
                        meta.Info.SetValue(instance, values[meta.Offset]);
                    }
                }

                // 目的関数を実行
                var results = _objectiveFunc(instance);

                // 目的関数の戻り値が null だった場合の保険
                if (results == null) return Enumerable.Repeat(double.MaxValue, _maximization.Length).ToArray();
                return results;
            }

            public override IDeepCloneable Clone(Cloner cloner)
            {
                return new GenericLargeScaleProblem(_objectiveFunc, (RealVectorEncoding)Encoding.Clone(cloner), _metaList, _maximization.Length);
            }
        }

        

    }

    public static class ResultReview
    {
        public static List<(T Solution, double[] Objectives)> GetBalancedSolutions<T>(List<(T Solution, double[] Objectives)> results, int count)
        {
            if (results.Count <= count) return results;

            // 1. 目的関数の値を正規化（各目的のスケールを 0.0 ~ 1.0 に揃える）
            int objCount = results[0].Objectives.Length;
            double[] mins = new double[objCount];
            double[] maxs = new double[objCount];

            for (int i = 0; i < objCount; i++)
            {
                mins[i] = results.Min(r => r.Objectives[i]);
                maxs[i] = results.Max(r => r.Objectives[i]);
            }

            var normalizedObjs = results.Select(r => {
                double[] norm = new double[objCount];
                for (int i = 0; i < objCount; i++)
                {
                    double range = maxs[i] - mins[i];
                    norm[i] = (range == 0) ? 0 : (r.Objectives[i] - mins[i]) / range;
                }
                return norm;
            }).ToList();

            // 2. 多様性を確保しながら選択（Max-Min Distance Selection）
            var selectedIndices = new List<int>();

            // 最初の1個は、第1目的関数が最も優れているものを選ぶ
            selectedIndices.Add(0);

            while (selectedIndices.Count < count)
            {
                int bestNextIndex = -1;
                double maxOfMinDist = -1;

                for (int i = 0; i < results.Count; i++)
                {
                    if (selectedIndices.Contains(i)) continue;

                    // すでに選ばれた集合との最小距離（最も近い既選解との距離）を計算
                    double minDistToSelected = selectedIndices.Min(sIdx =>
                        Math.Sqrt(normalizedObjs[i].Zip(normalizedObjs[sIdx], (a, b) => Math.Pow(a - b, 2)).Sum())
                    );

                    // その「最小距離」が最も大きいもの（＝既存の解から最も離れている未選解）を選ぶ
                    if (minDistToSelected > maxOfMinDist)
                    {
                        maxOfMinDist = minDistToSelected;
                        bestNextIndex = i;
                    }
                }
                selectedIndices.Add(bestNextIndex);
            }

            return selectedIndices.Select(idx => results[idx]).ToList();
        }
    }
}