using HeuristicLab.Algorithms.NSGA2;
using HeuristicLab.Collections;
using HeuristicLab.Common;
using HeuristicLab.Core;
using HeuristicLab.Data; // DoubleArray 用
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
        public bool IsBinary { get; set; }

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
            if (totalVectorSize == 0) return new List<(T, double[])>();

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

            // RealVectorEncoding の名前を "RealVector" に固定
            var encoding = new RealVectorEncoding("RealVector", totalVectorSize);
            encoding.Bounds = bounds;

            // 目的関数の数を確認
            T dummy = new T();
            foreach (var meta in metaList)
            {
                if (meta.IsBinary) meta.Info.SetValue(dummy, new bool[meta.Count]);
                else if (meta.Info.PropertyType == typeof(double[])) meta.Info.SetValue(dummy, new double[meta.Count]);
            }
            int objCount = objectiveFunc(dummy).Length;

            var problem = new GenericLargeScaleProblem(objectiveFunc, encoding, metaList, objCount);
            var nsga2 = new NSGA2 { Problem = problem };
            nsga2.PopulationSize.Value = popSize;
            nsga2.MaximumGenerations.Value = generations;
            nsga2.Engine = new ParallelEngine();

            nsga2.Prepare();

            // 初期値の注入 (インデクサによるアクセス)
            if (initialValue != null)
            {
                double[] initialVector = ExtractVector(initialValue, metaList, totalVectorSize);
                if (nsga2.Results["Population"].Value is IList population && population.Count > 0)
                {
                    if (population[0] is Individual firstInd)
                    {
                        var realVector = (DoubleArray)firstInd["RealVector"];
                        for (int i = 0; i < initialVector.Length; i++) realVector[i] = initialVector[i];
                    }
                }
            }

            nsga2.Start();
            while (nsga2.ExecutionState == ExecutionState.Started) System.Threading.Thread.Sleep(50);

            var results = new List<(T Solution, double[] Objectives)>();
            if (nsga2.Results.ContainsKey("ParetoFront") && nsga2.Results["ParetoFront"].Value is IEnumerable pf)
            {
                foreach (var ind in pf.OfType<Individual>())
                {
                    T solInstance = ConvertIndividualToT(ind, metaList);
                    double[] objectives = ((DoubleArray)ind["Objectives"]).ToArray();
                    results.Add((solInstance, objectives));
                }
            }
            return results;
        }

        private T ConvertIndividualToT(Individual ind, List<PropertyMeta> metaList)
        {
            var values = (DoubleArray)ind["RealVector"];
            T instance = new T();
            foreach (var meta in metaList)
            {
                if (meta.IsBinary)
                {
                    var arr = new bool[meta.Count];
                    for (int i = 0; i < meta.Count; i++) arr[i] = values[meta.Offset + i] >= 0.5;
                    meta.Info.SetValue(instance, arr);
                }
                else if (meta.Info.PropertyType == typeof(double[]))
                {
                    var arr = new double[meta.Count];
                    for (int i = 0; i < meta.Count; i++) arr[i] = values[meta.Offset + i];
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
                    for (int i = 0; i < meta.Count; i++) vector[meta.Offset + i] = arr[i] ? 1.0 : 0.0;
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

                // パラメータ辞書経由でセット（名前の不一致を回避）
                this.Parameters["Encoding"].Value = encoding;

                // エンコーディング固有のオペレータを指定
                this.Parameters["SolutionCreator"].Value = new UniformRealVectorCreator();
                this.Parameters["Crossover"].Value = new UniformRealVectorCrossover();
                this.Parameters["Mutator"].Value = new NormalAllPositionsMutator();
            }

            public override bool[] Maximization => _maximization;

            public override double[] Evaluate(Individual individual, IRandom random)
            {
                var values = (DoubleArray)individual["RealVector"];
                if (values == null || values.Length == 0) return Enumerable.Repeat(1e10, _maximization.Length).ToArray();

                T instance = new T();
                foreach (var meta in _metaList)
                {
                    if (meta.Offset + meta.Count > values.Length) continue;
                    if (meta.IsBinary)
                    {
                        var arr = new bool[meta.Count];
                        for (int i = 0; i < meta.Count; i++) arr[i] = values[meta.Offset + i] >= 0.5;
                        meta.Info.SetValue(instance, arr);
                    }
                    else if (meta.Info.PropertyType == typeof(double[]))
                    {
                        var arr = new double[meta.Count];
                        for (int i = 0; i < meta.Count; i++) arr[i] = values[meta.Offset + i];
                        meta.Info.SetValue(instance, arr);
                    }
                    else if (meta.Info.PropertyType == typeof(double))
                    {
                        meta.Info.SetValue(instance, values[meta.Offset]);
                    }
                }
                return _objectiveFunc(instance) ?? Enumerable.Repeat(1e10, _maximization.Length).ToArray();
            }

            public override IDeepCloneable Clone(Cloner cloner)
            {
                return new GenericLargeScaleProblem(_objectiveFunc, (RealVectorEncoding)this.Encoding.Clone(cloner), _metaList, _maximization.Length);
            }
        }
    }

    public static class ResultReview
    {
        public static List<(T Solution, double[] Objectives)> GetBalancedSolutions<T>(List<(T Solution, double[] Objectives)> results, int count)
        {
            if (results == null || results.Count == 0) return new List<(T, double[])>();
            if (results.Count <= count) return results;

            int objCount = results[0].Objectives.Length;
            double[] mins = new double[objCount];
            double[] maxs = new double[objCount];
            for (int i = 0; i < objCount; i++)
            {
                mins[i] = results.Min(r => r.Objectives[i]);
                maxs[i] = results.Max(r => r.Objectives[i]);
            }

            var norm = results.Select(r => {
                double[] n = new double[objCount];
                for (int i = 0; i < objCount; i++)
                {
                    double range = maxs[i] - mins[i];
                    n[i] = (range == 0) ? 0 : (r.Objectives[i] - mins[i]) / range;
                }
                return n;
            }).ToList();

            var selected = new List<int> { 0 };
            while (selected.Count < count)
            {
                int next = -1; double dMax = -1;
                for (int i = 0; i < results.Count; i++)
                {
                    if (selected.Contains(i)) continue;
                    double d = selected.Min(sIdx => Math.Sqrt(norm[i].Zip(norm[sIdx], (a, b) => Math.Pow(a - b, 2)).Sum()));
                    if (d > dMax) { dMax = d; next = i; }
                }
                if (next == -1) break;
                selected.Add(next);
            }
            return selected.Select(idx => results[idx]).ToList();
        }
    }
}