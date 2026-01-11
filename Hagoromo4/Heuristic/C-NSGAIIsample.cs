using Grasshopper.Kernel;
using HEAL.Attic;
using HeuristicLab.Algorithms.NSGA2;
using HeuristicLab.Common;
using HeuristicLab.Core;
using HeuristicLab.Data;
using HeuristicLab.Encodings.RealVectorEncoding;
using HeuristicLab.Optimization;
using HeuristicLab.SequentialEngine;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Threading;

namespace MeshOptimizationGH
{
    public class NSGAIIsample : GH_Component
    {
        public NSGAIIsample()
          : base("NSGA-IIsample", "NSGA-IIsample",
                 "Mesh を設計変数として NSGA-II で多目的最適化",
                 "Optimization", "HeuristicLab")
        { }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "最適化対象のメッシュ", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("ParetoPoints", "Pts", "Pareto front の点", GH_ParamAccess.list);
            pManager.AddMeshParameter("ParetoMeshes", "Meshes", "各点に対応するメッシュ", GH_ParamAccess.list);
            pManager.AddNumberParameter("F1List", "F1", "目的関数1", GH_ParamAccess.list);
            pManager.AddNumberParameter("F2List", "F2", "目的関数2", GH_ParamAccess.list);
            pManager.AddNumberParameter("F3List", "F3", "目的関数3", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh mesh = null;
            if (!DA.GetData(0, ref mesh)) return;

            int dimension = mesh.Vertices.Count * 3;

            // Bounds を設定（±range）
            var bounds = new DoubleMatrix(dimension, 2);
            double range = 50;
            for (int i = 0; i < mesh.Vertices.Count; i++)
            {
                var pt = mesh.Vertices[i];
                bounds[3 * i, 0] = pt.X - range;
                bounds[3 * i, 1] = pt.X + range;
                bounds[3 * i + 1, 0] = pt.Y - range;
                bounds[3 * i + 1, 1] = pt.Y + range;
                bounds[3 * i + 2, 0] = pt.Z - range;
                bounds[3 * i + 2, 1] = pt.Z + range;
            }

            var encoding = new RealVectorEncoding("Variables", dimension);
            encoding.Bounds = bounds;

            var evaluator = new MeshEvaluator(mesh);
            var problem = new MeshProblem(evaluator, encoding);

            var nsga2 = new NSGA2 { Problem = problem };
            nsga2.Engine = new SequentialEngine();
            nsga2.PopulationSize.Value = 500;
            nsga2.MaximumGenerations.Value = 1000;

            nsga2.Prepare();
            if (nsga2.ExecutionState != ExecutionState.Prepared)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, $"NSGA2 の状態が Prepared ではありません: {nsga2.ExecutionState}");
                return;
            }

            nsga2.Start(CancellationToken.None);
            while (nsga2.ExecutionState == ExecutionState.Started)
            {
                System.Threading.Thread.Sleep(50);
            }

            var evalCount = (nsga2.Results["EvaluatedSolutions"].Value as IntValue)?.Value ?? -1;
            var genCount = (nsga2.Results["Generations"].Value as IntValue)?.Value ?? -1;
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, $"Evaluated: {evalCount}, Generations: {genCount}");

            var paretoMatrix = nsga2.Results["Pareto Front"].Value as DoubleMatrix;
            var archiveRaw = nsga2.Results["Pareto Archive"].Value as ItemArray<IScope>;

            if (paretoMatrix == null)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Pareto Front が取得できませんでした");
                return;
            }

            if (archiveRaw == null)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Pareto Archive が取得できませんでした");
                return;
            }

            var paretoPoints = new List<Point3d>();
            var paretoMeshes = new List<Mesh>();
            var f1List = new List<double>();
            var f2List = new List<double>();
            var f3List = new List<double>();

            for (int i = 0; i < paretoMatrix.Rows && i < archiveRaw.Count(); i++)
            {
                double f1 = paretoMatrix[i, 0];
                double f2 = paretoMatrix[i, 1];
                double f3 = paretoMatrix[i, 2];

                paretoPoints.Add(new Point3d(f1, f2, f3));
                f1List.Add(f1);
                f2List.Add(f2);
                f3List.Add(f3);

                var scope = archiveRaw[i] as Scope;
                if (scope == null) continue;

                var rv = scope.Variables["Variables"].Value as RealVector;
                if (rv == null) continue;

                var m = MeshEvaluator.VectorToMesh(rv, mesh);
                paretoMeshes.Add(m);
            }

            DA.SetDataList(0, paretoPoints);
            DA.SetDataList(1, paretoMeshes);
            DA.SetDataList(2, f1List);
            DA.SetDataList(3, f2List);
            DA.SetDataList(4, f3List);
        }

        protected override Bitmap Icon => null;
        public override Guid ComponentGuid => new Guid("35935DFC-DB88-4AAB-A8BB-DBA75D41F225");
    }

    public class MeshEvaluator
    {
        private Mesh originalMesh;
        public MeshEvaluator(Mesh mesh) { originalMesh = mesh.DuplicateMesh(); }

        public void Evaluate(RealVector x, double[] qualities)
        {
            Mesh m = VectorToMesh(x, originalMesh);
            var amp = AreaMassProperties.Compute(m);
            var vmp = VolumeMassProperties.Compute(m);

            qualities[0] = Math.Abs((amp?.Area ?? 0.0) - 500);
            qualities[1] = Math.Abs((amp?.Area ?? 0.0) - 500);
            qualities[2] = Math.Abs(1000 - Math.Abs(vmp?.Volume ?? 0.0));
        }

        public static Mesh VectorToMesh(RealVector x, Mesh original)
        {
            var m = new Mesh();
            int vCount = original.Vertices.Count;
            for (int i = 0; i < vCount; i++)
                m.Vertices.Add(x[3 * i], x[3 * i + 1], x[3 * i + 2]);
            foreach (var f in original.Faces)
            {
                if (f.IsTriangle)
                    m.Faces.AddFace(f.A, f.B, f.C);
                else
                    m.Faces.AddFace(f.A, f.B, f.C, f.D);
            }
            m.Normals.ComputeNormals();
            return m;
        }
    }

    [StorableType("8E9D56F6-94AD-4F49-9C2E-9A6CE1B7C4EF")]
    public class MeshProblem : MultiObjectiveBasicProblem<RealVectorEncoding>
    {
        private MeshEvaluator evaluator;

        [StorableConstructor]
        protected MeshProblem(StorableConstructorFlag _) : base(_) { }

        public MeshProblem(MeshEvaluator evaluator, RealVectorEncoding encoding)
        {
            this.evaluator = evaluator;
            this.Encoding = encoding;
        }

        public override bool[] Maximization => new[] { false, false, false };

        public override double[] Evaluate(Individual individual, IRandom random)
        {
            double[] qualities = new double[3];
            try
            {
                evaluator.Evaluate(individual.RealVector(), qualities);
            }
            catch (Exception ex)
            {
                individual["EvaluationError"] = new StringValue(ex.Message);
                return new double[] { 100000, 100000, 100000 };
            }

            individual["Qualities"] = new DoubleArray(qualities);
            return qualities;
        }

        public override IDeepCloneable Clone(Cloner cloner)
        {
            return new MeshProblem(evaluator, Encoding);
        }
    }
}