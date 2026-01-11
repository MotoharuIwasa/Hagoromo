using GH_IO.Serialization;
using Grasshopper.Kernel;
using Hagoromo.GeometryTools;
using Hagoromo.MathTools;
using HeuristicLab.Data;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Collections;
using Rhino.Render.ChangeQueue;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Forms.VisualStyles;
using System.Xml.Linq;
using static Hagoromo.DevelopableMesh.DevelopableTools;
using static Hagoromo.GeometryTools.CutMeshCalcTools;

namespace Hagoromo.DevelopableMesh
{
    //--------------------------------------------- 3次元用、dupVertは一緒に移動 --------------------------------------------
    public class MinLengthTerm : LeastSquaresTerm
    {
        private List<int[]> edges;
        private double minLen;
        private int[] vertMap; // 重複頂点IDへのマッピング

        public MinLengthTerm(double weight, List<int[]> edges, double minLen, int[] vertMap) : base(weight)
        {
            this.edges = edges;
            this.minLen = minLen;
            this.vertMap = vertMap;
        }

        public override bool IsFeasible(double[] x)
        {
            for (int i = 0; i < edges.Count; i++)
            {
                int id1 = vertMap[edges[i][0]] * 3;
                int id2 = vertMap[edges[i][1]] * 3;

                double dx = x[id1] - x[id2];
                double dy = x[id1 + 1] - x[id2 + 1];
                double dz = x[id1 + 2] - x[id2 + 2];
                double lenSq = dx * dx + dy * dy + dz * dz;

                // 二乗比較で高速化（不等式制約の閾値を判定）
                if (lenSq < minLen * minLen)
                {
                    return false; // 1つでも違反があればNG
                }
            }
            return true;
        }

        public override void AddToSystem(double[] x, ConstraintBuilder builder)
        {
            for (int i = 0; i < edges.Count; i++)
            {
                // 座標計算...
                int id1 = vertMap[edges[i][0]];
                int id2 = vertMap[edges[i][1]];
                // ... (座標取得や距離計算は省略) ...
                double currentLen = 1023;
                Vector3d vec = new Vector3d(1, 2, 4); // p1 - p2

                // ★不等式の判定
                if (currentLen < minLen)//違反している場合は制約条件として不等式の<を=としたものを入れる
                {
                    // 違反しているので制約を追加する
                    double residual = Weight * (minLen - currentLen); // bの値
                    double factor = Weight / currentLen;

                    // builderに「この式を追加して！」と頼むだけ
                    // 行番号は builder が勝手に採番してくれる
                    builder.AddConstraint(
                        residual,
                        // 勾配のリスト (col, val)
                        (id1 * 3, factor * vec.X),
                        (id1 * 3 + 1, factor * vec.Y),
                        (id1 * 3 + 2, factor * vec.Z),
                        (id2 * 3, -factor * vec.X),
                        (id2 * 3 + 1, -factor * vec.Y),
                        (id2 * 3 + 2, -factor * vec.Z)
                    );
                }
                else
                {
                    // 違反していないときは何もしない（builderを呼ばない）
                    // -> 行も消費されないし、隙間もできない
                }
            }
        }
    }

    public class MinMaxLengthCGLS3D : LeastSquaresTerm
    {
        private List<int[]> cullDupEdges;
        private int[] vertOrderInDup;
        //initialLengthはcullDupEdgesの長さを入れたもの
        private double[] initialLength;
        //ミラーなどの都合で動けない要素のx[]での位置（x[]は動けないdupVertの座標もすべて入れておく)
        private HashSet<int> unMoveSet;

        public MinMaxLengthCGLS3D(double weight, List<int[]> cullDupEdges, int[] vertOrderInDup, double[] initialLength, HashSet<int> unMoveSet) : base(weight)
        {
            //重複しているエッジは取り除いたエッジのリスト
            this.cullDupEdges = cullDupEdges;
            this.vertOrderInDup = vertOrderInDup;
            this.initialLength = initialLength;
            this.unMoveSet = unMoveSet;
        }

        public override bool IsFeasible(double[] x)
        {
            for (int i = 0; i < cullDupEdges.Count; i++)
            {
                int[] edge = cullDupEdges[i];
                int id1 = vertOrderInDup[edge[0]] * 3;
                int id2 = vertOrderInDup[edge[1]] * 3;

                double dx = x[id1] - x[id2];
                double dy = x[id1 + 1] - x[id2 + 1];
                double dz = x[id1 + 2] - x[id2 + 2];
                double lenSq = dx * dx + dy * dy + dz * dz;

                // 二乗比較で高速化（不等式制約の閾値を判定）
                double minSq = initialLength[i] * initialLength[i] * 0.99;
                double maxSq = initialLength[i] * initialLength[i] * 1.1;
                if (lenSq < minSq || lenSq > maxSq)
                {
                    return false; // 1つでも違反があればNG
                }
            }
            return true;
        }

        public override void AddToSystem(double[] x, ConstraintBuilder builder)
        {
            for (int i = 0; i < cullDupEdges.Count; i++)
            {
                int[] edge = cullDupEdges[i];
                int i1 = vertOrderInDup[edge[0]] * 3;
                int i2 = vertOrderInDup[edge[1]] * 3;

                double dx = x[i1] - x[i2];
                double dy = x[i1 + 1] - x[i2 + 1];
                double dz = x[i1 + 2] - x[i2 + 2];
                double len = Math.Sqrt(dx * dx + dy * dy + dz * dz);
                double initLen = initialLength[i];
                double diff = 0;
                if (initLen * 0.1 > len)
                {
                   diff = len - initialLength[i] * 0.5;
                }
                /*
                if (initLen * 2 < len)
                {
                   diff = len - initialLength[i];
                }
                */
                if (initLen * 0.5 > len /*|| initLen * 2 < len*/)
                {
                    // 0除算対策
                    if (len < 1e-9) len = 0.001;

                    // 勾配計算 (J^T * R)
                    // Force magnitude = weight * (current - rest)
                    double wxdiff = Weight * diff;
                    double factor = Weight / len;

                    double fx = factor * dx;
                    double fy = factor * dy;
                    double fz = factor * dz;
                    double[] g = new double[6];
                    if (!unMoveSet.Contains(i1 + 0)) { g[0] = fx; }
                    if (!unMoveSet.Contains(i1 + 1)) { g[1] = fy; }
                    if (!unMoveSet.Contains(i1 + 2)) { g[2] = fz; }

                    // 頂点v2に減算 (反作用)
                    if (!unMoveSet.Contains(i2 + 0)) { g[3] = -fx; }
                    if (!unMoveSet.Contains(i2 + 1)) { g[4] = -fy; }
                    if (!unMoveSet.Contains(i2 + 2)) { g[5] = -fz; }
                    builder.AddConstraint(
                            -wxdiff,
                            // 勾配のリスト (col, val)
                            (i1 + 0, g[0]),
                            (i1 + 1, g[1]),
                            (i1 + 2, g[2]),
                            (i2 + 0, g[3]),
                            (i2 + 1, g[4]),
                            (i2 + 2, g[5])
                        );
                }
            }
        }
    }


    //エッジの長さ保存（等式制約)
    public class EdgeLengthCGLS3D : LeastSquaresTerm
    {
        private List<int[]> cullDupEdges;
        private int[] vertOrderInDup;
        //initialLengthはcullDupEdgesの長さを入れたもの
        private double[] initialLength;
        //ミラーなどの都合で動けない要素のx[]での位置（x[]は動けないdupVertの座標もすべて入れておく)
        private HashSet<int> unMoveSet;

        public EdgeLengthCGLS3D(double weight, List<int[]> cullDupEdges, int[] vertOrderInDup, double[] initialLength, HashSet<int> unMoveSet) : base(weight)
        {
            //重複しているエッジは取り除いたエッジのリスト
            this.cullDupEdges = cullDupEdges;
            this.vertOrderInDup = vertOrderInDup;
            this.initialLength = initialLength;
            this.unMoveSet = unMoveSet;
        }

        public override void AddToSystem(double[] x, ConstraintBuilder builder)
        {
            for (int i = 0; i < cullDupEdges.Count; i++)
            {
                int[] edge = cullDupEdges[i];
                int i1 = vertOrderInDup[edge[0]] * 3;
                int i2 = vertOrderInDup[edge[1]] * 3;

                double dx = x[i1] - x[i2];
                double dy = x[i1 + 1] - x[i2 + 1];
                double dz = x[i1 + 2] - x[i2 + 2];
                double len = Math.Sqrt(dx * dx + dy * dy + dz * dz);

                double diff = len - initialLength[i];

                // 0除算対策
                if (len < 1e-9) continue;

                // 勾配計算 (J^T * R)
                // Force magnitude = weight * (current - rest)
                double wxdiff = Weight * diff;
                double factor = Weight / len;

                double fx = factor * dx;
                double fy = factor * dy;
                double fz = factor * dz;
                double[] g = new double[6];
                if (!unMoveSet.Contains(i1 + 0)) { g[0] = fx; }
                if (!unMoveSet.Contains(i1 + 1)) { g[1] = fy; }
                if (!unMoveSet.Contains(i1 + 2)) { g[2] = fz; }

                // 頂点v2に減算 (反作用)
                if (!unMoveSet.Contains(i2 + 0)) { g[3] = -fx; }
                if (!unMoveSet.Contains(i2 + 1)) { g[4] = -fy; }
                if (!unMoveSet.Contains(i2 + 2)) { g[5] = -fz; }
                builder.AddConstraint(
                        -wxdiff,
                        // 勾配のリスト (col, val)
                        (i1 + 0, g[0]),
                        (i1 + 1, g[1]),
                        (i1 + 2, g[2]),
                        (i2 + 0, g[3]),
                        (i2 + 1, g[4]),
                        (i2 + 2, g[5])
                    );
            }
        }
    }

    //周りの点の平均とするスムージング制約
    public class SmoothingCGLS : LeastSquaresTerm
    {
        //dupVertのi番目のvertGroupが他のどのdupVertと隣り合っているかをまとめたリスト
        private List<List<int>> dupConnectedVertIndices;

        //ミラーやスムージングしたくないなどの都合で考えない要素のx[]での位置（x[]は動けないdupVertの座標もすべて入れておく)
        private HashSet<int> unMoveSet;

        public SmoothingCGLS(double weight, List<List<int>> dupConnectedVertIndices, HashSet<int> unMoveSet)
            : base(weight)
        {
            this.dupConnectedVertIndices = dupConnectedVertIndices;
            // 配列をHashSetに変換しておく（O(N) -> O(1)）
            this.unMoveSet = new HashSet<int>(unMoveSet);
        }

        public override void AddToSystem(double[] x, ConstraintBuilder builder)
        {
            for (int i = 0; i < dupConnectedVertIndices.Count; i++)
            {
                List<int> connected = dupConnectedVertIndices[i];
                int n = connected.Count;
                if (n == 0) continue;
                double factor = -Weight / n;
                int i3 = i * 3;

                // --- 共通計算---
                Point3d average = Point3d.Origin;
                int myRowIndex1 = builder.AllocateRow();
                int myRowIndex2 = builder.AllocateRow();
                int myRowIndex3 = builder.AllocateRow();
                if (!unMoveSet.Contains(i3 + 0)) builder.AddDerivative(myRowIndex1, i3 + 0, Weight);
                if (!unMoveSet.Contains(i3 + 1)) builder.AddDerivative(myRowIndex2, i3 + 1, Weight);
                if (!unMoveSet.Contains(i3 + 2)) builder.AddDerivative(myRowIndex3, i3 + 2, Weight);
                foreach (int vert in connected)
                {
                    int id = vert * 3;
                    average.X += x[id];
                    average.Y += x[id + 1];
                    average.Z += x[id + 2];
                    if (!unMoveSet.Contains(id + 0)) builder.AddDerivative(myRowIndex1, id + 0, factor);
                    if (!unMoveSet.Contains(id + 1)) builder.AddDerivative(myRowIndex2, id + 1, factor);
                    if (!unMoveSet.Contains(id + 2)) builder.AddDerivative(myRowIndex3, id + 2, factor);
                }

                average /= n;
                Vector3d vec = new Vector3d(
                    x[i3] - average.X,
                    x[i3 + 1] - average.Y,
                    x[i3 + 2] - average.Z
                );

                builder.AddResidual(myRowIndex1, -vec.X);
                builder.AddResidual(myRowIndex2, -vec.Y);
                builder.AddResidual(myRowIndex3, -vec.Z);

            }
        }

    }

    //周りの点の平均とするスムージング制約
    public class VertMoveCGLS : LeastSquaresTerm
    {
        //ミラーなどの都合で動けない要素のx[]での位置（x[]は動けないdupVertの座標もすべて入れておく)
        private HashSet<int> unMoveSet;

        private List<Point3d> dupInitialPosition;

        public VertMoveCGLS(double weight, List<Point3d> dupInitialPosition, HashSet<int> unMoveSet)
            : base(weight)
        {
            this.dupInitialPosition = dupInitialPosition;
            // 配列をHashSetに変換しておく（O(N) -> O(1)）
            this.unMoveSet = new HashSet<int>(unMoveSet);
        }

        public override void AddToSystem(double[] x, ConstraintBuilder builder)
        {
            for (int i = 0; i < dupInitialPosition.Count; i++)
            {
                int i3 = i * 3;
                Point3d current = new Point3d(x[i3 + 0], x[i3 + 1], x[i3 + 2]);
                Vector3d vec = Weight * (dupInitialPosition[i] - current);
                builder.AddConstraint(vec.X, (i3 + 0, Weight));
                builder.AddConstraint(vec.Y, (i3 + 1, Weight));
                builder.AddConstraint(vec.Z, (i3 + 2, Weight));
            }
        }

    }

    public class DevelopableCGLS : LeastSquaresTerm
    {
        //------------------------------GetEnergyなどの関数でx[]以外に使いたい変数をここに書いておく。
        private CutMesh preMesh;

        private List<List<List<int>>> category;

        private List<double> aveLengthDone;

        private List<double> aveLengthF;

        //各Vertごとにx,y,zが動くかのデータ
        private List<bool[]> moveList;

        private List<Dictionary<int, List<int>>> DoneMaps;

        private List<Dictionary<int, List<int>>> FMaps;

        // 事前計算データ
        private int[] vertOrderInDup;
        private int[] constraintsOrder;
        private int[] vertToConstraints;

        //BCEDFの各項のweightの設定、基本1でOK、長さは7
        private double[] w;

        public DevelopableCGLS(double weight, CutMesh preMesh, List<List<List<int>>> category,
            List<double> aveLengthDone, List<double> aveLengthF, double[] w, List<bool[]> moveList,
            List<Dictionary<int, List<int>>> DoneMaps, List<Dictionary<int, List<int>>> FMaps) : base(weight)
        {
            this.preMesh = preMesh;
            this.category = category;
            this.aveLengthDone = aveLengthDone;
            this.aveLengthF = aveLengthF;
            this.w = w;
            this.moveList = moveList;
            this.DoneMaps = DoneMaps;
            this.FMaps = FMaps;
            // 事前計算
            this.vertOrderInDup = preMesh.VertOrderInDup();
            this.constraintsOrder = ConstraintsOrder(preMesh.DuplicatedVertIndices, category);
            //verticesのi番目の点がconstraintsの何行目(～何行目)なのかの対応のリスト
            this.vertToConstraints = new int[preMesh.Vertices.Count];
            for (int i = 0; i < vertToConstraints.Length; i++)
            {
                vertToConstraints[i] = -1;
            }
            for (int i = 0; i < preMesh.Vertices.Count; i++)
            {
                vertToConstraints[i] = constraintsOrder[vertOrderInDup[i]];
            }
        }

        public override void AddToSystem(double[] x, ConstraintBuilder builder)
        {
            double w1 = w[0] * Weight;
            double w2 = w[1] * Weight;
            double w3 = w[2] * Weight;
            double w4 = w[3] * Weight;
            double w5 = w[4] * Weight;
            double w6 = w[5] * Weight;
            //----------------------------------------------------A～Fの分類-----------------------------------------------------------------------*/
            List<int> B = category[0][1];
            List<int> C = category[0][2];
            List<int> E = category[0][4];
            List<List<int>> DoneMirror = category[1];
            List<int[]> DoneMirror2 = DoneMirror.Select(inner => inner.ToArray()).ToList();

            List<List<int>> DtwoMirror = category[5];
            List<int[]> DtwoMirror2 = DtwoMirror.Select(inner => inner.ToArray()).ToList();
            List<List<int>> Fblocks = category[9];
            List<int[]> Fblocks2 = Fblocks.Select(inner => inner.ToArray()).ToList();

            List<List<int>> duplicatedVertIndices = preMesh.DuplicatedVertIndices;
            int vertCount = preMesh.Vertices.Count;

            //現在のx[]からcutMeshを作成
            Point3d[] vertices = new Point3d[vertCount];
            for (int i = 0; i < vertCount; i++)
            {
                int id = vertOrderInDup[i] * 3;
                vertices[i] = new Point3d(x[id], x[id + 1], x[id + 2]);
            }
            CutMesh mesh = new CutMesh(vertices.ToList(), preMesh.Faces, preMesh.Edges, preMesh.DuplicatedVertIndices);


            double[] angleSum = AngleSum(mesh);
            var loopData = CalcLoopData(mesh, angleSum, DoneMirror, Fblocks);
            List<List<double>> DoneLength = loopData.DoneLength;
            List<List<double>> DoneCos = loopData.DoneCos;
            List<List<double>> DoneSin = loopData.DoneSin;
            List<List<double>> FblocksLength = loopData.FblocksLength;
            List<List<double>> FblocksCos = loopData.FblocksCos;
            List<List<double>> FblocksSin = loopData.FblocksSin;

            foreach (int vert in B)
            {
                BCEVertCGLS(ref builder, mesh, moveList, vert, angleSum, vertOrderInDup, 4, w1);
            }
            foreach (int vert in C)
            {
                BCEVertCGLS(ref builder, mesh, moveList, vert, angleSum, vertOrderInDup, 2, w1);
            }


            foreach (int vert in E)
            {
                BCEVertCGLS(ref builder, mesh, moveList, vert, angleSum, vertOrderInDup, 1, w1);
            }


            foreach (int[] loop in DtwoMirror2)
            {
                DFVertAngleCGLS(ref builder, mesh, moveList, loop, angleSum, vertOrderInDup, 0, 1, w6);
            }

            int ii = 0;
            foreach (int[] loop in DoneMirror2)
            {
                DFVertAngleCGLS(ref builder, mesh, moveList, loop, angleSum, vertOrderInDup, 0, 1, w4);

                double aveLength = aveLengthDone[ii];
                List<double> loopLength = DoneLength[ii];
                List<double> loopSin = DoneSin[ii];
                List<double> loopCos = DoneCos[ii];
                double s = CalcDoneSinFactor(loopLength, loopSin);
                int n = loopLength.Count;

                double value = w5 * (n + 1) * Math.PI / aveLength;
                double E1 = s * value;
                builder.AddResidual(-E1);
                

                HashSet<int> skipVert = new HashSet<int>();
                int loopFirst = loop[0];
                foreach (int vert in loop)
                {
                    if (skipVert.Contains(vert)) continue;
                    bool[] move = moveList[vert];
                    if (move.All(m => m == false)) continue;

                    int dupIndex = vertOrderInDup[vert];
                    //dupIndex * 3
                    List<int> vertGroup = duplicatedVertIndices[dupIndex];
                    vertGroup.Sort();
                    //vertGroupのloop内でのインデックス
                    List<int> vertGroupLoopIndex = vertGroup.Select(xx => xx - loopFirst).ToList();
                    skipVert.UnionWith(vertGroup);
                    Vector3d angleSumDiffSum = new Vector3d();
                    Vector3d diffSelf = new Vector3d();
                    int j = 0;
                    for (int i = 0; i < n; i++)
                    {
                        if (j < vertGroupLoopIndex.Count)
                        {
                            int ik = vertGroupLoopIndex[j];
                            if (i == ik - 1)
                            {
                                Vector3d vec1 = mesh.Vertices[vertGroup[j]] - mesh.Vertices[vertGroup[j] - 1];
                                if (i % 2 == 0) { diffSelf += loopSin[ik - 1] * vec1 / loopLength[ik]; }
                                if (i % 2 == 1) { diffSelf -= loopSin[ik - 1] * vec1 / loopLength[ik]; }
                                double[] angleSumDiff = CalcDxDSigmaTheta(mesh, angleSum, vertGroup[j] - 1, vertGroup[j], move);
                                angleSumDiffSum += new Vector3d(angleSumDiff[0], angleSumDiff[1], angleSumDiff[2]);
                            }
                            else if (i == ik)
                            {
                                Vector3d vec2 = mesh.Vertices[vertGroup[j]] - mesh.Vertices[vertGroup[j] + 1];
                                if (i % 2 == 0) { diffSelf += loopSin[ik] * vec2 / loopLength[ik + 1]; }
                                if (i % 2 == 0) { diffSelf -= loopSin[ik] * vec2 / loopLength[ik + 1]; }
                                double[] angleSumDiff2 = CalcDxDSigmaTheta(mesh, angleSum, vertGroup[j], vertGroup[j], move);
                                angleSumDiffSum += new Vector3d(angleSumDiff2[0], angleSumDiff2[1], angleSumDiff2[2]);
                            }
                            else if (i == ik + 1)
                            {
                                double[] angleSumDiff = CalcDxDSigmaTheta(mesh, angleSum, vertGroup[j] + 1, vertGroup[j], move);
                                angleSumDiffSum += new Vector3d(angleSumDiff[0], angleSumDiff[1], angleSumDiff[2]);
                                j++;
                            }
                            if (j < vertGroupLoopIndex.Count) { if (i == ik + 1 && i == vertGroupLoopIndex[j] - 1) { continue; } }
                        }
                        if (i % 2 == 0) { diffSelf += loopLength[i + 1] * loopCos[i] * angleSumDiffSum; }
                        if (i % 2 == 1) { diffSelf -= loopLength[i + 1] * loopCos[i] * angleSumDiffSum; }
                    }
                    diffSelf *= value;
                    int dup3 = dupIndex * 3;
                    if (move[0]) builder.AddDerivative(dup3 + 0, diffSelf.X);
                    if (move[1]) builder.AddDerivative(dup3 + 1, diffSelf.Y);
                    if (move[2]) builder.AddDerivative(dup3 + 2, diffSelf.Z);
                }

                //次はループのconnectedVertによる偏微分を計算
                Dictionary<int, List<int>> loopMap = DoneMaps[ii];
                foreach (var kvp in loopMap)
                {
                    int id = kvp.Key;
                    List<int> connnected = duplicatedVertIndices[id];
                    bool[] move = moveList[connnected[0]];
                    List<int> loopVertsIndex = kvp.Value;
                    loopVertsIndex.Sort();
                    int j = 0;
                    Vector3d angleSumDiffSum = new Vector3d();
                    Vector3d diffOther = new Vector3d();
                    for (int i = 0; i < n; i++)
                    {
                        if (j < loopVertsIndex.Count)
                        {
                            int ik = loopVertsIndex[j];
                            int ikIndex = ik + loopFirst;
                            if (i == ik)
                            {
                                foreach (int connectedVert in connnected)
                                {
                                    double[] angleSumDiff = CalcDxDSigmaTheta(mesh, angleSum, ikIndex, connectedVert, move);
                                    angleSumDiffSum += new Vector3d(angleSumDiff[0], angleSumDiff[1], angleSumDiff[2]);
                                }
                                j++;
                            }
                        }
                        if (i % 2 == 0) { diffOther += loopLength[i + 1] * loopCos[i] * angleSumDiffSum; }
                        if (i % 2 == 1) { diffOther -= loopLength[i + 1] * loopCos[i] * angleSumDiffSum; }
                    }
                    int gIndex = 3 * id;
                    diffOther *= value;
                    if (move[0]) builder.AddDerivative(gIndex + 0, diffOther.X);
                    if (move[1]) builder.AddDerivative(gIndex + 1, diffOther.Y);
                    if (move[2]) builder.AddDerivative(gIndex + 2, diffOther.Z);
                }
                
                builder.NextRow();
                ii += 1;
            }

            ii = 0;
            foreach (int[] loop in Fblocks2)
            {
                DFVertAngleCGLS(ref builder, mesh, moveList, loop, angleSum, vertOrderInDup, 2, 4, w2);


                
                double aveLength = aveLengthF[ii];
                List<double> loopLength = FblocksLength[ii];
                List<double> loopSin = FblocksSin[ii];
                List<double> loopCos = FblocksCos[ii];
                int n = loopCos.Count;
                double value = w3 * (n + 4) * Math.PI / aveLength;

                //発散防止
                if (loopLength.Min() < aveLength * 0.001) { continue; }

                
                //-----------------------------------------------sinのエネルギー----------------------------------------
                double s = CalcFSinFactor(loopLength, loopSin);
                double E1 = s * value;
                int row1 = builder.AllocateRow();
                builder.AddResidual(row1,-E1);
                

                
                //-----------------------------------------------cosのエネルギー----------------------------------------
                double c = CalcFCosFactor(loopLength, loopCos);
                double E2 = c * value;
                int row2 = builder.AllocateRow();
                builder.AddResidual(row2, -E2);
                

                HashSet<int> skipVert = new HashSet<int>();
                int loopFirst = loop[0];
                
                
                //-----------------------------------------------sinの微分----------------------------------------
                foreach (int vert in loop)
                {
                    if (skipVert.Contains(vert)) continue;
                    bool[] move = moveList[vert];
                    if (move.All(m => m == false)) continue;

                    int dupIndex = vertOrderInDup[vert];

                    List<int> vertGroup = duplicatedVertIndices[dupIndex];
                    vertGroup.Sort();
                    //vertGroupのloop内でのインデックス
                    List<int> vertGroupLoopIndex = vertGroup.Select(xx => xx - loopFirst).ToList();
                    skipVert.UnionWith(vertGroup);
                    Vector3d angleSumDiffSum = new Vector3d();
                    Vector3d diffSelf = new Vector3d();
                    int j = 0;
                    for (int i = 1; i < n + 1; i++)
                    {
                        if (j < vertGroupLoopIndex.Count)
                        {
                            int ik = vertGroupLoopIndex[j];
                            if (i == (ik - 1) % (n + 1))
                            {
                                Vector3d vec1 = mesh.Vertices[vertGroup[j]] - mesh.Vertices[i + loopFirst];
                                if (i % 2 == 0) { diffSelf += loopSin[i - 1] * vec1 / loopLength[i + 1]; }
                                if (i % 2 == 1) { diffSelf -= loopSin[i - 1] * vec1 / loopLength[i + 1]; }
                                double[] angleSumDiff = CalcDxDSigmaTheta(mesh, angleSum, i + loopFirst, vertGroup[j], move);
                                angleSumDiffSum += new Vector3d(angleSumDiff[0], angleSumDiff[1], angleSumDiff[2]);
                            }
                            else if (i == ik)
                            {
                                Vector3d vec2 = mesh.Vertices[vertGroup[j]] - mesh.Vertices[(ik + 1) % (n + 1) + loopFirst];
                                if (i % 2 == 0) { diffSelf += loopSin[i - 1] * vec2 / loopLength[i + 1]; }
                                if (i % 2 == 1) { diffSelf -= loopSin[i - 1] * vec2 / loopLength[i + 1]; }
                                if (i != 0)
                                {
                                    double[] angleSumDiff2 = CalcDxDSigmaTheta(mesh, angleSum, vertGroup[j], vertGroup[j], move);
                                    angleSumDiffSum += new Vector3d(angleSumDiff2[0], angleSumDiff2[1], angleSumDiff2[2]);
                                }
                            }
                            else if (i == (ik + 1) % (n + 1))
                            {
                                double[] angleSumDiff = CalcDxDSigmaTheta(mesh, angleSum, i + loopFirst, vertGroup[j], move);
                                angleSumDiffSum += new Vector3d(angleSumDiff[0], angleSumDiff[1], angleSumDiff[2]);
                                j++;
                            }
                            if (j < vertGroupLoopIndex.Count) { if (i == ik + 1 && i == vertGroupLoopIndex[j] - 1) { continue; } }
                        }
                        if (vertGroupLoopIndex[0] == 0 && i == n)
                        {
                            Vector3d vec1 = mesh.Vertices[loopFirst] - mesh.Vertices[i + loopFirst];
                            if (i % 2 == 0) { diffSelf += loopSin[i - 1] * vec1 / loopLength[i + 1]; }
                            if (i % 2 == 1) { diffSelf -= loopSin[i - 1] * vec1 / loopLength[i + 1]; }
                            double[] angleSumDiff = CalcDxDSigmaTheta(mesh, angleSum, i + loopFirst, loopFirst, move);
                            angleSumDiffSum += new Vector3d(angleSumDiff[0], angleSumDiff[1], angleSumDiff[2]);
                        }

                        if (i % 2 == 0) { diffSelf += loopLength[i + 1] * loopCos[i - 1] * angleSumDiffSum; }
                        else if (i % 2 == 1) { diffSelf -= loopLength[i + 1] * loopCos[i - 1] * angleSumDiffSum; }
                    }
                    diffSelf *= value;
                    int dup3 = dupIndex * 3;
                    if (move[0]) builder.AddDerivative(row1, dup3 + 0, diffSelf.X);
                    if (move[1]) builder.AddDerivative(row1, dup3 + 1, diffSelf.Y);
                    if (move[2]) builder.AddDerivative(row1, dup3 + 2, diffSelf.Z);
                }

                //次はループのconnectedVertによる偏微分を計算
                Dictionary<int, List<int>> loopMap = FMaps[ii];
                foreach (var kvp in loopMap)
                {
                    int id = kvp.Key;
                    List<int> connnected = duplicatedVertIndices[id];
                    bool[] move = moveList[connnected[0]];
                    List<int> loopVertsIndex = kvp.Value;
                    loopVertsIndex.Sort();
                    int j = 0;
                    Vector3d angleSumDiffSum = new Vector3d();
                    Vector3d diffOther = new Vector3d();
                    for (int i = 1; i < n + 1; i++)
                    {
                        if (j < loopVertsIndex.Count)
                        {
                            int ik = loopVertsIndex[j];
                            int ikIndex = ik + loopFirst;
                            if (i == ik)
                            {
                                foreach (int connectedVert in connnected)
                                {
                                    double[] angleSumDiff = CalcDxDSigmaTheta(mesh, angleSum, ikIndex, connectedVert, move);
                                    angleSumDiffSum += new Vector3d(angleSumDiff[0], angleSumDiff[1], angleSumDiff[2]);
                                }
                                j++;
                            }
                        }
                        if (i % 2 == 0) { diffOther += loopLength[i + 1] * loopCos[i - 1] * angleSumDiffSum; }
                        if (i % 2 == 1) { diffOther -= loopLength[i + 1] * loopCos[i - 1] * angleSumDiffSum; }
                    }
                    int gIndex = 3 * id;
                    diffOther *= value;
                    if (move[0]) builder.AddDerivative(row1, gIndex + 0, diffOther.X);
                    if (move[1]) builder.AddDerivative(row1, gIndex + 1, diffOther.Y);
                    if (move[2]) builder.AddDerivative(row1, gIndex + 2, diffOther.Z);
                }

                
                //-----------------------------------------------cosの微分----------------------------------------
                skipVert = new HashSet<int>();

                foreach (int vert in loop)
                {
                    if (skipVert.Contains(vert)) continue;
                    bool[] move = moveList[vert];
                    if (move.All(m => m == false)) continue;

                    int dupIndex = vertOrderInDup[vert];

                    List<int> vertGroup = duplicatedVertIndices[dupIndex];
                    vertGroup.Sort();
                    //vertGroupのloop内でのインデックス
                    List<int> vertGroupLoopIndex = vertGroup.Select(xx => xx - loopFirst).ToList();
                    skipVert.UnionWith(vertGroup);
                    Vector3d angleSumDiffSum = new Vector3d();
                    Vector3d diffSelf = new Vector3d();

                    int j = 0;
                    if (vertGroupLoopIndex[0] == 0)
                    {
                        Vector3d vec1 = mesh.Vertices[loopFirst] - mesh.Vertices[loopFirst + 1];
                        diffSelf += vec1 / loopLength[1];
                    }
                    if (vertGroupLoopIndex[0] == 1)
                    {
                        Vector3d vec1 = mesh.Vertices[loopFirst + 1] - mesh.Vertices[loopFirst];
                        diffSelf += vec1 / loopLength[1];
                    }


                    for (int i = 1; i < n + 1; i++)
                    {
                        if (j < vertGroupLoopIndex.Count)
                        {
                            int ik = vertGroupLoopIndex[j];
                            if (i == (ik - 1) % (n + 1))
                            {
                                Vector3d vec1 = mesh.Vertices[vertGroup[j]] - mesh.Vertices[i + loopFirst];
                                if (i % 2 == 0) { diffSelf += loopCos[i - 1] * vec1 / loopLength[i + 1]; }
                                if (i % 2 == 1) { diffSelf -= loopCos[i - 1] * vec1 / loopLength[i + 1]; }
                                double[] angleSumDiff = CalcDxDSigmaTheta(mesh, angleSum, i + loopFirst, vertGroup[j], move);
                                angleSumDiffSum += new Vector3d(angleSumDiff[0], angleSumDiff[1], angleSumDiff[2]);
                            }

                            else if (i == ik)
                            {
                                Vector3d vec2 = mesh.Vertices[vertGroup[j]] - mesh.Vertices[(ik + 1) % (n + 1) + loopFirst];
                                if (i % 2 == 0) { diffSelf += loopCos[i - 1] * vec2 / loopLength[i + 1]; }
                                if (i % 2 == 1) { diffSelf -= loopCos[i - 1] * vec2 / loopLength[i + 1]; }
                                double[] angleSumDiff2 = CalcDxDSigmaTheta(mesh, angleSum, vertGroup[j], vertGroup[j], move);
                                angleSumDiffSum += new Vector3d(angleSumDiff2[0], angleSumDiff2[1], angleSumDiff2[2]);
                            }
                            else if (i == (ik + 1) % (n + 1))
                            {
                                double[] angleSumDiff = CalcDxDSigmaTheta(mesh, angleSum, i + loopFirst, vertGroup[j], move);
                                angleSumDiffSum += new Vector3d(angleSumDiff[0], angleSumDiff[1], angleSumDiff[2]);
                                j++;
                            }
                            if (j < vertGroupLoopIndex.Count) { if (i == ik + 1 && i == vertGroupLoopIndex[j] - 1) { continue; } }
                        }

                        if (vertGroupLoopIndex[0] == 0 && i == n)
                        {
                            Vector3d vec1 = mesh.Vertices[loopFirst] - mesh.Vertices[i + loopFirst];
                            if (i % 2 == 0) { diffSelf += loopCos[i - 1] * vec1 / loopLength[i + 1]; }
                            if (i % 2 == 1) { diffSelf -= loopCos[i - 1] * vec1 / loopLength[i + 1]; }
                            double[] angleSumDiff = CalcDxDSigmaTheta(mesh, angleSum, i + loopFirst, loopFirst, move);
                            angleSumDiffSum += new Vector3d(angleSumDiff[0], angleSumDiff[1], angleSumDiff[2]);
                        }

                        if (i % 2 == 0) { diffSelf -= loopLength[i + 1] * loopSin[i - 1] * angleSumDiffSum; }
                        else if (i % 2 == 1) { diffSelf += loopLength[i + 1] * loopSin[i - 1] * angleSumDiffSum; }
                    }
                    diffSelf *= value;
                    int dup3 = dupIndex * 3;
                    if (move[0]) builder.AddDerivative(row2, dup3 + 0, diffSelf.X);
                    if (move[1]) builder.AddDerivative(row2, dup3 + 1, diffSelf.Y);
                    if (move[2]) builder.AddDerivative(row2, dup3 + 2, diffSelf.Z);
                }

                //次はループのconnectedVertによる偏微分を計算

                foreach (var kvp in loopMap)
                {
                    int id = kvp.Key;
                    List<int> connnected = duplicatedVertIndices[id];
                    bool[] move = moveList[connnected[0]];
                    List<int> loopVertsIndex = kvp.Value;
                    loopVertsIndex.Sort();
                    int j = 0;
                    Vector3d angleSumDiffSum = new Vector3d();
                    Vector3d diffOther = new Vector3d();
                    for (int i = 1; i < n + 1; i++)
                    {
                        if (j < loopVertsIndex.Count)
                        {
                            int ik = loopVertsIndex[j];
                            int ikIndex = ik + loopFirst;
                            if (i == ik)
                            {
                                foreach (int connectedVert in connnected)
                                {
                                    double[] angleSumDiff = CalcDxDSigmaTheta(mesh, angleSum, ikIndex, connectedVert, move);
                                    angleSumDiffSum += new Vector3d(angleSumDiff[0], angleSumDiff[1], angleSumDiff[2]);
                                }
                                j++;
                            }
                        }
                        if (i % 2 == 0) { diffOther -= loopLength[i + 1] * loopSin[i - 1] * angleSumDiffSum; }
                        if (i % 2 == 1) { diffOther += loopLength[i + 1] * loopSin[i - 1] * angleSumDiffSum; }
                    }
                    int gIndex = 3 * id;
                    diffOther *= value;
                    if (move[0]) builder.AddDerivative(row2, gIndex + 0, diffOther.X);
                    if (move[1]) builder.AddDerivative(row2, gIndex + 1, diffOther.Y);
                    if (move[2]) builder.AddDerivative(row2, gIndex + 2, diffOther.Z);
                }
                

                builder.NextRow();
                ii += 1;
                
            }
        }
    }

    
    public class BoundarySmoothCGLS : LeastSquaresTerm
    {
        //ミラーやスムージングしたくないなどの都合で考えない要素のx[]での位置（x[]は動けないdupVertの座標もすべて入れておく)
        private HashSet<int> unMoveSet;

        //滑らかにしたいpolyline群のdupIndicesをセット
        private List<List<int>> dupVerts;

        public BoundarySmoothCGLS(double weight, HashSet<int> unMoveSet, List<List<int>> dupVerts)
            : base(weight)
        {
            // 配列をHashSetに変換しておく
            this.unMoveSet = new HashSet<int>(unMoveSet);
            this.dupVerts = dupVerts;
        }

        
        public override void AddToSystem(double[] x, ConstraintBuilder builder)
        {
            foreach (List<int> loop in dupVerts)
            {
                for (int i = 1; i < loop.Count-1; i++)
                {
                    List<int> connected = new List<int> { loop[i - 1], loop[i + 1] };
                    double factor = -Weight / 2;
                    int i3 = loop[i] * 3;

                    // --- 共通計算---
                    Point3d average = Point3d.Origin;
                    int myRowIndex1 = builder.AllocateRow();
                    int myRowIndex2 = builder.AllocateRow();
                    int myRowIndex3 = builder.AllocateRow();
                    if (!unMoveSet.Contains(i3 + 0)) builder.AddDerivative(myRowIndex1, i3 + 0, Weight);
                    if (!unMoveSet.Contains(i3 + 1)) builder.AddDerivative(myRowIndex2, i3 + 1, Weight);
                    if (!unMoveSet.Contains(i3 + 2)) builder.AddDerivative(myRowIndex3, i3 + 2, Weight);
                    foreach (int vert in connected)
                    {
                        int id = vert * 3;
                        average.X += x[id];
                        average.Y += x[id + 1];
                        average.Z += x[id + 2];
                        if (!unMoveSet.Contains(id + 0)) builder.AddDerivative(myRowIndex1, id + 0, factor);
                        if (!unMoveSet.Contains(id + 1)) builder.AddDerivative(myRowIndex2, id + 1, factor);
                        if (!unMoveSet.Contains(id + 2)) builder.AddDerivative(myRowIndex3, id + 2, factor);
                    }

                    average /= 2;
                    Vector3d vec = new Vector3d(
                        x[i3] - average.X,
                        x[i3 + 1] - average.Y,
                        x[i3 + 2] - average.Z
                    );

                    builder.AddResidual(myRowIndex1, -vec.X);
                    builder.AddResidual(myRowIndex2, -vec.Y);
                    builder.AddResidual(myRowIndex3, -vec.Z);
                }
            }
        }
    }
    
}
