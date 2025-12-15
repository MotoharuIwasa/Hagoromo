using GH_IO.Serialization;
using Grasshopper.Kernel;
using Hagoromo.GeometryTools;
using Hagoromo.MathTools;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Collections;
using Rhino.Render.ChangeQueue;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Xml.Linq;
using static Hagoromo.GeometryTools.CutMeshCalcTools;
using static Hagoromo.DevelopableMesh.DevelopableTools;

namespace Hagoromo.DevelopableMesh
{
    //CutMeshでやるとき、設計変数を各点の座標でやると思うが、それをdouble[] xとしているので、CutMesh.Verticesなどで数値を得ると
    //更新されていないものになるので、つながりなどiterationによって変わらないものは気にしなくてよいが、変わるものはdouble[] xを参照する

    //--------------------------------------------- 3次元用、dupVertは一緒に移動 --------------------------------------------
    public class CGNR3D
    {
        public static HashSet<int> VertToXindex(int vert, int[] vertOrderInDup, bool x, bool y, bool z)
        {
            HashSet<int> answer = new HashSet<int>();
            int id = vertOrderInDup[vert];
            if (!x) { answer.Add(id * 3); }
            if (!y) { answer.Add(id * 3 + 1); }
            if (!z) { answer.Add(id * 3 + 2); }
            return answer;
        }

        public static HashSet<int> MoveListToUnMoveSet(List<bool[]> categoryToMoveList, int[] vertOrderInDup)
        {
            HashSet<int> unMoveSet = new HashSet<int>();
            for (int i = 0; i < vertOrderInDup.Length; i++)
            {
                int dupIndex = vertOrderInDup[i];
                bool[] bools = categoryToMoveList[i];
                int id = dupIndex * 3;
                if (!bools[0]) { unMoveSet.Add(id); }
                if (!bools[1]) { unMoveSet.Add(id + 1); }
                if (!bools[2]) { unMoveSet.Add(id + 2); }
            }
            return unMoveSet;
        }

        public static List<bool[]> CategoryToMoveList(int vertCount, List<List<List<int>>> category, List<int> fixIndices)
        {
            bool[,] moveList = new bool[vertCount, 3];
            List<int> moveX = new List<int>(category[0][10]);
            foreach (List<int> loop in category[8]) { moveX.Add(loop[0]); moveX.Add(loop[loop.Count - 1]); }
            List<int> moveY = new List<int>(category[0][11]);
            foreach (List<int> loop in category[6]) { moveY.Add(loop[0]); moveY.Add(loop[loop.Count - 1]); }
            List<int> moveZ = new List<int>(category[0][12]);
            foreach (List<int> loop in category[7]) { moveZ.Add(loop[0]); moveZ.Add(loop[loop.Count - 1]); }

            List<int> moveXY = new List<int>(category[0][6]);
            moveXY.AddRange(category[0][13]);
            foreach (List<int> loop in category[2]) { moveXY.Add(loop[0]); moveXY.Add(loop[loop.Count - 1]); }
            List<int> moveYZ = new List<int>(category[0][7]);
            moveYZ.AddRange(category[0][14]);
            foreach (List<int> loop in category[3]) { moveYZ.Add(loop[0]); moveYZ.Add(loop[loop.Count - 1]); }
            List<int> moveZX = new List<int>(category[0][8]);
            moveZX.AddRange(category[0][15]);
            foreach (List<int> loop in category[4]) { moveZX.Add(loop[0]); moveZX.Add(loop[loop.Count - 1]); }

            List<int> moveXYZ = new List<int>(category[0][9]);
            moveXYZ.AddRange(category[0][4]);
            moveXYZ.AddRange(category[0][5]);

            foreach (int i in moveX) { moveList[i, 0] = true; }
            foreach (int i in moveY) { moveList[i, 1] = true; }
            foreach (int i in moveZ) { moveList[i, 2] = true; }
            foreach (int i in moveXY) { moveList[i, 0] = true; moveList[i, 1] = true; }
            foreach (int i in moveYZ) { moveList[i, 1] = true; moveList[i, 2] = true; }
            foreach (int i in moveZX) { moveList[i, 2] = true; moveList[i, 0] = true; }
            foreach (int i in moveXYZ) { moveList[i, 0] = true; moveList[i, 1] = true; moveList[i, 2] = true; }
            foreach (int i in fixIndices) { moveList[i, 0] = false; moveList[i, 1] = false; moveList[i, 2] = false; }
            List<bool[]> list = new List<bool[]>();
            for (int i = 0; i < vertCount; i++)
            {
                bool[] row = new bool[3];
                for (int j = 0; j < 3; j++)
                {
                    row[j] = moveList[i, j];
                }
                list.Add(row);
            }
            return list;
        }

        public static List<Dictionary<int, List<int>>> MakeMaps(CutMesh mesh, List<List<int>> blocks, int[] vertOrderInDup)
        {
            List<Dictionary<int, List<int>>> maps = new List<Dictionary<int, List<int>>>();
            foreach (List<int> loop in blocks)
            {
                int n = loop.Count;
                List<int[]> loopConnect = new List<int[]>();
                List<HashSet<int>> vertConnect = new List<HashSet<int>>();
                for (int i = 0; i < n; i++)
                {
                    List<int> connect = mesh.GetVerticesForVertex(loop[i]);
                    connect = connect.Except(loop).ToList();
                    HashSet<int> connectGroup = new HashSet<int>();
                    foreach (int j in connect)
                    {
                        connectGroup.Add(vertOrderInDup[j]);
                    }
                    vertConnect.Add(connectGroup);
                }

                Dictionary<int, List<int>> reverseMap = new Dictionary<int, List<int>>();
                for (int i = 0; i < vertConnect.Count; i++)
                {
                    foreach (int val in vertConnect[i])
                    {
                        if (!reverseMap.ContainsKey(val))
                        {
                            reverseMap[val] = new List<int>();
                        }
                        reverseMap[val].Add(i); // i番目のHashSetに含まれている
                    }
                }
                maps.Add(reverseMap);
            }
            return maps;
        }

        //cutMeshから重複しているエッジを取り除いた場合のedgesを格納
        public static List<int[]> CullDupEdges(CutMesh mesh, int[] vertOrderInDup)
        {
            List<int[]> edges = mesh.Edges;
            List<int> boundaryEdges = mesh.BoundaryEdgeIndices();
            List<int> cullEdgeIndices = new List<int>();
            List<HashSet<int>> edgeVertGroup = new List<HashSet<int>>();
            foreach (int edge in boundaryEdges)
            {
                if (cullEdgeIndices.Contains(edge)) { continue; }
                int dupIndex1 = vertOrderInDup[edges[edge][0]];
                int dupIndex2 = vertOrderInDup[edges[edge][1]];
                HashSet<int> vertGroup = new HashSet<int>(mesh.DuplicatedVertIndices[dupIndex1]);
                vertGroup.UnionWith(mesh.DuplicatedVertIndices[dupIndex2]);
                edgeVertGroup.Add(vertGroup);
            }
            int i = 0;
            foreach (int edge in boundaryEdges)
            {
                if (cullEdgeIndices.Contains(edge)) { continue; }
                HashSet<int> vertGroup = edgeVertGroup[i];
                // i番目と完全一致するインデックスを列挙
                for (int j = 0; j < edgeVertGroup.Count; j++)
                {
                    if (j == i) continue; // 自分自身は除外
                    if (vertGroup.SetEquals(edgeVertGroup[j]))
                    {
                        cullEdgeIndices.Add(boundaryEdges[j]);
                    }
                }
                i++;
            }
            List<int[]> newEdges = new List<int[]>();
            HashSet<int> cullEdgeIndices2 = cullEdgeIndices.ToHashSet();
            for (int j = 0; j < edges.Count; j++)
            {
                if (cullEdgeIndices2.Contains(j)) { continue; }
                newEdges.Add(mesh.Edges[j]);
            }
            return newEdges;
        }
    }


    //エッジ長さ保存エネルギー (E = 0.5 * w * (L - L0)^2)、等式制約条件: L - L0 = 0
    public class EdgeLengthEnergy3D : EnergyTerm
    {
        //------------------------------GetEnergyなどの関数でx[]以外に使いたい変数をここに書いておく。
        private List<int[]> cullDupEdges;
        private int[] vertOrderInDup;
        //initialLengthはcullDupEdgesの長さを入れたもの
        private double[] initialLength;
        //ミラーなどの都合で動けない要素のx[]での位置（x[]は動けないdupVertの座標もすべて入れておく)
        private HashSet<int> unMoveSet;

        public EdgeLengthEnergy3D(double weight, List<int[]> cullDupEdges, int[] vertOrderInDup, double[] initialLength, HashSet<int> unMoveSet) : base(weight)
        {
            //重複しているエッジは取り除いたエッジのリスト
            this.cullDupEdges = cullDupEdges;
            this.vertOrderInDup = vertOrderInDup;
            this.initialLength = initialLength;
            this.unMoveSet = unMoveSet;


            /*
            //verticesのi番目の点がdupVertの何番目のグループの点なのかの対応のリスト
            int[] vertOrderInDup = new int[mesh.Vertices.Count];
            for (int i = 0; i < vertOrderInDup.Length; i++)
            {
                vertOrderInDup[i] = -1;
            }
            for (int i = 0; i < duplicatedVertIndices.Count; i++)
            {
                foreach (int vertIndex in duplicatedVertIndices[i])
                {
                    vertOrderInDup[vertIndex] = i;
                }
            }
            */
        }
        //------------------------------GetEnergyなどの関数でx[]以外に使いたい変数をここに書いておく。



        public override double GetEnergy(double[] x)
        {
            double E = 0.0;
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
                E += 0.5 * Weight * diff * diff;
            }
            return E;
        }

        public override void AddGradient(double[] x, double[] g)
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

                // 0除算対策
                if (len < 1e-9) continue;

                // 勾配計算 (J^T * R)
                // Force magnitude = weight * (current - rest)
                double diff = len - initialLength[i];
                double factor = Weight * diff / len;

                double fx = factor * dx;
                double fy = factor * dy;
                double fz = factor * dz;

                // 頂点v1に加算
                if (!unMoveSet.Contains(i1)) { g[i1] += fx; }
                if (!unMoveSet.Contains(i1 + 1)) { g[i1 + 1] += fy; }
                if (!unMoveSet.Contains(i1 + 2)) { g[i1 + 2] += fz; }

                // 頂点v2に減算 (反作用)
                if (!unMoveSet.Contains(i2)) { g[i2] -= fx; }
                if (!unMoveSet.Contains(i2 + 1)) { g[i2 + 1] -= fy; }
                if (!unMoveSet.Contains(i2 + 2)) { g[i2 + 2] -= fz; }
            }
        }
    }

    //スムージング、周囲の位置との平均をとる
    public class SmoothingEnergy : EnergyTerm
    {
        //dupVertのi番目のvertGroupが他のどのdupVertと隣り合っているかをまとめたリスト
        private List<List<int>> dupConnectedVertIndices;

        //ミラーやスムージングしたくないなどの都合で考えない要素のx[]での位置（x[]は動けないdupVertの座標もすべて入れておく)
        private HashSet<int> unMoveSet;

        public SmoothingEnergy(double weight, List<List<int>> dupConnectedVertIndices, HashSet<int> unMoveSet)
            : base(weight)
        {
            this.dupConnectedVertIndices = dupConnectedVertIndices;
            // 配列をHashSetに変換しておく（O(N) -> O(1)）
            this.unMoveSet = new HashSet<int>(unMoveSet);
        }

        // GetEnergy と AddGradient は Evaluate を呼ぶようにしてもいいし、
        // LineSearch用に GetEnergy だけは軽量に残しておいても良いです。
        // ここでは「Evaluate」の中身を最適化します。

        public override double Evaluate(double[] x, double[] g)
        {
            double E = 0.0;
            bool calcGrad = (g != null); // 勾配計算が必要か？

            for (int i = 0; i < dupConnectedVertIndices.Count; i++)
            {
                List<int> connected = dupConnectedVertIndices[i];

                // --- 共通計算---
                Point3d average = Point3d.Origin;
                foreach (int vert in connected)
                {
                    int id = vert * 3;
                    average.X += x[id];
                    average.Y += x[id + 1];
                    average.Z += x[id + 2];
                }
                int n = connected.Count;
                if (n == 0) continue;

                average /= n;

                int i3 = i * 3;
                // vec: 自分の位置 - 周囲の平均
                Vector3d vec = new Vector3d(
                    x[i3] - average.X,
                    x[i3 + 1] - average.Y,
                    x[i3 + 2] - average.Z
                );

                // --- 1. エネルギーへの加算 ---
                // 0.5 * w * ||vec||^2
                E += 0.5 * Weight * (vec.X * vec.X + vec.Y * vec.Y + vec.Z * vec.Z);

                // --- 2. 勾配への加算 (必要な場合のみ) ---
                if (calcGrad)
                {
                    // 自分自身への勾配 (+vec)
                    if (!unMoveSet.Contains(i3)) g[i3] += Weight * vec.X;
                    if (!unMoveSet.Contains(i3 + 1)) g[i3 + 1] += Weight * vec.Y;
                    if (!unMoveSet.Contains(i3 + 2)) g[i3 + 2] += Weight * vec.Z;

                    // 接続相手への勾配 (-vec / n)
                    double factor = -Weight / n;
                    double fx = vec.X * factor;
                    double fy = vec.Y * factor;
                    double fz = vec.Z * factor;

                    foreach (int vert in connected)
                    {
                        int v3 = vert * 3;
                        if (!unMoveSet.Contains(v3)) g[v3] += fx;
                        if (!unMoveSet.Contains(v3 + 1)) g[v3 + 1] += fy;
                        if (!unMoveSet.Contains(v3 + 2)) g[v3 + 2] += fz;
                    }
                }
            }

            return E;
        }

        // 既存のメソッドは Evaluate に委譲してもよい
        public override double GetEnergy(double[] x) => Evaluate(x, null);
        public override void AddGradient(double[] x, double[] g) => Evaluate(x, g);
    }

    public class DevelopableConsiderOtherEnergy : EnergyTerm
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

        public DevelopableConsiderOtherEnergy(double weight, CutMesh preMesh, List<List<List<int>>> category,
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

        //------------------------------GetEnergyなどの関数でx[]以外に使いたい変数をここに書いておく。



        public override double Evaluate(double[] x, double[] g)
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

            double obj = 0;

            
            foreach (int vert in B)
            {
                BCEVertTerm(ref obj, ref g, mesh, moveList, vert, angleSum, vertOrderInDup, 4, w1);
            }
            foreach (int vert in C)
            {
                BCEVertTerm(ref obj, ref g, mesh, moveList, vert, angleSum, vertOrderInDup, 2, w1);
            }

            
            foreach (int vert in E)
            {
                BCEVertTerm(ref obj, ref g, mesh, moveList, vert, angleSum, vertOrderInDup, 1, w1);
            }
            

            foreach (int[] loop in DtwoMirror2)
            {
                DFVertAngleTerm(ref obj, ref g, mesh, moveList, loop, angleSum, vertOrderInDup, 0, 1, w6);
            }

            int ii = 0;
            foreach (int[] loop in DoneMirror2)
            {
                DFVertAngleTerm(ref obj, ref g, mesh, moveList, loop, angleSum, vertOrderInDup, 0, 1, w4);

                double aveLength = aveLengthDone[ii];
                List<double> loopLength = DoneLength[ii];
                List<double> loopSin = DoneSin[ii];
                List<double> loopCos = DoneCos[ii];
                double s = CalcDoneSinFactor(loopLength, loopSin);
                int n = loopLength.Count;

                double value = (n + 1) * Math.PI / aveLength;
                double E1 = s * value;
                obj += E1 * E1 * w5 * 0.5;

                if (g != null)
                {
                    double factor = w5 * value * E1;

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
                        diffSelf *= factor;
                        g[dupIndex * 3] += diffSelf.X;
                        g[dupIndex * 3 + 1] += diffSelf.Y;
                        g[dupIndex * 3 + 2] += diffSelf.Z;
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
                        diffOther *= factor;
                        g[gIndex] += diffOther.X; ;
                        g[gIndex + 1] += diffOther.Y;
                        g[gIndex + 2] += diffOther.Z;
                    }
                }

                ii += 1;
            }

            ii = 0;
            foreach (int[] loop in Fblocks2)
            {
                DFVertAngleTerm(ref obj, ref g, mesh, moveList, loop, angleSum, vertOrderInDup, 2, 4, w2);


                
                double aveLength = aveLengthF[ii];
                List<double> loopLength = FblocksLength[ii];
                List<double> loopSin = FblocksSin[ii];
                List<double> loopCos = FblocksCos[ii];
                int n = loopCos.Count;
                double value = (n + 4) * Math.PI / aveLength;

                
                //-----------------------------------------------sinのエネルギー----------------------------------------
                double s = CalcFSinFactor(loopLength, loopSin);
                double E1 = s * value;
                obj += E1 * E1 * w3 * 0.5;
                

                
                //-----------------------------------------------cosのエネルギー----------------------------------------
                double c = CalcFCosFactor(loopLength, loopCos);
                double E2 = c * value;
                obj += E2 * E2 * w3 * 0.5;
                

                if (g != null)
                {
                    
                    double factor = w3 * value * E1;

                    HashSet<int> skipVert = new HashSet<int>();
                    int loopFirst = loop[0];
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
                        diffSelf *= factor;
                        if (move[0]) { g[dupIndex * 3] += diffSelf.X; }
                        if (move[1]) { g[dupIndex * 3 + 1] += diffSelf.Y; }
                        if (move[2]) { g[dupIndex * 3 + 2] += diffSelf.Z; }
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
                        diffOther *= factor;
                        g[gIndex] += diffOther.X; ;
                        g[gIndex + 1] += diffOther.Y;
                        g[gIndex + 2] += diffOther.Z;
                    }
                    
                    
                    //-----------------------------------------------cosの項----------------------------------------
                    factor = w3 * value * E2;
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
                        diffSelf *= factor;
                        if (move[0]) { g[dupIndex * 3] += diffSelf.X; }
                        if (move[1]) { g[dupIndex * 3 + 1] += diffSelf.Y; }
                        if (move[2]) { g[dupIndex * 3 + 2] += diffSelf.Z; }
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
                        diffOther *= factor;
                        g[gIndex] += diffOther.X; ;
                        g[gIndex + 1] += diffOther.Y;
                        g[gIndex + 2] += diffOther.Z;
                    }
                    
                }
               
                

                ii += 1;
            }

            return obj;
        }

        // 既存のメソッドは Evaluate に委譲してもよい
        public override double GetEnergy(double[] x) => Evaluate(x, null);
        public override void AddGradient(double[] x, double[] g) => Evaluate(x, g);
    }

    //数値微分の場合、めっちゃ重いので非推奨
    public class DevelopableConsiderOtherEnergy2 : EnergyTerm
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

        public DevelopableConsiderOtherEnergy2(double weight, CutMesh preMesh, List<List<List<int>>> category,
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

        //------------------------------GetEnergyなどの関数でx[]以外に使いたい変数をここに書いておく。



        public override double Evaluate(double[] x, double[] g)
        {
            double w1 = w[0] * Weight;
            double w2 = w[1] * Weight;
            double w3 = w[2] * Weight;
            double w4 = w[3] * Weight;
            double w5 = w[4] * Weight;
            double w6 = w[5] * Weight;

            int vertCount = preMesh.Vertices.Count;

            //現在のx[]からcutMeshを作成
            List<Point3d> vertices = new List<Point3d>();
            for (int i = 0; i < vertCount; i++)
            {
                int id = vertOrderInDup[i] * 3;
                vertices.Add( new Point3d(x[id], x[id + 1], x[id + 2]));
            }
            List<List<int>> duplicatedVertIndices = preMesh.DuplicatedVertIndices;
            CutMesh mesh = new CutMesh(vertices, preMesh.Faces, preMesh.Edges, duplicatedVertIndices);


            double[] angleSum = AngleSum(mesh);
            double obj = CGNRObjectiveCalc(duplicatedVertIndices, vertices, angleSum, category, w, aveLengthDone, aveLengthF);

            double delta = 1e-3;
            if (g != null)
            {
                for (int i = 0; i < duplicatedVertIndices.Count; i++)
                {
                    bool[] move = moveList[duplicatedVertIndices[i][0]];
                    if (move[0])
                    {
                        List<Point3d> verticesDp = new List<Point3d>(vertices);
                        List<Point3d> verticesDm = new List<Point3d>(vertices);
                        foreach (int vert in duplicatedVertIndices[i])
                        {
                            verticesDp[vert] += new Vector3d(delta, 0, 0);
                            verticesDm[vert] -= new Vector3d(delta, 0, 0);
                        }
                        CutMesh meshP = new CutMesh(verticesDp, preMesh.Faces, preMesh.Edges, duplicatedVertIndices);
                        CutMesh meshM = new CutMesh(verticesDm, preMesh.Faces, preMesh.Edges, duplicatedVertIndices);
                        double[] angleSumP = AngleSum(meshP);
                        double[] angleSumM = AngleSum(meshM);
                        double objP = CGNRObjectiveCalc(duplicatedVertIndices, verticesDp, angleSumP, category, w, aveLengthDone, aveLengthF);
                        double objM = CGNRObjectiveCalc(duplicatedVertIndices, verticesDm, angleSumM, category, w, aveLengthDone, aveLengthF);
                        g[i * 3] += (objP - objM) * 0.5 / delta;
                    }
                    if (move[1])
                    {
                        List<Point3d> verticesDp = new List<Point3d>(vertices);
                        List<Point3d> verticesDm = new List<Point3d>(vertices);
                        foreach (int vert in duplicatedVertIndices[i])
                        {
                            verticesDp[vert] += new Vector3d(0, delta, 0);
                            verticesDm[vert] -= new Vector3d(0, delta, 0);
                        }
                        CutMesh meshP = new CutMesh(verticesDp, preMesh.Faces, preMesh.Edges, duplicatedVertIndices);
                        CutMesh meshM = new CutMesh(verticesDm, preMesh.Faces, preMesh.Edges, duplicatedVertIndices);
                        double[] angleSumP = AngleSum(meshP);
                        double[] angleSumM = AngleSum(meshM);
                        double objP = CGNRObjectiveCalc(duplicatedVertIndices, verticesDp, angleSumP, category, w, aveLengthDone, aveLengthF);
                        double objM = CGNRObjectiveCalc(duplicatedVertIndices, verticesDm, angleSumM, category, w, aveLengthDone, aveLengthF);
                        g[i * 3 + 1] += (objP - objM) * 0.5 / delta;
                    }
                    if (move[2])
                    {
                        List<Point3d> verticesDp = new List<Point3d>(vertices);
                        List<Point3d> verticesDm = new List<Point3d>(vertices);
                        foreach (int vert in duplicatedVertIndices[i])
                        {
                            verticesDp[vert] += new Vector3d(0, 0, delta);
                            verticesDm[vert] -= new Vector3d(0, 0, delta);
                        }
                        CutMesh meshP = new CutMesh(verticesDp, preMesh.Faces, preMesh.Edges, duplicatedVertIndices);
                        CutMesh meshM = new CutMesh(verticesDm, preMesh.Faces, preMesh.Edges, duplicatedVertIndices);
                        double[] angleSumP = AngleSum(meshP);
                        double[] angleSumM = AngleSum(meshM);
                        double objP = CGNRObjectiveCalc(duplicatedVertIndices, verticesDp, angleSumP, category, w, aveLengthDone, aveLengthF);
                        double objM = CGNRObjectiveCalc(duplicatedVertIndices, verticesDm, angleSumM, category, w, aveLengthDone, aveLengthF);
                        g[i * 3 + 2] += (objP - objM) * 0.5 / delta;
                    }
                }
            }
            return obj;
        }

        // 既存のメソッドは Evaluate に委譲してもよい
        public override double GetEnergy(double[] x) => Evaluate(x, null);
        public override void AddGradient(double[] x, double[] g) => Evaluate(x, g);
    }

    //-------------------------------------------- ２次元用、dupVertはバラバラに移動 -----------------------------------------


    //エッジ長さ保存エネルギー (E = 0.5 * w * (L - L0)^2)、等式制約条件: L - L0 = 0
    public class EdgeLengthEnergy2D : EnergyTerm
    {
        //------------------------------GetEnergyなどの関数でx[]以外に使いたい変数をここに書いておく。
        private List<int[]> edges;
        private double[] initialLength;

        public EdgeLengthEnergy2D(double weight, List<int[]> edges, double[] initialLength) : base(weight)
        {
            //重複しているエッジは取り除いたエッジのリスト
            this.edges = edges;
            this.initialLength = initialLength;
        }
        //------------------------------GetEnergyなどの関数でx[]以外に使いたい変数をここに書いておく。



        public override double GetEnergy(double[] x)
        {
            double E = 0.0;
            for (int i = 0; i < edges.Count; i++)
            {
                int[] edge = edges[i];
                int i1 = edge[0] * 2;
                int i2 = edge[1] * 2;

                double dx = x[i1] - x[i2];
                double dy = x[i1 + 1] - x[i2 + 1];
                double len = Math.Sqrt(dx * dx + dy * dy);

                double diff = len - initialLength[i];
                E += 0.5 * Weight * diff * diff;
            }
            return E;
        }

        public override void AddGradient(double[] x, double[] g)
        {
            for (int i = 0; i < edges.Count; i++)
            {
                int[] edge = edges[i];
                int i1 = edge[0] * 2;
                int i2 = edge[1] * 2;

                double dx = x[i1] - x[i2];
                double dy = x[i1 + 1] - x[i2 + 1];
                double len = Math.Sqrt(dx * dx + dy * dy);

                // 0除算対策
                if (len < 1e-9) continue;

                // 勾配計算 (J^T * R)
                // Force magnitude = weight * (current - rest)
                double diff = len - initialLength[i];
                double factor = Weight * diff / len;

                double fx = factor * dx;
                double fy = factor * dy;

                // 頂点v1に加算
                g[i1] += fx;
                g[i1 + 1] += fy;

                // 頂点v2に減算 (反作用)
                g[i2] -= fx;
                g[i2 + 1] -= fy;
            }
        }
    }

    //面積バリアエネルギー (E = -w * ln(Area)) 、不等式制約条件:Area > 0
    public class AreaBarrier : EnergyTerm, IInequalityConstraint
    {
        //符号付面積が正になるような方でfaceの三点の順番を決定する。
        //(x1-x0)(y2-y0)-(x2-x0)(y1-y0)で計算する。
        private int[,] facesWithOrder;
        private double minArea;

        public AreaBarrier(double weight,　int[,] facesWithOrder, double minArea = 1e-9)
            : base(weight)
        {
            this.facesWithOrder = facesWithOrder;
            this.minArea = minArea;
        }

        private double SignedArea(double[] x, int faceIndex)
        {
            int v0 = facesWithOrder[faceIndex,0];
            int v1 = facesWithOrder[faceIndex, 1];
            int v2 = facesWithOrder[faceIndex, 2];
            double x0 = x[v0 * 2], y0 = x[v0 * 2 + 1];
            double x1 = x[v1 * 2], y1 = x[v1 * 2 + 1];
            double x2 = x[v2 * 2], y2 = x[v2 * 2 + 1];
            return 0.5 * ((x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0));
        }

        public bool IsSatisfied(double[] x)
        {
            for (int i = 0; i <  facesWithOrder.GetLength(0); i++)
            {
                if (SignedArea(x, i) < minArea) return false;
            }
            return true;
        }

        public override double GetEnergy(double[] x)
        {
            double E = 0.0;
            for (int i = 0; i < facesWithOrder.GetLength(0); i++)
            {
                double area = SignedArea(x, i);
                if (area <= 0) return double.PositiveInfinity;
                E += -Weight * Math.Log(area);
            }
            return E;
        }

        public override void AddGradient(double[] x, double[] g)
        {
            for (int i = 0; i < facesWithOrder.GetLength(0); i++)
            {
                double area = SignedArea(x, i);
                if (area <= 1e-12) continue;

                double factor = -Weight / area;

                int v0 = facesWithOrder[i, 0];
                int v1 = facesWithOrder[i, 1];
                int v2 = facesWithOrder[i, 2];
                double x0 = x[v0 * 2], y0 = x[v0 * 2 + 1];
                double x1 = x[v1 * 2], y1 = x[v1 * 2 + 1];
                double x2 = x[v2 * 2], y2 = x[v2 * 2 + 1];

                g[v0 * 2] += factor * 0.5 * (y1 - y2);
                g[v0 * 2 + 1] += factor * 0.5 * (x2 - x1);
                g[v1 * 2] += factor * 0.5 * (y2 - y0);
                g[v1 * 2 + 1] += factor * 0.5 * (x0 - x2);
                g[v2 * 2] += factor * 0.5 * (y0 - y1);
                g[v2 * 2 + 1] += factor * 0.5 * (x1 - x0);
            }
        }
    }

    //符号付面積を目標面積に近づける
    public class AreaEnergy2D : EnergyTerm
    {
        //------------------------------GetEnergyなどの関数でx[]以外に使いたい変数をここに書いておく。
        //符号付面積が正になるような方でfaceの三点の順番を決定する。
        //(x1-x0)(y2-y0)-(x2-x0)(y1-y0)で計算する。
        private int[,] facesWithOrder;
        private double[] initialAbsArea;

        public AreaEnergy2D(double weight, int[,] facesWithOrder, double[] initialAbsArea) : base(weight)
        {
            //重複しているエッジは取り除いたエッジのリスト
            this.facesWithOrder = facesWithOrder;
            this.initialAbsArea = initialAbsArea;
        }
        //------------------------------GetEnergyなどの関数でx[]以外に使いたい変数をここに書いておく。

        private double SignedArea(double[] x, int faceIndex)
        {
            int v0 = facesWithOrder[faceIndex, 0];
            int v1 = facesWithOrder[faceIndex, 1];
            int v2 = facesWithOrder[faceIndex, 2];
            double x0 = x[v0 * 2], y0 = x[v0 * 2 + 1];
            double x1 = x[v1 * 2], y1 = x[v1 * 2 + 1];
            double x2 = x[v2 * 2], y2 = x[v2 * 2 + 1];
            return 0.5 * ((x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0));
        }

        public override double GetEnergy(double[] x)
        {
            double E = 0.0;
            for (int i = 0; i < initialAbsArea.Length; i++)
            {
                double area = SignedArea(x, i);

                double diff = area - initialAbsArea[i];
                E += 0.5 * Weight * diff * diff;
            }
            return E;
        }

        public override void AddGradient(double[] x, double[] g)
        {
            for (int i = 0; i < initialAbsArea.Length; i++)
            {
                double area = SignedArea(x, i);

                double diff = area - initialAbsArea[i];

                double factor = Weight * diff;

                int v0 = facesWithOrder[i, 0];
                int v1 = facesWithOrder[i, 1];
                int v2 = facesWithOrder[i, 2];
                double x0 = x[v0 * 2], y0 = x[v0 * 2 + 1];
                double x1 = x[v1 * 2], y1 = x[v1 * 2 + 1];
                double x2 = x[v2 * 2], y2 = x[v2 * 2 + 1];

                g[v0 * 2] += factor * 0.5 * (y1 - y2);
                g[v0 * 2 + 1] += factor * 0.5 * (x2 - x1);
                g[v1 * 2] += factor * 0.5 * (y2 - y0);
                g[v1 * 2 + 1] += factor * 0.5 * (x0 - x2);
                g[v2 * 2] += factor * 0.5 * (y0 - y1);
                g[v2 * 2 + 1] += factor * 0.5 * (x1 - x0);
            }
        }
    }
}
