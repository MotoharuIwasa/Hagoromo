using Grasshopper.Kernel;
using Hagoromo.GeometryTools;
using Hagoromo.MathTools;
using MathNet.Numerics;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Collections;
using Rhino.Render.ChangeQueue;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

namespace Hagoromo.DevelopableMesh
{
    public static class CutChoiceTools
    {

        //主曲率の絶対値の大きい方を返す、ただし境界上の点では0とする
        public static double[] BiggerAbsPrinciple(CutMesh mesh)
        {
            double[,] principle = CutMeshCalcTools.PrincipleCurvature(mesh);
            double[] big = new double[principle.GetLength(0)];
            List<int> boundaryVertIndices = mesh.BoundaryVertIndices();
            for (int i = 0; i < principle.GetLength(0); i++)
            {
                if (boundaryVertIndices.Contains(i))
                {
                    continue;
                }
                double a = Math.Abs(principle[i, 0]);
                double b = Math.Abs(principle[i, 1]);
                big[i] = Math.Max(a, b);
            }
            return big;
        }

        //ループにならないようにする、枝分かれなし、一点からスタート、主曲率が大きい点を選んでいく
        public static List<int> AvoidLoopOneBranchOneSeed(CutMesh mesh, double maxLength)
        {
            //-------------chankA---------------------------------------------------------------------
            double[] principle = BiggerAbsPrinciple(mesh);
            List<int> sortedIndices = principle
            .Select((value, index) => new { value, index })
            .OrderByDescending(x => x.value)
            .Select(x => x.index)
            .ToList();
            int maxIndex = sortedIndices[0];
            List<int> arroundVertices = mesh.GetVerticesForVertex(maxIndex);
            int secondVertex = sortedIndices.FirstOrDefault(index => arroundVertices.Contains(index));
            int firstEdge = mesh.GetEdgeForEndPoints(maxIndex, secondVertex);

            List<int> cutEdgeIndices = new List<int> { firstEdge };
            List<int> cutVertices = new List<int> { maxIndex, secondVertex };
            double totalLength = mesh.GetEdgeLine(firstEdge).Length;
            int[] endPoints = new int[] { maxIndex, secondVertex };
            //-------------chankA---------------------------------------------------------------------

            while (totalLength < maxLength)
            {
                List<int> possibleVertices1 = mesh.GetVerticesForVertex(endPoints[0]);
                List<int> possibleVertices2 = mesh.GetVerticesForVertex(endPoints[1]);
                possibleVertices1.RemoveAll(v => cutVertices.Contains(v));
                possibleVertices2.RemoveAll(v => cutVertices.Contains(v));
                int vertex1 = sortedIndices.FirstOrDefault(index => possibleVertices1.Contains(index));
                int vertex2 = sortedIndices.FirstOrDefault(index => possibleVertices2.Contains(index));
                int newEdge = 0;
                if (sortedIndices.IndexOf(vertex1) <= sortedIndices.IndexOf(vertex2))
                {
                    newEdge = mesh.GetEdgeForEndPoints(endPoints[0], vertex1);
                    endPoints[0] = vertex1;
                    cutVertices.Add(vertex1);
                }
                else
                {
                    newEdge = mesh.GetEdgeForEndPoints(endPoints[1], vertex2);
                    endPoints[1] = vertex2;
                    cutVertices.Add(vertex2);
                }
                cutEdgeIndices.Add(newEdge);
                totalLength += mesh.GetEdgeLine(newEdge).Length;
            }
            return cutEdgeIndices;
        }

        //ループにならないようにする、枝分かれあり、一点からスタート、主曲率が大きい点を選んでいく
        public static List<int> AvoidLoopMultiBranchesOneSeed(CutMesh mesh, double maxLength)
        {
            //-------------chankA---------------------------------------------------------------------
            double[] principle = BiggerAbsPrinciple(mesh);
            List<int> sortedIndices = principle
            .Select((value, index) => new { value, index })
            .OrderByDescending(x => x.value)
            .Select(x => x.index)
            .ToList();
            int maxIndex = sortedIndices[0];
            List<int> cutVertices = new List<int> { maxIndex };
            List<int> cutEdgeIndices = new List<int>();
            double totalLength = 0;
            //-------------chankA---------------------------------------------------------------------
            while (totalLength < maxLength)
            {
                List<int> possibleVertices = new List<int>();
                foreach (int vertex in cutVertices)
                {
                    possibleVertices.AddRange(mesh.GetVerticesForVertex(vertex));
                }
                possibleVertices = possibleVertices.Distinct().ToList();
                possibleVertices.RemoveAll(v => cutVertices.Contains(v));

                int newVertex = sortedIndices.FirstOrDefault(index => possibleVertices.Contains(index));
                List<int> possibleEdgesEndVerts = mesh.GetVerticesForVertex(newVertex);
                possibleEdgesEndVerts = possibleEdgesEndVerts.Intersect(cutVertices).ToList();
                int newEdgesEndVert = sortedIndices.FirstOrDefault(index => possibleEdgesEndVerts.Contains(index));
                int newEdge = mesh.GetEdgeForEndPoints(newEdgesEndVert, newVertex);

                cutVertices.Add(newVertex);
                cutEdgeIndices.Add(newEdge);
                totalLength += mesh.GetEdgeLine(newEdge).Length;
            }
            return cutEdgeIndices;
        }

        //ループにならないようにする、枝分かれなし、複数点からスタート、主曲率が大きい点を選んでいく、maxLengthはseedごとに与える
        public static List<int>[] AvoidLoopOneBranchMultiSeeds(CutMesh mesh, double[] maxLength, int seedCount, double distance)
        {
            //-------------chankA---------------------------------------------------------------------
            double[] principle = BiggerAbsPrinciple(mesh);
            List<int> sortedIndices = principle
            .Select((value, index) => new { value, index })
            .OrderByDescending(x => x.value)
            .Select(x => x.index)
            .ToList();
            int[] seedVertIndices = GetSeedIndices(mesh, seedCount, distance);

            List<int>[] cutEdgeIndices = new List<int>[seedCount];
            List<int>[] cutVertices = new List<int>[seedCount];
            double[] totalLength = new double[seedCount];
            int[,] endPoints = new int[seedCount, 2];

            for (int i = 0; i < seedCount; i++)
            {
                int firstVertex = seedVertIndices[i];
                cutEdgeIndices[i] = new List<int>();
                cutVertices[i] = new List<int>();
                cutVertices[i].Add(firstVertex);

                List<int> arroundVertices = mesh.GetVerticesForVertex(firstVertex);
                int secondVertex = sortedIndices.FirstOrDefault(index => arroundVertices.Contains(index));
                int firstEdge = mesh.GetEdgeForEndPoints(firstVertex, secondVertex);

                cutEdgeIndices[i].Add(firstEdge);
                cutVertices[i].Add(secondVertex);
                endPoints[i, 0] = firstVertex;
                endPoints[i, 1] = secondVertex;
            }

            //統合されて考えなくてよくなる場合はtrueにする。
            bool[] mergedSeed = new bool[seedCount];
            //mergeされた場合もoverLengthのところtrueにする。
            bool[] overLength = new bool[seedCount];
            //-------------chankA---------------------------------------------------------------------

            while (!overLength.All(x => x))
            {
                for (int i = 0; i < seedCount; i++)
                {
                    if (mergedSeed[i]) { continue; }

                    List<int> possibleVertices1 = mesh.GetVerticesForVertex(endPoints[i, 0]);
                    List<int> possibleVertices2 = mesh.GetVerticesForVertex(endPoints[i, 1]);
                    possibleVertices1.RemoveAll(v => cutVertices[i].Contains(v));
                    possibleVertices2.RemoveAll(v => cutVertices[i].Contains(v));
                    int vertex1 = sortedIndices.FirstOrDefault(index => possibleVertices1.Contains(index));
                    int vertex2 = sortedIndices.FirstOrDefault(index => possibleVertices2.Contains(index));
                    int newEdge = 0;
                    int newVertex = 0;
                    if (sortedIndices.IndexOf(vertex1) <= sortedIndices.IndexOf(vertex2))
                    {
                        newVertex = vertex1;
                        newEdge = mesh.GetEdgeForEndPoints(endPoints[i, 0], vertex1);
                        endPoints[i, 0] = vertex1;
                        cutVertices[i].Add(vertex1);
                    }
                    else
                    {
                        newVertex = vertex2;
                        newEdge = mesh.GetEdgeForEndPoints(endPoints[i, 1], vertex2);
                        endPoints[i, 1] = vertex2;
                        cutVertices[i].Add(vertex2);
                    }
                    cutEdgeIndices[i].Add(newEdge);
                    totalLength[i] += mesh.GetEdgeLine(newEdge).Length;
                    if (totalLength[i] > maxLength[i]) { overLength[i] = true; }
                    for (int j = 0; j < seedCount; j++)
                    {
                        if (mergedSeed[j] || i == j) { continue; }
                        if (cutVertices[j].Contains(newVertex))
                        {
                            //以降はiの方で考えていく
                            mergedSeed[j] = true;
                            overLength[j] = true;
                            totalLength[i] += totalLength[j];
                            maxLength[i] += maxLength[j];
                            cutVertices[i].AddRange(cutVertices[j]);
                            cutVertices[i] = cutVertices[i].Distinct().ToList();
                            cutEdgeIndices[i].AddRange(cutEdgeIndices[j]);
                        }
                    }
                }
            }
            return cutEdgeIndices;
        }

        //ループにならないようにする、枝分かれあり、複数点からスタート、主曲率が大きい点を選んでいく、maxLengthはseedごとに与える
        public static List<int>[] AvoidLoopMultiBranchesMultiSeeds(CutMesh mesh, double[] maxLength, int seedCount, double distance)
        {
            //-------------chankA---------------------------------------------------------------------
            double[] principle = BiggerAbsPrinciple(mesh);
            List<int> sortedIndices = principle
            .Select((value, index) => new { value, index })
            .OrderByDescending(x => x.value)
            .Select(x => x.index)
            .ToList();

            int[] seedVertIndices = GetSeedIndices(mesh, seedCount, distance);

            List<int>[] cutEdgeIndices = new List<int>[seedCount];
            List<int>[] cutVertices = new List<int>[seedCount];
            double[] totalLength = new double[seedCount];

            for (int i = 0; i < seedCount; i++)
            {
                cutEdgeIndices[i] = new List<int>();
                cutVertices[i] = new List<int>();
                cutVertices[i].Add(seedVertIndices[i]);
            }

            //統合されて考えなくてよくなる場合はtrueにする。
            bool[] mergedSeed = new bool[seedCount];
            //mergeされた場合もoverLengthのところtrueにする。
            bool[] overLength = new bool[seedCount];
            //-------------chankA---------------------------------------------------------------------

            while (!overLength.All(x => x))
            {
                for (int i = 0; i < seedCount; i++)
                {
                    if (mergedSeed[i]) { continue; }
                    List<int> possibleVertices = new List<int>();
                    foreach (int vertex in cutVertices[i])
                    {
                        possibleVertices.AddRange(mesh.GetVerticesForVertex(vertex));
                    }
                    possibleVertices = possibleVertices.Distinct().ToList();
                    possibleVertices.RemoveAll(v => cutVertices[i].Contains(v));

                    int newVertex = sortedIndices.FirstOrDefault(index => possibleVertices.Contains(index));
                    List<int> possibleEdgesEndVerts = mesh.GetVerticesForVertex(newVertex);
                    possibleEdgesEndVerts = possibleEdgesEndVerts.Intersect(cutVertices[i]).ToList();
                    int newEdgesEndVert = sortedIndices.FirstOrDefault(index => possibleEdgesEndVerts.Contains(index));
                    int newEdge = mesh.GetEdgeForEndPoints(newEdgesEndVert, newVertex);

                    cutVertices[i].Add(newVertex);
                    cutEdgeIndices[i].Add(newEdge);
                    totalLength[i] += mesh.GetEdgeLine(newEdge).Length;
                    if (totalLength[i] > maxLength[i]) { overLength[i] = true; }
                    for (int j = 0; j < seedCount; j++)
                    {
                        if (mergedSeed[j] || i == j) { continue; }
                        if (cutVertices[j].Contains(newVertex))
                        {
                            //以降はiの方で考えていく
                            mergedSeed[j] = true;
                            overLength[j] = true;
                            totalLength[i] += totalLength[j];
                            maxLength[i] += maxLength[j];
                            cutVertices[i].AddRange(cutVertices[j]);
                            cutVertices[i] = cutVertices[i].Distinct().ToList();
                            cutEdgeIndices[i].AddRange(cutEdgeIndices[j]);
                        }
                    }
                }
            }
            return cutEdgeIndices;
        }

        //ループにならないようにする、枝分かれあり、複数点からスタート、主曲率が大きい点を選んでいく、maxLengthはseedごとに与える
        public static List<int>[] AvoidLoopMultiBranchesMultiSeedsNoMerge(CutMesh mesh, double[] maxLength, int seedCount, double distance)
        {
            //-------------chankA---------------------------------------------------------------------
            double[] principle = BiggerAbsPrinciple(mesh);
            List<int> sortedIndices = principle
            .Select((value, index) => new { value, index })
            .OrderByDescending(x => x.value)
            .Select(x => x.index)
            .ToList();

            int[] seedVertIndices = GetSeedIndices(mesh, seedCount, distance);

            List<int>[] cutEdgeIndices = new List<int>[seedCount];
            List<int>[] cutVertices = new List<int>[seedCount];
            double[] totalLength = new double[seedCount];

            for (int i = 0; i < seedCount; i++)
            {
                cutEdgeIndices[i] = new List<int>();
                cutVertices[i] = new List<int>();
                cutVertices[i].Add(seedVertIndices[i]);
            }

            bool[] overLength = new bool[seedCount];
            //-------------chankA---------------------------------------------------------------------

            while (!overLength.All(x => x))
            {
                for (int i = 0; i < seedCount; i++)
                {
                    List<int> possibleVertices = new List<int>();
                    foreach (int vertex in cutVertices[i])
                    {
                        possibleVertices.AddRange(mesh.GetVerticesForVertex(vertex));
                    }
                    possibleVertices = possibleVertices.Distinct().ToList();
                    possibleVertices.RemoveAll(v => cutVertices[i].Contains(v));

                    int newVertex = sortedIndices.FirstOrDefault(index => possibleVertices.Contains(index));
                    List<int> possibleEdgesEndVerts = mesh.GetVerticesForVertex(newVertex);
                    possibleEdgesEndVerts = possibleEdgesEndVerts.Intersect(cutVertices[i]).ToList();
                    int newEdgesEndVert = sortedIndices.FirstOrDefault(index => possibleEdgesEndVerts.Contains(index));
                    int newEdge = mesh.GetEdgeForEndPoints(newEdgesEndVert, newVertex);

                    cutVertices[i].Add(newVertex);
                    cutEdgeIndices[i].Add(newEdge);
                    totalLength[i] += mesh.GetEdgeLine(newEdge).Length;
                    if (totalLength[i] > maxLength[i]) { overLength[i] = true; }
                }
            }
            return cutEdgeIndices;
        }


        //ちゃんと離れているseedたちからスタートするようにする。
        public static int[] GetSeedIndices(CutMesh mesh, int seedCount, double distance)
        {
            double[] principle = BiggerAbsPrinciple(mesh);
            List<int> sortedIndices = principle
            .Select((value, index) => new { value, index })
            .OrderByDescending(x => x.value)
            .Select(x => x.index)
            .ToList();

            List<int> seedVertIndices = new List<int>() { sortedIndices[0] };
            int current = 0;

            while (seedVertIndices.Count < seedCount)
            {
                current += 1;
                bool tooClose = false;
                foreach (int i in seedVertIndices)
                {
                    if ((mesh.Vertices[i] - mesh.Vertices[sortedIndices[current]]).Length < distance)
                    {
                        tooClose = true;
                        break;
                    }
                }

                if (!tooClose)
                {
                    seedVertIndices.Add(sortedIndices[current]);
                }

            }
            return seedVertIndices.ToArray();
        }

        /*
        public static List<int> GetLongestTopoTree(CutMesh mesh, List<int> edgeIndices)
        {
            List<int>[] verticesConnection = new List<int>[mesh.Vertices.Count];
            for (int i = 0; i < mesh.Vertices.Count; i++)
            {
                verticesConnection[i] = new List<int>();
            }
            foreach (int edge in edgeIndices)
            {
                int v1 = mesh.Edges[edge][0];
                int v2 = mesh.Edges[edge][1];
                verticesConnection[v1].Add(v2);
                verticesConnection[v2].Add(v1);
            }
            List<int> doneVertices = new List<int>();
            for (int i = 0; i < mesh.Vertices.Count; i++)
            {
                if (verticesConnection[i].Count == 0)
                {
                    doneVertices.Add(i);
                }
            }

            int firstPoint = -1;
            for (int i = 0; i < mesh.Vertices.Count; i++)
            {
                if (doneVertices.Contains(i)) { continue; }
                firstPoint = i;
                doneVertices.Add(i);
                break;
            }

            List<int> currentVert = new List<int> { firstPoint };
            while (doneVertices.Count < mesh.Vertices.Count)
            {
                List<int> nextVert = new List<int>();
                foreach (int vert in currentVert)
                {
                    nextVert.AddRange(mesh.GetVerticesForVertex(vert));
                }
                nextVert.RemoveAll(v => doneVertices.Contains(v));
                doneVertices.AddRange(nextVert);
                currentVert = new List<int> (nextVert);
            }
            int start = currentVert[0];

            currentVert = new List<int> { start };
            while (doneVertices.Count < mesh.Vertices.Count)
            {
                List<int> nextVert = new List<int>();
                foreach (int vert in currentVert)
                {
                    nextVert.AddRange(mesh.GetVerticesForVertex(vert));
                }
                nextVert.RemoveAll(v => doneVertices.Contains(v));
                doneVertices.AddRange(nextVert);
                currentVert = new List<int>(nextVert);
            }

        }
        */

        // topology的な最遠ノードと親リストを返す
        public static (int farthest, Dictionary<int, int> parent) BFStopo(int start, List<int>[] verticesConnection)
        {
            Queue<int> q = new Queue<int>();
            Dictionary<int, int> dist = new Dictionary<int, int>();
            //最後の節点の親の親の親...をたどれば通った道がわかる。
            Dictionary<int, int> parentMap = new Dictionary<int, int>();

            q.Enqueue(start);
            dist[start] = 0;
            parentMap[start] = -1;

            while (q.Count > 0)
            {
                int cur = q.Dequeue();
                foreach (int nb in verticesConnection[cur])
                {
                    if (!dist.ContainsKey(nb))
                    {
                        dist[nb] = dist[cur] + 1;
                        parentMap[nb] = cur;
                        q.Enqueue(nb);
                    }
                }
            }

            // 最遠ノードを探す
            int farthestNode = start;
            int maxDist = -1;
            foreach (var kv in dist)
            {
                if (kv.Value > maxDist)
                {
                    maxDist = kv.Value;
                    farthestNode = kv.Key;
                }
            }

            return (farthestNode, parentMap);
        }

        // topology的な最遠ノードと親リストを返す
        public static (int farthest, Dictionary<int, int> parent) BFSlength(int start, List<int>[] verticesConnection, CutMesh mesh)
        {
            Queue<int> q = new Queue<int>();
            Dictionary<int, double> dist = new Dictionary<int, double>();
            //最後の節点の親の親の親...をたどれば通った道がわかる。
            Dictionary<int, int> parentMap = new Dictionary<int, int>();

            q.Enqueue(start);
            dist[start] = 0;
            parentMap[start] = -1;

            while (q.Count > 0)
            {
                int cur = q.Dequeue();
                foreach (int nb in verticesConnection[cur])
                {
                    if (!dist.ContainsKey(nb))
                    {
                        double distance = mesh.Vertices[nb].DistanceTo(mesh.Vertices[cur]);
                        dist[nb] = dist[cur] + distance;
                        parentMap[nb] = cur;
                        q.Enqueue(nb);
                    }
                }
            }

            // 最遠ノードを探す
            int farthestNode = start;
            double maxDist = -1;
            foreach (var kv in dist)
            {
                if (kv.Value > maxDist)
                {
                    maxDist = kv.Value;
                    farthestNode = kv.Key;
                }
            }

            return (farthestNode, parentMap);
        }

        //bool topoがtrueならtopology的に、falseなら距離的に一番長いツリーの通る順の節点番号たちを返す
        public static List<int> GetLongestTree(CutMesh mesh, List<int> edgeIndices, bool topo)
        {
            // --- 隣接リスト（選ばれたエッジのみ）を構築 ---
            List<int>[] verticesConnection = new List<int>[mesh.Vertices.Count];
            for (int i = 0; i < mesh.Vertices.Count; i++)
                verticesConnection[i] = new List<int>();

            foreach (int edge in edgeIndices)
            {
                int v1 = mesh.Edges[edge][0];
                int v2 = mesh.Edges[edge][1];
                verticesConnection[v1].Add(v2);
                verticesConnection[v2].Add(v1);
            }

            // --- 1つでも接続がある頂点をスタート候補にする ---
            int firstPoint = -1;
            for (int i = 0; i < verticesConnection.Length; i++)
            {
                if (verticesConnection[i].Count > 0)
                {
                    firstPoint = i;
                    break;
                }
            }
            if (firstPoint == -1) return new List<int>(); // エッジがない場合

            // --- 1回目の探索 ---
            int nodeA = 0;
            if (topo) { nodeA = BFStopo(firstPoint, verticesConnection).farthest; }
            else { nodeA = BFSlength(firstPoint, verticesConnection, mesh).farthest; }

            // --- 2回目の探索 ---
            Dictionary<int, int> parentMap2 = new Dictionary<int, int>();
            int nodeB = 0;
            if (topo)
            {
                var bfs2 = BFStopo(nodeA, verticesConnection);
                nodeB = bfs2.farthest;
                parentMap2 = bfs2.parent;
            }
            else
            {
                var bfs2 = BFSlength(nodeA, verticesConnection, mesh);
                nodeB = bfs2.farthest;
                parentMap2 = bfs2.parent;
            }

            // --- 経路を復元 ---
            List<int> path = new List<int>();
            int curNode = nodeB;
            while (curNode != -1)
            {
                path.Add(curNode);
                curNode = parentMap2[curNode];
            }
            return path;
        }

        //h1 - H1がpositiveかどうか、initialHeightはfaceの順で格納
        public static (List<Point3d> vecStart, List<List<bool>> positive, List<List<Vector3d>> edgeVec) EdgeVecList(List<List<double>> initialHeight, CutMesh mesh2d)
        {
            List<Point3d> vecStart = new List<Point3d>();
            List<List<bool>> positive = new List<List<bool>>();
            List<List<Vector3d>> edgeVec = new List<List<Vector3d>>();
            for (int i = 0; i < mesh2d.Edges.Count; i++)
            {
                List<int> faces = mesh2d.GetFacesForEdge(i);
                Point3d Q0 = mesh2d.Vertices[mesh2d.Edges[i][0]];
                Point3d Q1 = mesh2d.Vertices[mesh2d.Edges[i][1]];
                Vector3d Q0Q1 = Q1 - Q0;
                double l2 = Q0Q1.SquareLength;
                Point3d M = (Q0 + Q1) * 0.5;
                List<bool> bools = new List<bool>();
                List<Vector3d> vecs = new List<Vector3d>();
                int j = 0;
                foreach (int face in faces)
                {
                    vecStart.Add(M);
                    Point3d P = mesh2d.Vertices[mesh2d.GetVerticesForFace(face).Where(v => v != mesh2d.Edges[i][0] && v != mesh2d.Edges[i][1]).ToList()[0]];
                    Vector3d Q0P = P - Q0;
                    double dot = Q0Q1 * Q0P;
                    Vector3d HP = Q0P - (dot / l2) * Q0Q1;
                    double diff = HP.Length - initialHeight[i][j];
                    if (diff > 0) { bools.Add(true); }
                    else { bools.Add(false); }
                    HP.Unitize();
                    vecs.Add(diff * HP);
                    j++;
                }
                positive.Add(bools);
                edgeVec.Add(vecs);
            }
            List<bool> positiveFlat = positive.SelectMany(x => x).ToList();
            List<Vector3d> edgeVecFlat = edgeVec.SelectMany(x => x).ToList();
            return (vecStart, positive, edgeVec);
        }

        public static List<List<double>> EdgeVecDiff(List<List<double>> initialHeight, CutMesh mesh2d)
        {
            List<List<double>> diff = new List<List<double>>();
            List<List<bool>> positive = new List<List<bool>>();
            List<List<Vector3d>> edgeVec = new List<List<Vector3d>>();
            for (int i = 0; i < mesh2d.Edges.Count; i++)
            {
                List<int> faces = mesh2d.GetFacesForEdge(i);
                Point3d Q0 = mesh2d.Vertices[mesh2d.Edges[i][0]];
                Point3d Q1 = mesh2d.Vertices[mesh2d.Edges[i][1]];
                Vector3d Q0Q1 = Q1 - Q0;
                double l2 = Q0Q1.SquareLength;
                Point3d M = (Q0 + Q1) * 0.5;
                List<bool> bools = new List<bool>();
                List<Vector3d> vecs = new List<Vector3d>();
                List<double> doubles = new List<double>();
                int j = 0;
                foreach (int face in faces)
                {
                    Point3d P = mesh2d.Vertices[mesh2d.GetVerticesForFace(face).Where(v => v != mesh2d.Edges[i][0] && v != mesh2d.Edges[i][1]).ToList()[0]];
                    Vector3d Q0P = P - Q0;
                    double dot = Q0Q1 * Q0P;
                    Vector3d HP = Q0P - (dot / l2) * Q0Q1;
                    double delta = (HP.Length - initialHeight[i][j])/initialHeight[i][j];
                    doubles.Add(delta);
                    j++;
                }
                diff.Add(doubles);
            }
            return diff;
        }


        public static List<List<double>> GetInitialHeight(CutMesh mesh)
        {
            List<List<double>> initialHeight = new List<List<double>>();
            for (int i = 0; i < mesh.Edges.Count; i++)
            {
                List<double> height = new List<double>();
                List<int> faces = mesh.GetFacesForEdge(i);
                Point3d Q0 = mesh.Vertices[mesh.Edges[i][0]];
                Point3d Q1 = mesh.Vertices[mesh.Edges[i][1]];
                Vector3d Q0Q1 = Q1 - Q0;
                double l = Q0Q1.Length;
                foreach (int face in faces)
                {
                    Point3d P = mesh.Vertices[mesh.GetVerticesForFace(face).Where(v => v != mesh.Edges[i][0] && v != mesh.Edges[i][1]).ToList()[0]];
                    Vector3d Q0P = P - Q0;
                    double dot = Q0Q1 * Q0P;
                    Vector3d HP = Q0P - (dot / (l * l)) * Q0Q1;
                    height.Add(HP.Length);
                }
                initialHeight.Add(height);
            }
            return initialHeight;
        }

        public static List<int> FindNextCutEdge(CutMesh mesh3d, CutMesh mesh2d)
        {
            List<List<double>> initialHeight = GetInitialHeight(mesh3d);
            List<List<double>> edgeVecDiffs = EdgeVecDiff(initialHeight, mesh2d);
            List<double> DiffTotal = new List<double>();
            foreach (List<double> diffs in edgeVecDiffs)
            {
                if (diffs.Count <= 1) { DiffTotal.Add(double.MinValue); continue; }
                double total = 0;
                foreach (double diff in diffs) { total += diff; }
                DiffTotal.Add(total);
            }
            double maxValue = DiffTotal.Max();
            // 最大値のインデックスを取得
            int maxIndex = DiffTotal.IndexOf(maxValue);
            HashSet<int> boundaryVerts = mesh2d.BoundaryVertIndices().ToHashSet();
            int[] edgeVerts = mesh2d.Edges[maxIndex];
            if (boundaryVerts.Contains(edgeVerts[0]) || boundaryVerts.Contains(edgeVerts[1])) { return new List<int> { maxIndex }; }
            else
            {
                HashSet<int> edgeList = mesh2d.GetEdgesForVertex(edgeVerts[0]).ToHashSet();
                edgeList.UnionWith(mesh2d.GetEdgesForVertex(edgeVerts[1]).ToHashSet());
                edgeList.Remove(maxIndex);
                int secondIndex = edgeList.OrderByDescending(i => DiffTotal[i]).First();
                return new List<int> { maxIndex, secondIndex };
            }
        }

        public static List<int> FindNextCutEdgeNoBranch(CutMesh mesh3d, CutMesh mesh2d, int startVert, int endVert)
        {
            List<List<double>> initialHeight = GetInitialHeight(mesh3d);
            List<List<double>> edgeVecDiffs = EdgeVecDiff(initialHeight, mesh2d);
            List<double> DiffTotal = new List<double>();
            foreach (List<double> diffs in edgeVecDiffs)
            {
                if (diffs.Count <= 1) { DiffTotal.Add(double.MinValue); continue; }
                double total = 0;
                foreach (double diff in diffs) { total += diff; }
                DiffTotal.Add(total);
            }
            double maxValue = DiffTotal.Max();
            // 最大値のインデックスを取得
            int maxIndex = DiffTotal.IndexOf(maxValue);
            HashSet<int> boundaryVerts = mesh2d.BoundaryVertIndices().ToHashSet();
            int[] edgeVerts = mesh2d.Edges[maxIndex];
            if (boundaryVerts.Contains(edgeVerts[0]) || boundaryVerts.Contains(edgeVerts[1])) { return new List<int> { maxIndex }; }
            else
            {
                HashSet<int> edgeList = mesh2d.GetEdgesForVertex(edgeVerts[0]).ToHashSet();
                edgeList.UnionWith(mesh2d.GetEdgesForVertex(edgeVerts[1]).ToHashSet());
                edgeList.Remove(maxIndex);
                int secondIndex = edgeList.OrderByDescending(i => DiffTotal[i]).First();
                return new List<int> { maxIndex, secondIndex };
            }
        }
    }
}