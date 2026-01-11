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
using static Hagoromo.GeometryTools.MeshCutTools;
using static Hagoromo.GeometryTools.CutMeshCalcTools;
using static Hagoromo.DevelopableMesh.DevelopableTools;

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
                    //double delta = (HP.Length - initialHeight[i][j])/initialHeight[i][j];
                    double delta = HP.Length - initialHeight[i][j];
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

        public static List<int> FindNextCutEdge(CutMesh mesh3d, CutMesh mesh2d, List<BoundaryLoop> loops)
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

            List<int> order = DiffTotal.Select((value, index) => new { value, index }).OrderByDescending(x => x.value).Select(x => x.index).ToList();
            int count = 0;

            while (count < 20)
            {
                int maxIndex = order[0];
                order.RemoveAt(0);
                int[] edgeVerts = mesh2d.Edges[maxIndex];
                //０なら次のステップへ、1なら
                int loopFlag = 0;
                foreach (BoundaryLoop loop in loops)
                {
                    HashSet<int> loopVert = loop.VertexIndices.ToHashSet();
                    if (loopVert.Contains(edgeVerts[0]) && loopVert.Contains(edgeVerts[1])) { loopFlag = 1; break; }
                    if ((loopVert.Contains(edgeVerts[0]) || loopVert.Contains(edgeVerts[1]))) { loopFlag = 2; break; }
                }
                if (loopFlag == 1) { count += 1; continue; }
                if (loopFlag == 0)
                {
                    HashSet<int> edgeList = mesh2d.GetEdgesForVertex(edgeVerts[0]).ToHashSet();
                    edgeList.UnionWith(mesh2d.GetEdgesForVertex(edgeVerts[1]).ToHashSet());
                    edgeList.Remove(maxIndex);
                    int secondIndex = edgeList.OrderByDescending(i => DiffTotal[i]).First();
                    return new List<int> { maxIndex, secondIndex };
                }
                if (loopFlag == 2) { return new List<int> { maxIndex }; }
            }
            return null;
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




        //----------------------------------------------------------------------------------------------------------------------------------------------------------------------------

        public static List<int> FindShortestPathEdges(CutMesh cutMesh, List<double> edgeCost, int startIndex, int endIndex)
        {
            // 隣接キャッシュの更新を確認
            cutMesh.ReloadVertexToEdgesCache();

            int vertexCount = cutMesh.Vertices.Count;
            double[] minCostToVertex = new double[vertexCount];
            int[] edgeToVertex = new int[vertexCount]; // どのエッジを通ってこの頂点に来たか
            int[] parentVertex = new int[vertexCount]; // どの頂点から来たか

            for (int i = 0; i < vertexCount; i++)
            {
                minCostToVertex[i] = double.PositiveInfinity;
                edgeToVertex[i] = -1;
                parentVertex[i] = -1;
            }

            // (コスト, 頂点インデックス) のペアで管理
            // 重複を避けるため、コストが同じでもインデックスが違えば別ノードとして扱うComparerを使用
            var priorityQueue = new SortedSet<(double cost, int vIdx)>(Comparer<(double cost, int vIdx)>.Create((a, b) =>
            {
                int res = a.cost.CompareTo(b.cost);
                return res != 0 ? res : a.vIdx.CompareTo(b.vIdx);
            }));

            minCostToVertex[startIndex] = 0;
            priorityQueue.Add((0, startIndex));

            while (priorityQueue.Count > 0)
            {
                var current = priorityQueue.Min;
                priorityQueue.Remove(current);

                int u = current.vIdx;

                // ゴールに到達したら終了
                if (u == endIndex) break;
                if (current.cost > minCostToVertex[u]) continue;

                // 頂点uに接続されているエッジを探索
                List<int> connectedEdges = cutMesh.GetEdgesForVertex(u);
                foreach (int eIdx in connectedEdges)
                {
                    int[] edge = cutMesh.Edges[eIdx];
                    int v = (edge[0] == u) ? edge[1] : edge[0]; // 隣接頂点

                    double newCost = minCostToVertex[u] + edgeCost[eIdx];

                    if (newCost < minCostToVertex[v])
                    {
                        priorityQueue.Remove((minCostToVertex[v], v));
                        minCostToVertex[v] = newCost;
                        edgeToVertex[v] = eIdx;
                        parentVertex[v] = u;
                        priorityQueue.Add((newCost, v));
                    }
                }
            }

            // --- 経路の復元 ---
            List<int> pathEdges = new List<int>();
            if (edgeToVertex[endIndex] == -1) return pathEdges; // 到達不能

            int currV = endIndex;
            while (currV != startIndex)
            {
                int eIdx = edgeToVertex[currV];
                pathEdges.Add(eIdx);
                currV = parentVertex[currV];
            }

            pathEdges.Reverse(); // ゴールから辿ったので反転
            return pathEdges;
        }

        public static List<int> FindShortestPathToBoundary(CutMesh cutMesh, List<double> edgeCost, int startIndex)
        {
            // 1. 境界頂点のセットを高速検索用に準備
            HashSet<int> boundarySet = new HashSet<int>(cutMesh.BoundaryVertIndices());

            // 開始点自体が境界の場合は即終了
            if (boundarySet.Contains(startIndex)) return new List<int>();

            int vertexCount = cutMesh.Vertices.Count;
            double[] minCostToVertex = new double[vertexCount];
            int[] edgeToVertex = new int[vertexCount]; // どのエッジを通って来たか
            int[] parentVertex = new int[vertexCount]; // どの頂点から来たか

            for (int k = 0; k < vertexCount; k++)
            {
                minCostToVertex[k] = double.PositiveInfinity;
                edgeToVertex[k] = -1;
                parentVertex[k] = -1;
            }

            // (現在のコスト, 頂点インデックス) の優先度付きキュー
            var priorityQueue = new SortedSet<(double cost, int vIdx)>(Comparer<(double cost, int vIdx)>.Create((a, b) =>
            {
                int res = a.cost.CompareTo(b.cost);
                return res != 0 ? res : a.vIdx.CompareTo(b.vIdx);
            }));

            minCostToVertex[startIndex] = 0;
            priorityQueue.Add((0, startIndex));

            int reachedBoundaryVertex = -1;

            // 2. 最短経路探索（ダイクストラ法）
            while (priorityQueue.Count > 0)
            {
                var current = priorityQueue.Min;
                priorityQueue.Remove(current);

                int u = current.vIdx;

                // 境界に到達したか判定
                if (boundarySet.Contains(u))
                {
                    reachedBoundaryVertex = u;
                    break; // 最短の境界が見つかったので終了
                }

                if (current.cost > minCostToVertex[u]) continue;

                // 隣接エッジの探索
                foreach (int eIdx in cutMesh.GetEdgesForVertex(u))
                {
                    int[] edge = cutMesh.Edges[eIdx];
                    int v = (edge[0] == u) ? edge[1] : edge[0];

                    double newCost = minCostToVertex[u] + edgeCost[eIdx];

                    if (newCost < minCostToVertex[v])
                    {
                        priorityQueue.Remove((minCostToVertex[v], v));
                        minCostToVertex[v] = newCost;
                        edgeToVertex[v] = eIdx;
                        parentVertex[v] = u;
                        priorityQueue.Add((newCost, v));
                    }
                }
            }

            // 3. 経路の復元
            List<int> pathEdges = new List<int>();
            if (reachedBoundaryVertex == -1) return pathEdges; // 到達不能な場合

            int currV = reachedBoundaryVertex;
            while (currV != startIndex)
            {
                int eIdx = edgeToVertex[currV];
                pathEdges.Add(eIdx);
                currV = parentVertex[currV];
            }

            pathEdges.Reverse(); // スタートからゴールの順にする
            return pathEdges;
        }

        public static double[] TotalLengthAndObj(CutMesh mesh, List<List<int>> path, List<double> edgeLength, bool[] isCut)
        {
            double totalLength = 0;
            HashSet<int> edges = new HashSet<int>();
            for (int i = 0; i < isCut.Length; i++)
            {
                if (!isCut[i]) continue;
                List<int> route = path[i];
                edges.UnionWith(route);
            }
            foreach (int edge in edges) { totalLength += edgeLength[edge]; }

            List<int> boundary = mesh.BoundaryVertIndices();
            CutMesh cutMesh2 = CutMeshWithEdgeIndices(mesh, edges.ToList());
            List<CutMesh> cutMeshes = SplitIntoConnectedComponents(cutMesh2);
            double energy = 0;
            double w1 = 1;
            double w2 = 1;
            double w3 = 1;
            double w4 = 1;
            double w5 = 1;
            double w6 = 1;
            foreach (CutMesh cutMesh in cutMeshes)
            {
                List<List<List<int>>> category = CategolizeCutMesh(cutMesh, false, false, false, boundary, null);

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

                List<List<int>> duplicatedVertIndices = cutMesh.DuplicatedVertIndices;
                int vertCount = cutMesh.Vertices.Count;

                var loopLengthData = CalcLoopLength(cutMesh, category[5], category[9]);
                List<double> aveLengthDone = loopLengthData.DoneAveLength;
                List<double> aveLengthF = loopLengthData.FblocksAveLength;

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
                    double E1 = 2 * Math.PI - 4 * angleSum[vert];
                    double factor = E1 * w1;
                    energy += factor * factor;
                }
                foreach (int vert in C)
                {
                    double E1 = 2 * Math.PI - 2 * angleSum[vert];
                    double factor = E1 * w1;
                    energy += factor * factor;
                }
                foreach (int vert in E)
                {
                    double E1 = 2 * Math.PI - angleSum[vert];
                    double factor = E1 * w1;
                    energy += factor * factor;
                }
                foreach (int[] loop in DtwoMirror2)
                {
                    int n = loop.Length;
                    double angleSumSum = 0;
                    foreach (int vert in loop)
                    {
                        angleSumSum += angleSum[vert];
                    }

                    double E1 = (n + 1) * (n * Math.PI - angleSumSum) / n;
                    double factor = E1 * w6;
                    energy += factor * factor;
                }

                int ii = 0;
                foreach (int[] loop in DoneMirror2)
                {
                    int n = loop.Length;
                    double angleSumSum = 0;
                    foreach (int vert in loop)
                    {
                        angleSumSum += angleSum[vert];
                    }

                    double E1 = (n + 1) * (n * Math.PI - angleSumSum) / n;
                    double factor = E1 * w4;
                    energy += factor * factor;

                    double aveLength = aveLengthDone[ii];
                    List<double> loopLength = DoneLength[ii];
                    List<double> loopSin = DoneSin[ii];
                    List<double> loopCos = DoneCos[ii];
                    double s = CalcDoneSinFactor(loopLength, loopSin);
                    n = loopLength.Count;

                    double value = w5 * (n + 1) * Math.PI / aveLength;
                    E1 = s * value;
                    energy += E1 * E1;
                    ii += 1;
                }

                ii = 0;
                foreach (int[] loop in Fblocks2)
                {
                    int n = loop.Length;
                    double angleSumSum = 0;
                    foreach (int vert in loop)
                    {
                        angleSumSum += angleSum[vert];
                    }

                    double E1 = (n + 4) * ((n + 2) * Math.PI - angleSumSum) / (n + 2);
                    double factor = E1 * w2;
                    energy += factor * factor;



                    double aveLength = aveLengthF[ii];
                    List<double> loopLength = FblocksLength[ii];
                    List<double> loopSin = FblocksSin[ii];
                    List<double> loopCos = FblocksCos[ii];
                    n = loopCos.Count;
                    double value = w3 * (n + 4) * Math.PI / aveLength;

                    //発散防止
                    if (loopLength.Min() < aveLength * 0.001) { continue; }


                    //-----------------------------------------------sinのエネルギー----------------------------------------
                    double s = CalcFSinFactor(loopLength, loopSin);
                    E1 = s * value;
                    energy += E1 * E1;

                    //-----------------------------------------------cosのエネルギー----------------------------------------
                    double c = CalcFCosFactor(loopLength, loopCos);
                    double E2 = c * value;
                    energy += E2 * E2;

                    ii += 1;
                }
            }
            return new double[] {totalLength, energy};
        }
    }
}
