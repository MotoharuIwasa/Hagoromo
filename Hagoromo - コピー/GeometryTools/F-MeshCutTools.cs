using Grasshopper.Kernel;
using Grasshopper.Kernel.Geometry.Delaunay;
using Hagoromo.DataTools;
using Hagoromo.MathTools;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Collections;
using Rhino.Render.ChangeQueue;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;


namespace Hagoromo.GeometryTools
{
    public static class MeshCutTools
    {
        //topoEdgesのindicesに対応するエッジで切込みを入れたメッシュをCutMesh型で作成する
        public static CutMesh CutMeshWithEdgeIndices(Rhino.Geometry.Mesh mesh, List<int> edgeIndices2)
        {
            MeshTopologyEdgeList edgeList = mesh.TopologyEdges;
            List<int> edgeIndices = MeshDataTools.RemoveBoundaryEdges(mesh, edgeIndices2);

            // edgeIndicesのうち、頂点→その頂点を含むedgeのIndicesの辞書
            var vertToEdges = new Dictionary<int, List<int>>();

            foreach (int ei in edgeIndices)
            {
                IndexPair tv = edgeList.GetTopologyVertices(ei);
                tv.I = mesh.TopologyVertices.MeshVertexIndices(tv.I)[0];
                tv.J = mesh.TopologyVertices.MeshVertexIndices(tv.J)[0];

                if (!vertToEdges.TryGetValue(tv.I, out var list0))
                {
                    list0 = new List<int>();
                    vertToEdges[tv.I] = list0;
                }
                list0.Add(ei);

                if (!vertToEdges.TryGetValue(tv.J, out var list1))
                {
                    list1 = new List<int>();
                    vertToEdges[tv.J] = list1;
                }
                list1.Add(ei);
            }

            // edgeIndices に含まれるエッジの端点（頂点インデックス）を列挙
            var edgeVertIndices = vertToEdges.Keys.ToList();

            // それぞれの頂点に対応するエッジ集合を取り出す
            foreach (int v in edgeVertIndices)
            {
                List<int> connectedEdges = vertToEdges[v];
                // ここで v を含む edgeIndices 内のエッジ番号一覧が取れる
            }


            CutMesh originalMesh = new CutMesh(mesh);
            CutMesh newMesh = originalMesh.Clone();
            List<int> internalVertIndices = MeshDataTools.TopoInternalVertIndices(mesh);

            foreach (int v in edgeVertIndices)
            {
                List<int> faces = MeshDataTools.GetOrderedFacesAroundVertex(mesh, mesh.TopologyVertices.TopologyVertexIndex(v));
                // v を含む edgeIndices 内のエッジ番号一覧
                List<int> connectedEdges = vertToEdges[v];

                int count = connectedEdges.Count;
                List<int> vertGroup = new List<int>();
                if ((internalVertIndices.Contains(v)&&count > 1)|| !(internalVertIndices.Contains(v)))
                {
                    int[] cutGuide2 = new int[2 * count];
                    for (int i = 0; i < count; i++)
                    {
                        //aは必ず長さ2
                        int[] a = edgeList.GetConnectedFaces(connectedEdges[i]);
                        cutGuide2[2 * i] = a[0];
                        cutGuide2[2 * i + 1] = a[1];
                    }

                    // faces の要素をキー → 順序番号の Dictionary に変換
                    var orderMap = new Dictionary<int, int>();
                    for (int i = 0; i < faces.Count; i++)
                        orderMap[faces[i]] = i;
                    // cutGuide を orderMap に従ってソート
                    int[] cutGuide = cutGuide2.OrderBy(x => orderMap[x]).ToArray();

                    //vが内部の点である場合
                    if (internalVertIndices.Contains(v))
                    {
                        if (cutGuide[0] == faces[0])
                        {
                            bool sharedEdge = false; // 見つからなかった場合は false
                            for (int i = 0; i < count; i++)
                            {
                                int[] cFaces = edgeList.GetConnectedFaces(connectedEdges[i]);
                                if ((cFaces[0] == cutGuide[0] && cFaces[1] == cutGuide[1]) || (cFaces[0] == cutGuide[1] && cFaces[1] == cutGuide[0]))
                                {
                                    sharedEdge = true;
                                    break;
                                }
                            }

                            //cutGuideの0番目と1番目に切れ目がない場合
                            if (!sharedEdge)
                            {
                                for (int i = 0; i < count - 1; i++)
                                {
                                    newMesh.Vertices.Add(originalMesh.Vertices[v]);
                                    int vertLastIndex = newMesh.Vertices.Count - 1;
                                    vertGroup.Add(vertLastIndex);
                                    int start = faces.IndexOf(cutGuide[2 * i]);
                                    int end = faces.IndexOf(cutGuide[2 * i + 1]);
                                    for (int j = start; j <= end; j++)
                                    {
                                        if (originalMesh.Faces[faces[j], 0] == v) newMesh.Faces[faces[j], 0] = vertLastIndex;
                                        if (originalMesh.Faces[faces[j], 1] == v) newMesh.Faces[faces[j], 1] = vertLastIndex;
                                        if (originalMesh.Faces[faces[j], 2] == v) newMesh.Faces[faces[j], 2] = vertLastIndex;
                                    }
                                }
                            }

                            else
                            {
                                for (int i = 0; i < count - 1; i++)
                                {
                                    newMesh.Vertices.Add(originalMesh.Vertices[v]);
                                    int vertLastIndex = newMesh.Vertices.Count - 1;
                                    vertGroup.Add(vertLastIndex);
                                    int start = faces.IndexOf(cutGuide[2 * i + 1]);
                                    int end = faces.IndexOf(cutGuide[2 * i + 2]);
                                    for (int j = start; j <= end; j++)
                                    {
                                        if (originalMesh.Faces[faces[j], 0] == v) newMesh.Faces[faces[j], 0] = vertLastIndex;
                                        if (originalMesh.Faces[faces[j], 1] == v) newMesh.Faces[faces[j], 1] = vertLastIndex;
                                        if (originalMesh.Faces[faces[j], 2] == v) newMesh.Faces[faces[j], 2] = vertLastIndex;
                                    }
                                }
                            }
                        }

                        else
                        {
                            for (int i = 0; i < count - 1; i++)
                            {
                                newMesh.Vertices.Add(originalMesh.Vertices[v]);
                                int vertLastIndex = newMesh.Vertices.Count - 1;
                                vertGroup.Add(vertLastIndex);
                                int start = faces.IndexOf(cutGuide[2 * i + 1]);
                                int end = faces.IndexOf(cutGuide[2 * i + 2]);
                                for (int j = start; j <= end; j++)
                                {
                                    if (originalMesh.Faces[faces[j], 0] == v) newMesh.Faces[faces[j], 0] = vertLastIndex;
                                    if (originalMesh.Faces[faces[j], 1] == v) newMesh.Faces[faces[j], 1] = vertLastIndex;
                                    if (originalMesh.Faces[faces[j], 2] == v) newMesh.Faces[faces[j], 2] = vertLastIndex;
                                }
                            }
                        }
                    }

                    //vが境界上の点である場合
                    else
                    {
                        for (int i = 0; i < count; i++)
                        {
                            newMesh.Vertices.Add(originalMesh.Vertices[v]);
                            int vertLastIndex = newMesh.Vertices.Count - 1;
                            vertGroup.Add(vertLastIndex);
                            int start = 0;
                            //int end = faces.Count - 1;
                            int end = faces.IndexOf(cutGuide[2 * i]);
                            if (i != 0)
                            {
                                start = faces.IndexOf(cutGuide[2 * i - 1]);
                            }
                            /*
                            if (i != count - 1)
                            {
                                end = faces.IndexOf(cutGuide[2 * i]);
                            }
                            */

                            for (int j = start; j <= end; j++)
                            {
                                if (originalMesh.Faces[faces[j], 0] == v) newMesh.Faces[faces[j], 0] = vertLastIndex;
                                if (originalMesh.Faces[faces[j], 1] == v) newMesh.Faces[faces[j], 1] = vertLastIndex;
                                if (originalMesh.Faces[faces[j], 2] == v) newMesh.Faces[faces[j], 2] = vertLastIndex;
                            }
                        }
                    }
                }
                newMesh.DuplicatedVertIndices[v].AddRange(vertGroup);
            }
            
            int oldMaxIndex = mesh.Vertices.Count - 1;
            //あとは新しいfaceとVerticesをもとにedgeを再構築する
            for (int ei =0; ei < mesh.TopologyEdges.Count; ei++)
            {//oriMeshでもnewMeshでもfacは同じ。eiをもつfaceindexを見る。そのfaceのeiの部分の2点を見る
                //(old,old)がなければ消す。newが入っていたらそのエッジを追加
                IndexPair tv = edgeList.GetTopologyVertices(ei);
                tv.I = mesh.TopologyVertices.MeshVertexIndices(tv.I)[0];
                tv.J = mesh.TopologyVertices.MeshVertexIndices(tv.J)[0];
                
                List<int> foundList1 = newMesh.DuplicatedVertIndices.FirstOrDefault(list => list.Contains(tv.I));
                List<int> foundList2 = newMesh.DuplicatedVertIndices.FirstOrDefault(list => list.Contains(tv.J));
                int[] cFaces = edgeList.GetConnectedFaces(ei);
                bool t = false;
                int valueInListOld1 = -1;
                int valueInListOld2 = -1;
                for (int j = 0; j < cFaces.Length; j++)
                {
                    int A = newMesh.Faces[cFaces[j], 0];
                    int B = newMesh.Faces[cFaces[j], 1];
                    int C = newMesh.Faces[cFaces[j], 2];
                    int[] values = { A, B, C };
                    int valueInList1 = values.First(v => foundList1.Contains(v));
                    int valueInList2 = values.First(v => foundList2.Contains(v));
                    if (!((valueInList1 == valueInListOld1 && valueInList2 == valueInListOld2)|| (valueInList2 == valueInListOld1 && valueInList1 == valueInListOld2)))
                    {
                        if (valueInList1 <= oldMaxIndex)
                        {
                            if (valueInList2 <= oldMaxIndex)
                            {
                                t = true;
                            }
                            else
                            {
                                newMesh.Edges.Add(new int[] { valueInList1, valueInList2 });
                            }
                        }
                        else
                        {
                            newMesh.Edges.Add(new int[] { valueInList1, valueInList2 });
                        }
                        valueInListOld1 = valueInList1;
                        valueInListOld2 = valueInList2;
                    }
                }
                
                if (!t)
                {
                    newMesh.Edges.RemoveAll(e =>(e[0] == tv.I && e[1] == tv.J) || (e[0] == tv.J && e[1] == tv.I));
                }
                
            }
            newMesh.ReloadVertexToFacesCache();
            newMesh.ReloadEdgeToFacesCache();
            newMesh.ReloadVertexToEdgesCache();
            return newMesh;
        }

        //CutMeshに追加のCutLineを加える
        public static CutMesh CutMeshWithEdgeIndices(CutMesh cutMesh, List<int> edgeIndices2)
        {
            /*
            //念のため辞書を更新しておく
            cutMesh.ReloadEdgeToFacesCache();
            cutMesh.ReloadVertexToEdgesCache();
            cutMesh.ReloadVertexToFacesCache();
            */
            List<int> boundaryVertIndices = cutMesh.BoundaryVertIndices();
            List<int> fullSet = Enumerable.Range(0, cutMesh.Vertices.Count).ToList();

            List<int> boundaryEdgeIndices = cutMesh.BoundaryEdgeIndices();
            List<int> edgeIndices = edgeIndices2.Except(boundaryEdgeIndices).ToList();

            // edgeIndicesのうち、頂点→その頂点を含むedgeのIndicesの辞書
            var vertToEdges = new Dictionary<int, List<int>>();

            foreach (int ei in edgeIndices)
            {
                int[] edgeVertices = cutMesh.Edges[ei];
                IndexPair tv = new IndexPair();
                tv.I = edgeVertices[0];
                tv.J = edgeVertices[1];

                if (!vertToEdges.TryGetValue(tv.I, out var list0))
                {
                    list0 = new List<int>();
                    vertToEdges[tv.I] = list0;
                }
                list0.Add(ei);

                if (!vertToEdges.TryGetValue(tv.J, out var list1))
                {
                    list1 = new List<int>();
                    vertToEdges[tv.J] = list1;
                }
                list1.Add(ei);
            }

            // edgeIndices に含まれるエッジの端点（頂点インデックス）を列挙
            var edgeVertIndices = vertToEdges.Keys.ToList();

            /*
            // それぞれの頂点に対応するエッジ集合を取り出す
            foreach (int v in edgeVertIndices)
            {
                List<int> connectedEdges = vertToEdges[v];
                // ここで v を含む edgeIndices 内のエッジ番号一覧が取れる
            }
            */

            //CutMesh originalMesh = cutMesh.Clone();
            CutMesh newMesh = cutMesh.Clone();
            List<int> internalVertIndices = fullSet.Except(boundaryVertIndices).ToList();

            foreach (int v in edgeVertIndices)
            {
                List<int> faces = cutMesh.GetOrderedFacesForVertex(v);
                // v を含む edgeIndices 内のエッジ番号一覧
                List<int> connectedEdges = vertToEdges[v];

                int count = connectedEdges.Count;
                List<int> vertGroup = new List<int>();
                if ((internalVertIndices.Contains(v) && count > 1) || !(internalVertIndices.Contains(v)))
                {
                    int[] cutGuide2 = new int[2 * count];
                    for (int i = 0; i < count; i++)
                    {
                        //aは必ず長さ2
                        List<int> a = cutMesh.GetFacesForEdge(connectedEdges[i]);
                        cutGuide2[2 * i] = a[0];
                        cutGuide2[2 * i + 1] = a[1];
                    }

                    // faces の要素をキー → 順序番号の Dictionary に変換
                    var orderMap = new Dictionary<int, int>();
                    for (int i = 0; i < faces.Count; i++)
                        orderMap[faces[i]] = i;
                    // cutGuide を orderMap に従ってソート
                    int[] cutGuide = cutGuide2.OrderBy(x => orderMap[x]).ToArray();

                    //vが内部の点である場合
                    if (internalVertIndices.Contains(v))
                    {
                        if (cutGuide[0] == faces[0])
                        {
                            bool sharedEdge = false; // 見つからなかった場合は false
                            for (int i = 0; i < count; i++)
                            {
                                List<int> cFaces = cutMesh.GetFacesForEdge(connectedEdges[i]);
                                if ((cFaces[0] == cutGuide[0] && cFaces[1] == cutGuide[1]) || (cFaces[0] == cutGuide[1] && cFaces[1] == cutGuide[0]))
                                {
                                    sharedEdge = true;
                                    break;
                                }
                            }

                            //cutGuideの0番目と1番目に切れ目がない場合
                            if (!sharedEdge)
                            {
                                for (int i = 0; i < count - 1; i++)
                                {
                                    newMesh.Vertices.Add(cutMesh.Vertices[v]);
                                    int vertLastIndex = newMesh.Vertices.Count - 1;
                                    vertGroup.Add(vertLastIndex);
                                    int start = faces.IndexOf(cutGuide[2 * i]);
                                    int end = faces.IndexOf(cutGuide[2 * i + 1]);
                                    for (int j = start; j <= end; j++)
                                    {
                                        if (cutMesh.Faces[faces[j], 0] == v) newMesh.Faces[faces[j], 0] = vertLastIndex;
                                        if (cutMesh.Faces[faces[j], 1] == v) newMesh.Faces[faces[j], 1] = vertLastIndex;
                                        if (cutMesh.Faces[faces[j], 2] == v) newMesh.Faces[faces[j], 2] = vertLastIndex;
                                    }
                                }
                            }

                            else
                            {
                                for (int i = 0; i < count - 1; i++)
                                {
                                    newMesh.Vertices.Add(cutMesh.Vertices[v]);
                                    int vertLastIndex = newMesh.Vertices.Count - 1;
                                    vertGroup.Add(vertLastIndex);
                                    int start = faces.IndexOf(cutGuide[2 * i + 1]);
                                    int end = faces.IndexOf(cutGuide[2 * i + 2]);
                                    for (int j = start; j <= end; j++)
                                    {
                                        if (cutMesh.Faces[faces[j], 0] == v) newMesh.Faces[faces[j], 0] = vertLastIndex;
                                        if (cutMesh.Faces[faces[j], 1] == v) newMesh.Faces[faces[j], 1] = vertLastIndex;
                                        if (cutMesh.Faces[faces[j], 2] == v) newMesh.Faces[faces[j], 2] = vertLastIndex;
                                    }
                                }
                            }
                        }

                        else
                        {
                            for (int i = 0; i < count - 1; i++)
                            {
                                newMesh.Vertices.Add(cutMesh.Vertices[v]);
                                int vertLastIndex = newMesh.Vertices.Count - 1;
                                vertGroup.Add(vertLastIndex);
                                int start = faces.IndexOf(cutGuide[2 * i + 1]);
                                int end = faces.IndexOf(cutGuide[2 * i + 2]);
                                for (int j = start; j <= end; j++)
                                {
                                    if (cutMesh.Faces[faces[j], 0] == v) newMesh.Faces[faces[j], 0] = vertLastIndex;
                                    if (cutMesh.Faces[faces[j], 1] == v) newMesh.Faces[faces[j], 1] = vertLastIndex;
                                    if (cutMesh.Faces[faces[j], 2] == v) newMesh.Faces[faces[j], 2] = vertLastIndex;
                                }
                            }
                        }
                    }

                    //vが境界上の点である場合
                    else
                    {
                        for (int i = 0; i < count; i++)
                        {
                            newMesh.Vertices.Add(cutMesh.Vertices[v]);
                            int vertLastIndex = newMesh.Vertices.Count - 1;
                            vertGroup.Add(vertLastIndex);
                            int start = 0;
                            //int end = faces.Count - 1;
                            int end = faces.IndexOf(cutGuide[2 * i]);
                            if (i != 0)
                            {
                                start = faces.IndexOf(cutGuide[2 * i - 1]);
                            }

                            for (int j = start; j <= end; j++)
                            {
                                if (cutMesh.Faces[faces[j], 0] == v) newMesh.Faces[faces[j], 0] = vertLastIndex;
                                if (cutMesh.Faces[faces[j], 1] == v) newMesh.Faces[faces[j], 1] = vertLastIndex;
                                if (cutMesh.Faces[faces[j], 2] == v) newMesh.Faces[faces[j], 2] = vertLastIndex;
                            }
                        }
                    }
                }

                foreach (List<int> group in newMesh.DuplicatedVertIndices)
                {
                    if (group.Contains(v))
                    {
                        group.AddRange(vertGroup);
                        break;
                    }
                }

            }

            int oldMaxIndex = cutMesh.Vertices.Count - 1;
            //あとは新しいfaceとVerticesをもとにedgeを再構築する
            for (int ei = 0; ei < cutMesh.Edges.Count; ei++)
            {//oriMeshでもnewMeshでもfacは同じ。eiをもつfaceindexを見る。そのfaceのeiの部分の2点を見る
                //(old,old)がなければ消す。newが入っていたらそのエッジを追加
                int[] edgeVertices = cutMesh.Edges[ei];
                IndexPair tv = new IndexPair();
                tv.I = edgeVertices[0];
                tv.J = edgeVertices[1];

                List<int> foundList1 = newMesh.DuplicatedVertIndices.FirstOrDefault(list => list.Contains(tv.I));
                List<int> foundList2 = newMesh.DuplicatedVertIndices.FirstOrDefault(list => list.Contains(tv.J));
                List<int> cFaces = cutMesh.GetFacesForEdge(ei);
                bool t = false;
                int valueInListOld1 = -1;
                int valueInListOld2 = -1;
                for (int j = 0; j < cFaces.Count; j++)
                {
                    int A = newMesh.Faces[cFaces[j], 0];
                    int B = newMesh.Faces[cFaces[j], 1];
                    int C = newMesh.Faces[cFaces[j], 2];
                    int[] values = { A, B, C };
                    int valueInList1 = values.First(v => foundList1.Contains(v));
                    int valueInList2 = values.First(v => foundList2.Contains(v));
                    if (!((valueInList1 == valueInListOld1 && valueInList2 == valueInListOld2) || (valueInList2 == valueInListOld1 && valueInList1 == valueInListOld2)))
                    {
                        if (valueInList1 <= oldMaxIndex)
                        {
                            if (valueInList2 <= oldMaxIndex)
                            {
                                t = true;
                            }
                            else
                            {
                                newMesh.Edges.Add(new int[] { valueInList1, valueInList2 });
                            }
                        }
                        else
                        {
                            newMesh.Edges.Add(new int[] { valueInList1, valueInList2 });
                        }
                        valueInListOld1 = valueInList1;
                        valueInListOld2 = valueInList2;
                    }
                }

                if (!t)
                {
                    newMesh.Edges.RemoveAll(e => (e[0] == tv.I && e[1] == tv.J) || (e[0] == tv.J && e[1] == tv.I));
                }

            }
            newMesh.ReloadVertexToFacesCache();
            newMesh.ReloadEdgeToFacesCache();
            newMesh.ReloadVertexToEdgesCache();
            return newMesh;
        }

        //完全に分割されているCutMeshを検出して、複数のCutMeshに直す
        public static List<CutMesh> SplitIntoConnectedComponents(CutMesh mesh)
        {
            int faceCount = mesh.Faces.GetLength(0);
            int vertCount = mesh.Vertices.Count;
            int edgeCount = mesh.Edges.Count;

            // -----------------------------
            // 1. Face adjacency を構築
            // -----------------------------
            // 各 face に隣接する face のリストを作る
            List<int>[] faceAdj = new List<int>[faceCount];
            for (int i = 0; i < faceCount; i++) faceAdj[i] = new List<int>(6);
            Dictionary<int, List<int>> edgeToFacesCache = mesh.GetEdgeToFacesCache();
            foreach (var kv in edgeToFacesCache)
            {
                var faces = kv.Value;
                if (faces.Count == 2) // 両側にfaceがある場合のみ隣接
                {
                    int f0 = faces[0];
                    int f1 = faces[1];
                    faceAdj[f0].Add(f1);
                    faceAdj[f1].Add(f0);
                }
            }

            // -----------------------------
            // 2. DFSで連結成分分解
            // -----------------------------
            bool[] visited = new bool[faceCount];
            List<List<int>> components = new List<List<int>>();

            for (int fi = 0; fi < faceCount; fi++)
            {
                if (visited[fi]) continue;

                List<int> comp = new List<int>();
                Stack<int> stack = new Stack<int>();
                stack.Push(fi);

                while (stack.Count > 0)
                {
                    int f = stack.Pop();
                    if (visited[f]) continue;
                    visited[f] = true;
                    comp.Add(f);

                    var nbs = faceAdj[f];
                    for (int i = 0; i < nbs.Count; i++)
                    {
                        int nf = nbs[i];
                        if (!visited[nf]) stack.Push(nf);
                    }
                }

                components.Add(comp);
            }

            // -----------------------------
            // 3. 各成分ごとに CutMesh を再構築
            // -----------------------------
            List<CutMesh> result = new List<CutMesh>(components.Count);

            foreach (var compFaces in components)
            {
                // old → new index の対応表
                Dictionary<int, int> vmap = new Dictionary<int, int>();
                List<Point3d> newVerts = new List<Point3d>();
                int[,] newFaces = new int[compFaces.Count, 3];

                // 頂点抽出 + faceインデックス変換
                for (int i = 0; i < compFaces.Count; i++)
                {
                    int f = compFaces[i];
                    for (int j = 0; j < 3; j++)
                    {
                        int oldVi = mesh.Faces[f, j];
                        if (!vmap.TryGetValue(oldVi, out int newVi))
                        {
                            newVi = newVerts.Count;
                            vmap[oldVi] = newVi;
                            newVerts.Add(mesh.Vertices[oldVi]);
                        }
                        newFaces[i, j] = newVi;
                    }
                }

                // エッジ再構築（HashSetで重複排除）
                HashSet<(int, int)> edgeSet = new HashSet<(int, int)>();
                List<int[]> newEdges = new List<int[]>();

                for (int i = 0; i < newFaces.GetLength(0); i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        int v0 = newFaces[i, j];
                        int v1 = newFaces[i, (j + 1) % 3];
                        var e = v0 < v1 ? (v0, v1) : (v1, v0);
                        if (edgeSet.Add(e))
                            newEdges.Add(new int[] { e.Item1, e.Item2 });
                    }
                }

                // DuplicatedVertIndices （ここでは trivial に1頂点=1グループ）
                List<List<int>> dVerts = new List<List<int>>(newVerts.Count);
                for (int i = 0; i < newVerts.Count; i++)
                    dVerts.Add(new List<int> { i });

                result.Add(new CutMesh(newVerts, newFaces, newEdges, dVerts));
            }

            return result;
        }



        //faceの閉ループを持つメッシュに切込みを入れて枝分かれありの一本の木になるようにしたときのconnectedFacesを出力
        public static int[][] MeshToOpenTree(Rhino.Geometry.Mesh mesh)
        {
            int faceCount = mesh.Faces.Count;
            int[][] connectedFaces = MeshDataTools.ConnectedFaces(mesh);

            int[][] newConnectedFaces = new int[faceCount][];
            List<int> recordedFaces = new List<int>();
            List<int> currentFaces = new List<int>() { 0 };
            List<List<int>> nextFaces = new List<List<int>>();
            List<List<int>> nextFaces2 = new List<List<int>>();
            newConnectedFaces[currentFaces[0]] = connectedFaces[currentFaces[0]];
            recordedFaces.Add(0);
            recordedFaces.AddRange(newConnectedFaces[currentFaces[0]]);
            nextFaces.Add(new List<int>());
            nextFaces[0].AddRange(newConnectedFaces[currentFaces[0]]);
            int totalCount = 1;

            while (totalCount > 0)
            {
                for (int i = 0; i < currentFaces.Count; i++)
                {
                    if (nextFaces[i].Count > 0)
                    {
                        for (int j = 0; j < nextFaces[i].Count; j++)
                        {
                            int[] missingFaces = DataTools.DataTools.GetMissingElements(recordedFaces, connectedFaces[nextFaces[i][j]]);
                            List<int> a = missingFaces.ToList();
                            nextFaces2.Add(a);
                            if (missingFaces.Length > 0)
                            {
                                newConnectedFaces[nextFaces[i][j]] = new int[missingFaces.Length + 1];
                                newConnectedFaces[nextFaces[i][j]][0] = currentFaces[i];
                                for (int k = 0; k < missingFaces.Length; k++)
                                {
                                    newConnectedFaces[nextFaces[i][j]][k + 1] = missingFaces[k];
                                }
                                recordedFaces.AddRange(missingFaces);
                            }
                        }
                    }
                }
                currentFaces = nextFaces.SelectMany(x => x).ToList();
                nextFaces = nextFaces2;
                nextFaces2 = new List<List<int>>();
                totalCount = nextFaces.Sum(inner => inner.Count);
            }

            //上記でほとんどうまくいっているが、i番目のfaceがj番目のfaceとつながっているのに
            //j番目のfaceがi番目のfaceとつながっていないという感じでnewConnectedFacesが作成されているのでその修正
            int n = newConnectedFaces.Length;
            // List に変換して要素追加を容易にする
            List<int>[] lists = new List<int>[n];
            for (int i = 0; i < n; i++)
            {
                lists[i] = newConnectedFaces[i]?.ToList() ?? new List<int>();
            }

            // 双方向に接続を追加
            for (int i = 0; i < n; i++)
            {
                foreach (int j in lists[i].ToList()) // 元リストをコピーして foreach
                {
                    if (!lists[j].Contains(i))
                    {
                        lists[j].Add(i);
                    }
                }
            }

            // List を配列に戻す
            int[][] result = new int[n][];
            for (int i = 0; i < n; i++)
            {
                result[i] = lists[i].ToArray();
            }

            return result;
        }

        //newConnectedFacesからmeshで切込みを入れるedgeのmesh.TopologyEdgesにおけるindexを返す
        public static List<int> CutEdgeIndices(Rhino.Geometry.Mesh mesh, int[][] newConnectedFaces)
        {
            int edgeCount = mesh.TopologyEdges.Count;
            List<int> connectedEdge2 = new List<int>();

            for (int i = 0; i < newConnectedFaces.Length; i++)
            {
                if (newConnectedFaces[i].Length == 0) continue;
                for (int j = 0; j < newConnectedFaces[i].Length; j++)
                {
                    connectedEdge2.Add(MeshDataTools.SharedEdgeIndex(mesh, newConnectedFaces[i][j], i));
                }
            }
            HashSet<int> connectedEdge = new HashSet<int>(connectedEdge2);

            List<int> cutEdge = new List<int>();
            for (int i = 0; i < edgeCount; i++)
            {
                if (!connectedEdge.Contains(i))
                {
                    cutEdge.Add(i);
                }
            }
            return cutEdge;
        }

        //newConnectedFacesをもとにmeshのfaceのつながりを表すグラフを作成する
        public static List<Curve> MeshTreeCentersToCurves(Rhino.Geometry.Mesh mesh, int[][] newConnectedFaces)
        {
            int faceCount = mesh.Faces.Count;
            Point3d[] faceCenters = new Point3d[faceCount];
            // フェイス中心を計算
            for (int i = 0; i < faceCount; i++)
            {
                faceCenters[i] = MeshCalcTools.FaceCentroid(mesh, i);
            }

            var curves = new List<Curve>();

            for (int i = 0; i < faceCount; i++)
            {
                if (newConnectedFaces[i] == null) continue;

                foreach (int j in newConnectedFaces[i])
                {
                    if (j < 0 || j >= faceCount) continue;
                    
                    // 重複描画防止 (i<j のときだけ描画)
                    if (i > j) continue;

                    curves.Add(new Line(faceCenters[i], faceCenters[j]).ToNurbsCurve());
                }
            }

            return curves;
        }
    }

}
