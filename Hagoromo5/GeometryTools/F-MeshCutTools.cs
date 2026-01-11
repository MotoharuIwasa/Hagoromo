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
            List<List<int>> originDupVerts = mesh.DuplicatedVertIndices;

            List<int> duplicatedIndex = mesh.DuplicatedVertIndices.Where(l => l.Count >= 2).Select(l => l[0]).ToList();

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

                List<List<int>> dVerts = new List<List<int>>();
                {
                    foreach (List<int> vertGroup in originDupVerts)
                    {
                        List<int> newGroup = new List<int>();
                        foreach (int vert in vertGroup)
                        {
                            if (vmap.TryGetValue(vert, out int newVert))
                            {
                                newGroup.Add(newVert);
                            }
                        }
                        if (newGroup.Count != 0) { dVerts.Add(newGroup); }
                    }
                }
                CutMesh newOne = new CutMesh(newVerts, newFaces, newEdges, dVerts);
                newOne = newOne.Sort();
                newOne.DuplicatedVertIndices = newOne.DuplicatedVertIndices.Select(list => list.OrderBy(x => x).ToList()).OrderBy(list => list[0]).ToList();
                /*
                newOne.ReloadEdgeToFacesCache();
                newOne.ReloadVertexToEdgesCache();
                newOne.ReloadVertexToFacesCache();
                */
                /*
                // DuplicatedVertIndices （ここでは trivial に1頂点=1グループ）
                List<List<int>> dVerts = new List<List<int>>(newVerts.Count);
                for (int i = 0; i < newVerts.Count; i++)
                    dVerts.Add(new List<int> { i });
                */

                result.Add(newOne);
            }

            return result;
        }

        public class MeshSplitResult
        {
            public CutMesh Mesh { get; set; }
            // Mesh.Vertices[i] が 元Mesh.Vertices[VertexMap[i]] に対応する
            public List<int> VertexMap { get; set; }
        }
        public static List<MeshSplitResult> SplitIntoConnectedComponents2(CutMesh mesh)
        {
            int faceCount = mesh.Faces.GetLength(0);
            List<List<int>> originDupVerts = mesh.DuplicatedVertIndices;

            // 1. Face adjacency を構築
            List<int>[] faceAdj = new List<int>[faceCount];
            for (int i = 0; i < faceCount; i++) faceAdj[i] = new List<int>(6);
            Dictionary<int, List<int>> edgeToFacesCache = mesh.BuildEdgeFacesCache();
            foreach (var kv in edgeToFacesCache)
            {
                var faces = kv.Value;
                if (faces.Count == 2)
                {
                    int f0 = faces[0];
                    int f1 = faces[1];
                    faceAdj[f0].Add(f1);
                    faceAdj[f1].Add(f0);
                }
            }

            // 2. DFSで連結成分分解
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
                    foreach (int nf in faceAdj[f])
                    {
                        if (!visited[nf]) stack.Push(nf);
                    }
                }
                components.Add(comp);
            }

            // 3. 各成分ごとに再構築
            List<MeshSplitResult> results = new List<MeshSplitResult>();

            foreach (var compFaces in components)
            {
                Dictionary<int, int> vmap = new Dictionary<int, int>();
                List<Point3d> newVerts = new List<Point3d>();
                List<int> intermediateMap = new List<int>(); // [分割後LocalIndex] = 元のGlobalIndex
                int[,] newFaces = new int[compFaces.Count, 3];

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
                            intermediateMap.Add(oldVi);
                        }
                        newFaces[i, j] = newVi;
                    }
                }

                // エッジ再構築
                HashSet<(int, int)> edgeSet = new HashSet<(int, int)>();
                List<int[]> newEdges = new List<int[]>();
                for (int i = 0; i < newFaces.GetLength(0); i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        int v0 = newFaces[i, j];
                        int v1 = newFaces[i, (j + 1) % 3];
                        var e = v0 < v1 ? (v0, v1) : (v1, v0);
                        if (edgeSet.Add(e)) newEdges.Add(new int[] { e.Item1, e.Item2 });
                    }
                }

                // 重複頂点リストの更新
                List<List<int>> dVerts = new List<List<int>>();
                foreach (List<int> vertGroup in originDupVerts)
                {
                    List<int> newGroup = vertGroup.Where(v => vmap.ContainsKey(v)).Select(v => vmap[v]).ToList();
                    if (newGroup.Count != 0) dVerts.Add(newGroup);
                }

                // ソート前のサブメッシュを作成
                CutMesh subMesh = new CutMesh(newVerts, newFaces, newEdges, dVerts);

                // --- ソート順序(relation)の計算 ---
                // 提供されたSort()の内部ロジックを用いて並び替え順をシミュレートします
                List<int> relation = GetSortRelation(subMesh);

                // 実際のソート実行
                //CutMesh sortedMesh = subMesh.Sort();

                // マップの合成: 
                // 最終インデックス i -> ソート前インデックス relation[i] -> 元のグローバルインデックス intermediateMap[relation[i]]
                var sorted = subMesh.Sort2WithRelation();
                CutMesh sortedMesh = sorted.Mesh;
                List<int> sortMap = sorted.VertexMap; // これがソートによる対応表

                List<int> finalVertexMap = new List<int>();

                // インデックスの追跡ロジック:
                // ソート後のインデックス i 
                //   -> subMeshでのインデックス sortMap[i] 
                //   -> 元メッシュでのインデックス intermediateMap[sortMap[i]]
                foreach (int subIdx in sortMap)
                {
                    finalVertexMap.Add(intermediateMap[subIdx]);
                }

                results.Add(new MeshSplitResult
                {
                    Mesh = sortedMesh,
                    VertexMap = finalVertexMap
                });
            }

            return results;
        }

        //Sort()に基づき、並び替え後のインデックス順序(relation)を返す
        private static List<int> GetSortRelation(CutMesh mesh)
        {
            // キャッシュを最新にする
            mesh.ReloadVertexToEdgesCache();
            mesh.ReloadEdgeToFacesCache();
            mesh.ReloadVertexToFacesCache();

            List<int> boundaryVertIndices = mesh.BoundaryVertIndices();
            List<int> boundaryEdges = mesh.BoundaryEdgeIndices();
            int totalVertCount = mesh.Vertices.Count;

            if (boundaryVertIndices.Count == 0)
                return Enumerable.Range(0, totalVertCount).ToList();

            bool[] isSorted = new bool[totalVertCount];
            List<int> sortBoundaryIndices = new List<int>(boundaryVertIndices.Count);

            var vertToBoundaryEdges = new Dictionary<int, List<int>>();
            foreach (int ei in boundaryEdges)
            {
                foreach (int v in mesh.Edges[ei])
                {
                    if (!vertToBoundaryEdges.ContainsKey(v)) vertToBoundaryEdges[v] = new List<int>();
                    vertToBoundaryEdges[v].Add(ei);
                }
            }

            int currentVertex = boundaryVertIndices[0];
            sortBoundaryIndices.Add(currentVertex);
            isSorted[currentVertex] = true;

            for (int i = 0; i < boundaryVertIndices.Count - 1; i++)
            {
                int nextVertex = -1;
                if (vertToBoundaryEdges.ContainsKey(currentVertex))
                {
                    foreach (int ei in vertToBoundaryEdges[currentVertex])
                    {
                        int v0 = mesh.Edges[ei][0];
                        int v1 = mesh.Edges[ei][1];
                        int candidate = (v0 == currentVertex) ? v1 : v0;
                        if (!isSorted[candidate])
                        {
                            nextVertex = candidate;
                            break;
                        }
                    }
                }

                if (nextVertex == -1)
                {
                    foreach (int bv in boundaryVertIndices)
                    {
                        if (!isSorted[bv]) { nextVertex = bv; break; }
                    }
                }

                if (nextVertex != -1)
                {
                    sortBoundaryIndices.Add(nextVertex);
                    isSorted[nextVertex] = true;
                    currentVertex = nextVertex;
                }
            }

            List<int> internalVertIndices = new List<int>();
            for (int i = 0; i < totalVertCount; i++)
            {
                if (!isSorted[i]) internalVertIndices.Add(i);
            }

            // これが提供された Sort() 内の relation と同一のものになります
            return sortBoundaryIndices.Concat(internalVertIndices).ToList();
        }



        public static CutMesh SimplifyLoopsByMedianPoint(CutMesh rawCutMesh, int n)
        {
            // 0. ベースとなるメッシュ（溶接状態）の情報をエッジ・頂点インデックスの対応のために整理
            // rawCutMesh.DuplicatedVertIndices のインデックスを「ベース頂点ID」として扱う
            int baseVertCount = rawCutMesh.DuplicatedVertIndices.Count;
            int[] localToBase = new int[rawCutMesh.Vertices.Count];
            for (int i = 0; i < baseVertCount; i++)
                foreach (int localV in rawCutMesh.DuplicatedVertIndices[i]) localToBase[localV] = i;

            // 1. パッチ（連結成分）の抽出
            var patches = GetFacePatches(rawCutMesh);

            // 現在カットされているエッジのベースIDペアを保持
            HashSet<(int, int)> currentBaseCuts = GetCurrentBaseCutEdges(rawCutMesh, localToBase);
            bool isModified = false;

            foreach (var patchFaces in patches)
            {
                if (patchFaces.Count >= n) continue;
                isModified = true;

                // 2. ループ境界エッジと外部接続点（Split Vertices）の特定
                var boundary = GetPatchBoundary(rawCutMesh, patchFaces);
                List<int> splitVertsLocal = IdentifySplitVertices(rawCutMesh, boundary.Vertices);
                List<int> splitVertsBase = splitVertsLocal.Select(v => localToBase[v]).Distinct().ToList();

                // 3. ループの境界切込みを「溶接」対象として削除
                foreach (int eIdx in boundary.Edges)
                {
                    int bV1 = localToBase[rawCutMesh.Edges[eIdx][0]];
                    int bV2 = localToBase[rawCutMesh.Edges[eIdx][1]];
                    var key = bV1 < bV2 ? (bV1, bV2) : (bV2, bV1);
                    currentBaseCuts.Remove(key);
                }

                if (splitVertsBase.Count == 0) continue; // 接続がなければ溶接のみで終了

                // 4. パッチ内グラフでの中央値探索
                // パッチに含まれる全ベース頂点ID
                HashSet<int> patchBaseVerts = new HashSet<int>();
                foreach (int fIdx in patchFaces)
                    for (int j = 0; j < 3; j++) patchBaseVerts.Add(localToBase[rawCutMesh.Faces[fIdx, j]]);

                // パッチ内のみを通る隣接リスト
                var patchAdj = BuildPatchBaseAdj(rawCutMesh, patchFaces, localToBase);

                // 中央値 (Median Point) の選定: 1~k への距離の和が最小の点
                int medianVertBase = FindMedianBaseVertex(patchBaseVerts.ToList(), splitVertsBase, patchAdj, rawCutMesh);

                // 5. 中央値から各接点 1~k への最短経路を計算（重複はHashSetで自動処理）
                foreach (int targetBaseV in splitVertsBase)
                {
                    var pathEdges = FindShortestPathBaseEdges(medianVertBase, targetBaseV, patchAdj);
                    foreach (var edgeKey in pathEdges) currentBaseCuts.Add(edgeKey);
                }
            }

            if (!isModified) return rawCutMesh;

            // 6. 整理された currentBaseCuts を使ってメッシュを再構築
            return ReconstructFromBaseCuts(rawCutMesh, currentBaseCuts, localToBase);
        }

        // --- 内部ロジック ---

        private static int FindMedianBaseVertex(List<int> candidates, List<int> roots, Dictionary<int, List<(int to, double len)>> adj, CutMesh mesh)
        {
            int bestV = candidates[0];
            double minTotalDist = double.MaxValue;

            foreach (int v in candidates)
            {
                double currentSum = 0;
                // 各 roots (1~k) への距離をダイクストラ等で計算
                var dists = Dijkstra(v, adj);
                foreach (int r in roots)
                {
                    if (dists.ContainsKey(r)) currentSum += dists[r];
                    else currentSum += 1e6; // 到達不能パスへのペナルティ
                }

                if (currentSum < minTotalDist) { minTotalDist = currentSum; bestV = v; }
            }
            return bestV;
        }

        private static Dictionary<int, double> Dijkstra(int start, Dictionary<int, List<(int to, double len)>> adj)
        {
            var dists = new Dictionary<int, double> { { start, 0 } };
            var pq = new SortedSet<(double d, int v)>(Comparer<(double d, int v)>.Create((a, b) => a.d == b.d ? a.v.CompareTo(b.v) : a.d.CompareTo(b.d)));
            pq.Add((0, start));

            while (pq.Count > 0)
            {
                var curr = pq.Min; pq.Remove(curr);
                if (!adj.ContainsKey(curr.v)) continue;
                foreach (var edge in adj[curr.v])
                {
                    double newDist = curr.d + edge.len;
                    if (!dists.ContainsKey(edge.to) || newDist < dists[edge.to])
                    {
                        pq.Remove((dists.ContainsKey(edge.to) ? dists[edge.to] : -1, edge.to));
                        dists[edge.to] = newDist;
                        pq.Add((newDist, edge.to));
                    }
                }
            }
            return dists;
        }

        private static List<(int, int)> FindShortestPathBaseEdges(int start, int end, Dictionary<int, List<(int to, double len)>> adj)
        {
            if (start == end) return new List<(int, int)>();
            var parent = new Dictionary<int, int>();
            var q = new Queue<int>();
            q.Enqueue(start);
            parent[start] = -1;

            while (q.Count > 0)
            {
                int c = q.Dequeue();
                if (c == end) break;
                if (!adj.ContainsKey(c)) continue;
                foreach (var e in adj[c])
                    if (!parent.ContainsKey(e.to)) { parent[e.to] = c; q.Enqueue(e.to); }
            }

            var edges = new List<(int, int)>();
            int temp = end;
            while (parent.ContainsKey(temp) && parent[temp] != -1)
            {
                int v1 = temp, v2 = parent[temp];
                edges.Add(v1 < v2 ? (v1, v2) : (v2, v1));
                temp = parent[temp];
            }
            return edges;
        }

        private static Dictionary<int, List<(int to, double len)>> BuildPatchBaseAdj(CutMesh mesh, List<int> faces, int[] localToBase)
        {
            var adj = new Dictionary<int, List<(int to, double len)>>();
            void Add(int u, int v, double l)
            {
                if (!adj.ContainsKey(u)) adj[u] = new List<(int, double)>();
                adj[u].Add((v, l));
            }
            foreach (int fIdx in faces)
            {
                for (int j = 0; j < 3; j++)
                {
                    int v1 = mesh.Faces[fIdx, j], v2 = mesh.Faces[fIdx, (j + 1) % 3];
                    int b1 = localToBase[v1], b2 = localToBase[v2];
                    double len = mesh.Vertices[v1].DistanceTo(mesh.Vertices[v2]);
                    Add(b1, b2, len); Add(b2, b1, len);
                }
            }
            return adj;
        }

        private static HashSet<(int, int)> GetCurrentBaseCutEdges(CutMesh mesh, int[] localToBase)
        {
            var cuts = new HashSet<(int, int)>();
            for (int i = 0; i < mesh.Edges.Count; i++)
            {
                if (mesh.GetFacesForEdge(i).Count == 1) // 境界エッジ＝切込み
                {
                    int b1 = localToBase[mesh.Edges[i][0]], b2 = localToBase[mesh.Edges[i][1]];
                    cuts.Add(b1 < b2 ? (b1, b2) : (b2, b1));
                }
            }
            return cuts;
        }

        private static CutMesh ReconstructFromBaseCuts(CutMesh raw, HashSet<(int, int)> baseCutEdges, int[] localToBase)
        {
            // 1. 全てのDuplicatedVertIndicesをマージした「ベースメッシュ」を仮想的に構築
            // (既存の頂点座標と面構成は変えずに、Duplicatedリストだけリセットする)
            List<Point3d> verts = raw.Vertices.Take(raw.DuplicatedVertIndices.Count).ToList();
            // ※ここでは簡略化のため、各グループの[0]番目の座標をベース頂点座標とする
            verts = raw.DuplicatedVertIndices.Select(g => raw.Vertices[g[0]]).ToList();

            int[,] faces = new int[raw.Faces.GetLength(0), 3];
            for (int i = 0; i < faces.GetLength(0); i++)
                for (int j = 0; j < 3; j++) faces[i, j] = localToBase[raw.Faces[i, j]];

            // 2. ベースエッジリストの作成
            var baseEdges = new List<int[]>();
            var edgeSet = new HashSet<(int, int)>();
            for (int i = 0; i < faces.GetLength(0); i++)
                for (int j = 0; j < 3; j++)
                {
                    int v1 = faces[i, j], v2 = faces[i, (j + 1) % 3];
                    var k = v1 < v2 ? (v1, v2) : (v2, v1);
                    if (edgeSet.Add(k)) baseEdges.Add(new[] { k.Item1, k.Item2 });
                }

            // 3. CutMeshWithEdgeIndices の呼び出しに必要な情報を整理
            // ベースエッジのリストの中で、どのインデックスが baseCutEdges に含まれるか特定
            List<int> finalCutIndices = new List<int>();
            for (int i = 0; i < baseEdges.Count; i++)
            {
                var k = (baseEdges[i][0], baseEdges[i][1]);
                if (baseCutEdges.Contains(k)) finalCutIndices.Add(i);
            }

            // 4. 重複なしのベースCutMeshを一旦作り、そこから切り開く
            var baseDup = Enumerable.Range(0, verts.Count).Select(i => new List<int> { i }).ToList();
            CutMesh baseMesh = new CutMesh(verts, faces, baseEdges, baseDup);

            return MeshCutTools.CutMeshWithEdgeIndices(baseMesh, finalCutIndices);
        }

        // --- 補助関数群 ---
        private static List<List<int>> GetFacePatches(CutMesh mesh)
        {
            int fCount = mesh.Faces.GetLength(0);
            int[] labels = new int[fCount];
            for (int i = 0; i < fCount; i++) labels[i] = -1;
            List<List<int>> patches = new List<List<int>>();
            var e2f = mesh.BuildEdgeFacesCache();
            for (int i = 0; i < fCount; i++)
            {
                if (labels[i] != -1) continue;
                var current = new List<int>();
                var stack = new Stack<int>();
                stack.Push(i);
                labels[i] = patches.Count;
                while (stack.Count > 0)
                {
                    int f = stack.Pop(); current.Add(f);
                    foreach (int e in mesh.GetEdgesForFace(f))
                        if (e2f.TryGetValue(e, out var neighbors) && neighbors.Count == 2)
                        {
                            int nf = (neighbors[0] == f) ? neighbors[1] : neighbors[0];
                            if (labels[nf] == -1) { labels[nf] = patches.Count; stack.Push(nf); }
                        }
                }
                patches.Add(current);
            }
            return patches;
        }

        private static (List<int> Vertices, List<int> Edges) GetPatchBoundary(CutMesh mesh, List<int> faces)
        {
            var counts = new Dictionary<int, int>();
            foreach (int f in faces) foreach (int e in mesh.GetEdgesForFace(f))
                {
                    if (!counts.ContainsKey(e)) counts[e] = 0;
                    counts[e]++;
                }
            var bEdges = counts.Where(kv => kv.Value == 1).Select(kv => kv.Key).ToList();
            return (mesh.EdgesToVerts(bEdges), bEdges);
        }

        private static List<int> IdentifySplitVertices(CutMesh mesh, List<int> bVerts)
        {
            return bVerts.Where(v => mesh.DuplicatedVertIndices.Any(g => g.Contains(v) && g.Count >= 3)).ToList();
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
