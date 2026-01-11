using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Collections;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Forms.VisualStyles;


namespace Hagoromo.GeometryTools
{
    public class CutMesh
    {
        public List<Point3d> Vertices { get; set; }

        // Faces[i,j]はi番目のfaceのj番目のVerticesでのindex(j=0,1,2)
        public int[,] Faces { get; set; }

        // Edges[i] = int[2] : i番目のedgeの両端頂点インデックス
        public List<int[]> Edges { get; set; }

        //重複している(常に同じ座標にいてほしい)頂点のindexのグループ
        public List<List<int>> DuplicatedVertIndices { get; set; }

        //コンストラクタでCutMesh型の変数が作られるときは自動で作るが、CutMeshのVFEのどれかを更新してもReloadしなければ
        //これらは更新されない
        private Dictionary<int, List<int>> VertexToEdgesCache { get; set; }

        private Dictionary<int, List<int>> EdgeToFacesCache { get; set; }

        private Dictionary<int, List<int>> VertexToFacesCache { get; set; }


        /*----------------------------------------------コンストラクタ-------------------------------------------------------*/
        public CutMesh()
        {
            Vertices = new List<Point3d>();
            Faces = new int[,] { };
            Edges = new List<int[]>();
            DuplicatedVertIndices = new List<List<int>> ();
            VertexToEdgesCache = new Dictionary<int, List<int>>();
            EdgeToFacesCache = new Dictionary<int, List<int>>();
            VertexToFacesCache = new Dictionary<int, List<int>>();
        }

        public CutMesh(List<Point3d> vertices, int[,] faces, List<int[]> edges, List<List<int>> dVertIndices)
        {
            Vertices = new List<Point3d>(vertices);
            Faces = (int[,])faces.Clone();

            Edges = new List<int[]>();
            foreach (var e in edges) Edges.Add((int[])e.Clone());

            DuplicatedVertIndices = new List<List<int>>();
            foreach (var list in dVertIndices)
                DuplicatedVertIndices.Add(new List<int>(list));
            VertexToEdgesCache = new Dictionary<int, List<int>> (BuildVertexEdgesCache());
            EdgeToFacesCache = new Dictionary<int, List<int>> (BuildEdgeFacesCache());
            VertexToFacesCache = new Dictionary<int, List<int>>(BuildVertexFacesCache());
        }

        public CutMesh(Rhino.Geometry.Mesh mesh)
        {
            Vertices = mesh.Vertices.ToPoint3dArray().ToList();
            Faces = new int[mesh.Faces.Count, 3];

            for (int i = 0; i < mesh.Faces.Count; i++)
            {
                var f = mesh.Faces[i];
                Faces[i, 0] = f.A;
                Faces[i, 1] = f.B;
                Faces[i, 2] = f.C;
            }

            // Edges を構築
            Edges = new List<int[]>();
            var edgeSet = new HashSet<(int, int)>();

            for (int fi = 0; fi < Faces.GetLength(0); fi++)
            {
                for (int j = 0; j < 3; j++)
                {
                    int v0 = Faces[fi, j];
                    int v1 = Faces[fi, (j + 1) % 3];
                    var e = (Math.Min(v0, v1), Math.Max(v0, v1));

                    if (!edgeSet.Contains(e))
                    {
                        edgeSet.Add(e);
                        Edges.Add(new int[] { e.Item1, e.Item2 });
                    }
                }
            }

            DuplicatedVertIndices = Enumerable.Range(0, Vertices.Count)
                              .Select(i => new List<int> { i })
                              .ToList();
            
            VertexToEdgesCache = new Dictionary<int, List<int>>(BuildVertexEdgesCache());
            EdgeToFacesCache = new Dictionary<int, List<int>>(BuildEdgeFacesCache());
            VertexToFacesCache = new Dictionary<int, List<int>>(BuildVertexFacesCache());
            /*
            VertexToEdgesCache = BuildVertexEdgesCache();
            EdgeToFacesCache = BuildEdgeFacesCache();
            VertexToFacesCache = BuildVertexFacesCache();
            */
        }

        public CutMesh Clone()
        {
            // Vertices のコピー（新しい List）
            var newVertices = new List<Point3d>(Vertices);

            // Faces のコピー（2次元配列のクローン）
            var newFaces = (int[,])Faces.Clone();

            // Edges のコピー（List<int[]> 内の配列も個別にコピー）
            var newEdges = new List<int[]>();
            foreach (var e in Edges)
                newEdges.Add((int[])e.Clone());

            // DuplicatedVertIndices のコピー（内側の List<int> も複製）
            var newDuplicatedVertIndices = new List<List<int>>();
            foreach (var group in DuplicatedVertIndices)
                newDuplicatedVertIndices.Add(new List<int>(group));

            // 新しい CutMesh を返す
            CutMesh clone =  new CutMesh(newVertices, newFaces, newEdges, newDuplicatedVertIndices);

            return clone;
        }



        public void ReloadVertexToEdgesCache()
        {
            VertexToEdgesCache = BuildVertexEdgesCache();
        }
        public void ReloadEdgeToFacesCache()
        {
            EdgeToFacesCache = BuildEdgeFacesCache();
        }
        public void ReloadVertexToFacesCache()
        {
            VertexToFacesCache = BuildVertexFacesCache();
        }

        public Dictionary<int, List<int>> GetEdgeToFacesCache()
        {
            return new Dictionary<int, List<int>>(EdgeToFacesCache);
        }


        /*----------------------------------------------辞書-------------------------------------------------------*/
        public Dictionary<int, List<int>> BuildEdgeFacesCache()
        {
            Dictionary<int, List<int>> edgeToFacesCache = new Dictionary<int, List<int>>();
            int faceCount = Faces.GetLength(0);

            for (int fi = 0; fi < faceCount; fi++)
            {
                for (int vi = 0; vi < 3; vi++) // 三角形の場合
                {
                    int v0 = Faces[fi, vi];
                    int v1 = Faces[fi, (vi + 1) % 3];

                    int edgeIndex = FindEdgeIndex(v0, v1);
                    if (edgeIndex < 0) continue;

                    if (!edgeToFacesCache.ContainsKey(edgeIndex))
                        edgeToFacesCache[edgeIndex] = new List<int>();

                    edgeToFacesCache[edgeIndex].Add(fi);
                }
            }
            return edgeToFacesCache;
        }
        public Dictionary<int, List<int>> BuildVertexEdgesCache()
        {
            Dictionary<int, List<int>> vertexToEdges = new Dictionary<int, List<int>>();

            for (int ei = 0; ei < Edges.Count; ei++)
            {
                int[] edge = Edges[ei];
                foreach (int vi in edge)
                {
                    if (!vertexToEdges.ContainsKey(vi))
                        vertexToEdges[vi] = new List<int>();

                    vertexToEdges[vi].Add(ei);
                }
            }
            return vertexToEdges;
        }
        public Dictionary<int, List<int>> BuildVertexFacesCache()
        {
            Dictionary<int, List<int>> vertexToFaces = new Dictionary<int, List<int>>();
            int faceCount = Faces.GetLength(0);

            for (int fi = 0; fi < faceCount; fi++)
            {
                for (int vi = 0; vi < 3; vi++) // 三角形なので3頂点
                {
                    int vertexIndex = Faces[fi, vi];

                    if (!vertexToFaces.ContainsKey(vertexIndex))
                        vertexToFaces[vertexIndex] = new List<int>();

                    vertexToFaces[vertexIndex].Add(fi);
                }
            }

            return vertexToFaces;
        }




        /*------------------------------------EFdictionary更新必要関数-------------------------------------------*/

        // あるエッジを含むフェイスを取得
        public List<int> GetFacesForEdge(int edgeIndex)
        {

            if (EdgeToFacesCache != null && EdgeToFacesCache.TryGetValue(edgeIndex, out var faces))
                return new List<int> (faces);

            return new List<int>();
        }

        // 境界エッジのインデックス
        public List<int> BoundaryEdgeIndices()
        {
            return Edges
                .Select((e, i) => new { edge = e, index = i })
                .Where(x => GetFacesForEdge(x.index).Count == 1)
                .Select(x => x.index)
                .ToList();
        }

        //境界の点のインデックス
        public List<int> BoundaryVertIndices()
        {
            List<int> boundaryEdges = BoundaryEdgeIndices();
            HashSet<int> boundaryVertices = new HashSet<int>();
            foreach (int ei in boundaryEdges)
            {
                boundaryVertices.Add(Edges[ei][0]);
                boundaryVertices.Add(Edges[ei][1]);
            }
            return boundaryVertices.ToList();
        }

        //polyEdgesからそれを通る点群のインデックスを返す
        public List<int> EdgesToVerts(List<int> edges)
        {
            HashSet<int> vertices = new HashSet<int>();
            foreach (int ei in edges)
            {
                vertices.Add(Edges[ei][0]);
                vertices.Add(Edges[ei][1]);
            }
            return vertices.ToList();
        }


        //内側の点のインデックス
        /*
            InternalVertIndicesは以下のように入手できる         
            List<int> fullSet = Enumerable.Range(0, cutMesh.Vertices.Count).ToList();
            List<int> InternalVertIndices = fullSet.Except(boundaryVertIndices()).ToList();
        */

        //あるfaceのとなりのfacesたち
        public List<int> GetFacesForFace(int faceIndex)
        {
            int[] faceEdges = GetEdgesForFace(faceIndex);
            List<int> result = new List<int>();
            foreach (int ei in faceEdges)
            {
                result.AddRange(GetFacesForEdge(ei).Where(x => x != faceIndex).ToList());
            }
            return result;
        }

        //あるfaceが境界線を持っていたらtrue
        bool FaceHasBoundaryEdge(int faceIndex)
        {
            int[] faceEdges = GetEdgesForFace(faceIndex);
            foreach (int ei in faceEdges)
            {
                // そのエッジが境界エッジか？
                if (GetFacesForEdge(ei).Count == 1)
                {
                    return true; // 境界エッジあり
                }
            }

            return false; // 境界エッジなし
        }

        //boundary→internalの順に点を並び替える。boundaryはそのpolyliineの順になるようにverticesとedgesを並び替えている。
        //VEdictionaryも更新必要
        public CutMesh Sort3()
        {
            ReloadVertexToEdgesCache();
            ReloadEdgeToFacesCache();
            ReloadVertexToFacesCache();
            List<int> boundaryVertIndices = BoundaryVertIndices();
            List<int> boundaryEdges = BoundaryEdgeIndices();
            List<int> fullSetV = Enumerable.Range(0, Vertices.Count).ToList();
            List<int> internalVertIndices = fullSetV.Except(boundaryVertIndices).ToList();
            List<int> sortBoundaryIndices = new List<int>();
            List<int> boundaryPart = new List<int>();
            int boundaryCurrentVertex = boundaryVertIndices[0];
            sortBoundaryIndices.Add(boundaryCurrentVertex);
            for (int i = 0; i < boundaryVertIndices.Count - 1; i++)
            {
                List<int> arroundEdges = GetEdgesForVertex(boundaryCurrentVertex);
                List<int> twoVertices = new List<int> { };
                foreach (int ei in arroundEdges)
                {
                    if (boundaryEdges.Contains(ei))
                    {
                        twoVertices.AddRange(new List<int> { Edges[ei][0], Edges[ei][1] });
                    }
                }
                int nextVertex = -1;
                if (twoVertices.Any(x => !sortBoundaryIndices.Contains(x)))
                {
                    nextVertex = (twoVertices.Where(x => !sortBoundaryIndices.Contains(x)).ToList())[0];
                }
                else
                {
                    boundaryPart.Add(i);
                    nextVertex = (boundaryVertIndices.Where(x => !sortBoundaryIndices.Contains(x)).ToList())[0];
                }
                sortBoundaryIndices.Add(nextVertex);
                boundaryCurrentVertex = nextVertex;

            }
            boundaryPart.Add(boundaryVertIndices.Count-1);
            //int countcount = 0;
            /*
            while (DuplicatedVertIndices.FirstOrDefault(inner => inner.Contains(sortBoundaryIndices[0])).ToArray().Length > 1 && countcount <sortBoundaryIndices.Count+1)
            {
                int first = sortBoundaryIndices[0];       // 先頭要素を取得
                sortBoundaryIndices.RemoveAt(0);          // 先頭要素を削除
                sortBoundaryIndices.Add(first);           // 末尾に追加
                countcount++;
            }
            */
            /*
            List<Point3d> vertices = new List<Point3d>();
            int count = sortBoundaryIndices.Count;
            for (int i = 0; i < count; i++)
            {
                vertices.Add(Vertices[sortBoundaryIndices[i]]);
            }
            for (int i = 0; i < internalVertIndices.Count; i++)
            {
                vertices.Add(Vertices[internalVertIndices[i]]);
            }

            List<int> relation = sortBoundaryIndices.ToList();
            relation.AddRange(internalVertIndices);
            Dictionary<int, int> indexMap = relation.Select((originalIndex, newIndex) => new { originalIndex, newIndex }).ToDictionary(x => x.originalIndex, x => x.newIndex);
            */


            // 並べ替え後の頂点インデックスを1本にまとめる
            var relation = sortBoundaryIndices.Concat(internalVertIndices).ToList();

            // vertices と indexMap を relation に従って作る
            var vertices = relation.Select(i => Vertices[i]).ToList();
            var indexMap = relation.Select((orig, i) => new { orig, i })
                                   .ToDictionary(x => x.orig, x => x.i);


            int count = sortBoundaryIndices.Count;
            List<int[]> edges = new List<int[]>();
            List<int> boundaryEdgeIndices = BoundaryEdgeIndices();
            List<int> fullSetE = Enumerable.Range(0, Edges.Count).ToList();
            List<int> internalEdgeIndices = fullSetE.Except(boundaryEdgeIndices).ToList();
            for (int i = 0; i < count; i++)
            {
                if (boundaryPart.Contains(i))
                {
                    int indexofi = boundaryPart.IndexOf(i);
                    if (indexofi == 0) { edges.Add(new int[] { i, 0 }); }
                    else { edges.Add(new int[] { i, boundaryPart[indexofi - 1] + 1 }); }
                }
                else
                {
                    edges.Add(new int[] { i, (i + 1) % count });
                }
            }
            foreach (int ei in internalEdgeIndices)
            {
                edges.Add(new int[] { indexMap[Edges[ei][0]], indexMap[Edges[ei][1]] });
            }

            int[,] faces = new int[Faces.GetLength(0), 3];
            for (int i = 0; i < Faces.GetLength(0); i++)
            {
                faces[i, 0] = indexMap[Faces[i, 0]];
                faces[i, 1] = indexMap[Faces[i, 1]];
                faces[i, 2] = indexMap[Faces[i, 2]];
            }

            List<List<int>> dVertIndices = new List<List<int>>();
            for (int i = 0; i < DuplicatedVertIndices.Count; i++)
            {
                dVertIndices.Add(new List<int>());
                for (int j = 0; j < DuplicatedVertIndices[i].Count; j++)
                {
                    dVertIndices[i].Add(indexMap[DuplicatedVertIndices[i][j]]);
                }
            }
            CutMesh cutMesh = new CutMesh(vertices, faces, edges, dVertIndices);
            return cutMesh;
        }

        public CutMesh Sort()
        {
            ReloadVertexToEdgesCache();
            ReloadEdgeToFacesCache();
            ReloadVertexToFacesCache();
            List<int> boundaryVertIndices = BoundaryVertIndices();
            List<int> boundaryEdges = BoundaryEdgeIndices();
            List<int> fullSetV = Enumerable.Range(0, Vertices.Count).ToList();
            List<int> internalVertIndices = fullSetV.Except(boundaryVertIndices).ToList();
            List<int> sortBoundaryIndices = new List<int>();
            List<int> boundaryPart = new List<int>();
            int boundaryCurrentVertex = boundaryVertIndices[0];
            sortBoundaryIndices.Add(boundaryCurrentVertex);
            for (int i = 0; i < boundaryVertIndices.Count - 1; i++)
            {
                List<int> arroundEdges = GetEdgesForVertex(boundaryCurrentVertex);
                List<int> twoVertices = new List<int> { };
                foreach (int ei in arroundEdges)
                {
                    if (boundaryEdges.Contains(ei))
                    {
                        twoVertices.AddRange(new List<int> { Edges[ei][0], Edges[ei][1] });
                    }
                }
                int nextVertex = -1;
                if (twoVertices.Any(x => !sortBoundaryIndices.Contains(x)))
                {
                    nextVertex = (twoVertices.Where(x => !sortBoundaryIndices.Contains(x)).ToList())[0];
                }
                else
                {
                    boundaryPart.Add(i);
                    nextVertex = (boundaryVertIndices.Where(x => !sortBoundaryIndices.Contains(x)).ToList())[0];
                }
                sortBoundaryIndices.Add(nextVertex);
                boundaryCurrentVertex = nextVertex;

            }
            boundaryPart.Add(boundaryVertIndices.Count - 1);



            // 境界のつながり（ループ）ごとの頂点リストを格納するリスト
            List<List<int>> boundaryLoops = new List<List<int>>();

            int startIndex = 0;
            foreach (int endIndex in boundaryPart)
            {
                // 各ループの要素数を計算
                int count2 = endIndex - startIndex + 1;

                // GetRangeを使って指定範囲の頂点を抽出
                List<int> currentLoop = sortBoundaryIndices.GetRange(startIndex, count2);

                boundaryLoops.Add(currentLoop);

                // 次のループの開始位置を更新
                startIndex = endIndex + 1;
            }

            //外部境界線は考えなくてよい。ここでは一番エッジ本数が長いものを外部境界線としている
            List<List<int>> boundaryVertsGroup = new List<List<int>>();
            List<double> area = new List<double>();

            for (int i = 0; i < boundaryLoops.Count; i++)
            {
                var verts = boundaryLoops[i];
                boundaryVertsGroup.Add(verts);
                /*
                // 面積計算
                var pointsList = verts.Select(v => Vertices[v]).ToList();
                var polyline = new Polyline(pointsList);
                var mesh = Mesh.CreateFromClosedPolyline(polyline);
                if (mesh == null) { area.Add(0); }
                else { area.Add(Math.Abs(AreaMassProperties.Compute(mesh).Area)); }
                */
            }

            // 最大面積のインデックス
            //int maxIndex = area.IndexOf(area.Max());
            int maxIndex = boundaryVertsGroup.Select((list, index) => new { list, index }).OrderByDescending(x => x.list.Count).First().index;


            // 1. まず、既存のデータから現在の全てのループ（List<List<int>>）を抽出
            List<List<int>> allLoops = new List<List<int>>();
            int start = 0;
            foreach (int end in boundaryPart)
            {
                allLoops.Add(sortBoundaryIndices.GetRange(start, end - start + 1));
                start = end + 1;
            }

            // 2. 指定したインデックス i (targetLoopIndex) を先頭に並び替える
            int targetLoopIndex = maxIndex; // 先頭にしたいループの番号
            List<List<int>> reorderedLoops = new List<List<int>>();

            // 指定されたループを最初に追加
            reorderedLoops.Add(allLoops[targetLoopIndex]);

            // それ以外のループを順番に追加
            for (int j = 0; j < allLoops.Count; j++)
            {
                if (j == targetLoopIndex) continue;
                reorderedLoops.Add(allLoops[j]);
            }

            // 3. 並び替えたループリストから、新しい sortBoundaryIndices と boundaryPart を作成
            List<int> newSortBoundaryIndices = new List<int>();
            List<int> newBoundaryPart = new List<int>();

            int currentCumulativeCount = 0;
            foreach (var loop in reorderedLoops)
            {
                newSortBoundaryIndices.AddRange(loop);
                currentCumulativeCount += loop.Count;

                // そのループの最後のインデックスを記録
                newBoundaryPart.Add(currentCumulativeCount - 1);
            }

            // 必要に応じて、元の変数に上書き
            sortBoundaryIndices = newSortBoundaryIndices;
            boundaryPart = newBoundaryPart;






            // 並べ替え後の頂点インデックスを1本にまとめる
            var relation = sortBoundaryIndices.Concat(internalVertIndices).ToList();

            // vertices と indexMap を relation に従って作る
            var vertices = relation.Select(i => Vertices[i]).ToList();
            var indexMap = relation.Select((orig, i) => new { orig, i })
                                   .ToDictionary(x => x.orig, x => x.i);


            int count = sortBoundaryIndices.Count;
            List<int[]> edges = new List<int[]>();
            List<int> boundaryEdgeIndices = BoundaryEdgeIndices();
            List<int> fullSetE = Enumerable.Range(0, Edges.Count).ToList();
            List<int> internalEdgeIndices = fullSetE.Except(boundaryEdgeIndices).ToList();
            for (int i = 0; i < count; i++)
            {
                if (boundaryPart.Contains(i))
                {
                    int indexofi = boundaryPart.IndexOf(i);
                    if (indexofi == 0) { edges.Add(new int[] { i, 0 }); }
                    else { edges.Add(new int[] { i, boundaryPart[indexofi - 1] + 1 }); }
                }
                else
                {
                    edges.Add(new int[] { i, (i + 1) % count });
                }
            }
            foreach (int ei in internalEdgeIndices)
            {
                edges.Add(new int[] { indexMap[Edges[ei][0]], indexMap[Edges[ei][1]] });
            }

            int[,] faces = new int[Faces.GetLength(0), 3];
            for (int i = 0; i < Faces.GetLength(0); i++)
            {
                faces[i, 0] = indexMap[Faces[i, 0]];
                faces[i, 1] = indexMap[Faces[i, 1]];
                faces[i, 2] = indexMap[Faces[i, 2]];
            }

            List<List<int>> dVertIndices = new List<List<int>>();
            for (int i = 0; i < DuplicatedVertIndices.Count; i++)
            {
                dVertIndices.Add(new List<int>());
                for (int j = 0; j < DuplicatedVertIndices[i].Count; j++)
                {
                    dVertIndices[i].Add(indexMap[DuplicatedVertIndices[i][j]]);
                }
            }
            CutMesh cutMesh = new CutMesh(vertices, faces, edges, dVertIndices);
            return cutMesh;
        }

        public CutMesh Sort2()
        {
            // キャッシュを最新にする
            ReloadVertexToEdgesCache();
            ReloadEdgeToFacesCache();
            ReloadVertexToFacesCache();

            List<int> boundaryVertIndices = BoundaryVertIndices();
            List<int> boundaryEdges = BoundaryEdgeIndices();
            int totalVertCount = Vertices.Count;

            // --- 高速化の要: List.Contains を bool配列(フラグ)に置き換え ---
            bool[] isSorted = new bool[totalVertCount];
            List<int> sortBoundaryIndices = new List<int>(boundaryVertIndices.Count);
            List<int> boundaryPart = new List<int>();

            // 境界エッジを高速に引くための辞書（境界頂点 -> その頂点に繋がる境界エッジ）
            var vertToBoundaryEdges = new Dictionary<int, List<int>>();
            foreach (int ei in boundaryEdges)
            {
                foreach (int v in Edges[ei])
                {
                    if (!vertToBoundaryEdges.ContainsKey(v)) vertToBoundaryEdges[v] = new List<int>();
                    vertToBoundaryEdges[v].Add(ei);
                }
            }

            int currentVertex = boundaryVertIndices[0];
            sortBoundaryIndices.Add(currentVertex);
            isSorted[currentVertex] = true;

            // 全境界頂点を処理するまでループ
            for (int i = 0; i < boundaryVertIndices.Count - 1; i++)
            {
                List<int> aroundEdges = vertToBoundaryEdges.ContainsKey(currentVertex)
                                        ? vertToBoundaryEdges[currentVertex]
                                        : new List<int>();
                int nextVertex = -1;

                // 次の境界頂点を探す
                foreach (int ei in aroundEdges)
                {
                    int v0 = Edges[ei][0];
                    int v1 = Edges[ei][1];
                    int candidate = (v0 == currentVertex) ? v1 : v0;

                    if (!isSorted[candidate])
                    {
                        nextVertex = candidate;
                        break;
                    }
                }

                // 見つからない場合は新しいループ(穴など)へジャンプ
                if (nextVertex == -1)
                {
                    boundaryPart.Add(sortBoundaryIndices.Count - 1);
                    // まだソートされていない境界頂点を1つ選ぶ
                    foreach (int bv in boundaryVertIndices)
                    {
                        if (!isSorted[bv])
                        {
                            nextVertex = bv;
                            break;
                        }
                    }
                }

                if (nextVertex != -1)
                {
                    sortBoundaryIndices.Add(nextVertex);
                    isSorted[nextVertex] = true;
                    currentVertex = nextVertex;
                }
            }
            boundaryPart.Add(sortBoundaryIndices.Count - 1);

            // 内部頂点の取得
            List<int> internalVertIndices = new List<int>();
            for (int i = 0; i < totalVertCount; i++)
            {
                if (!isSorted[i]) internalVertIndices.Add(i);
            }

            // 新旧インデックスの対応マップ (Dictionaryの代わりに配列を使って高速化)
            var relation = sortBoundaryIndices.Concat(internalVertIndices).ToList();
            int[] indexMap = new int[totalVertCount];
            for (int i = 0; i < relation.Count; i++)
            {
                indexMap[relation[i]] = i;
            }

            // 頂点の再配置
            List<Point3d> newVertices = relation.Select(i => Vertices[i]).ToList();

            // エッジの再構築
            List<int[]> newEdges = new List<int[]>(Edges.Count);
            int bCount = sortBoundaryIndices.Count;

            // boundaryPartを高速に引くためのフラグ
            bool[] isPartEnd = new bool[bCount];
            foreach (int p in boundaryPart) isPartEnd[p] = true;

            for (int i = 0; i < bCount; i++)
            {
                if (isPartEnd[i])
                {
                    // ループを閉じる
                    int partIdx = boundaryPart.IndexOf(i);
                    int startOfPart = (partIdx == 0) ? 0 : boundaryPart[partIdx - 1] + 1;
                    newEdges.Add(new int[] { i, startOfPart });
                }
                else
                {
                    newEdges.Add(new int[] { i, i + 1 });
                }
            }

            // 内部エッジ
            List<int> boundaryEdgeIndicesSet = new HashSet<int>(boundaryEdges).ToList(); // 境界エッジのインデックス
            for (int ei = 0; ei < Edges.Count; ei++)
            {
                if (!boundaryEdges.Contains(ei))
                {
                    newEdges.Add(new int[] { indexMap[Edges[ei][0]], indexMap[Edges[ei][1]] });
                }
            }

            // 面の再構築
            int faceCount = Faces.GetLength(0);
            int[,] newFaces = new int[faceCount, 3];
            for (int i = 0; i < faceCount; i++)
            {
                newFaces[i, 0] = indexMap[Faces[i, 0]];
                newFaces[i, 1] = indexMap[Faces[i, 1]];
                newFaces[i, 2] = indexMap[Faces[i, 2]];
            }

            // 重複頂点インデックスの更新
            List<List<int>> dVertIndices = new List<List<int>>();
            foreach (var group in DuplicatedVertIndices)
            {
                dVertIndices.Add(group.Select(oldIdx => indexMap[oldIdx]).ToList());
            }

            return new CutMesh(newVertices, newFaces, newEdges, dVertIndices);
        }

        public (CutMesh Mesh,List<int> VertexMap) Sort2WithRelation()
        {
            // キャッシュを最新にする
            ReloadVertexToEdgesCache();
            ReloadEdgeToFacesCache();
            ReloadVertexToFacesCache();

            List<int> boundaryVertIndices = BoundaryVertIndices();
            List<int> boundaryEdges = BoundaryEdgeIndices();
            int totalVertCount = Vertices.Count;

            // --- 高速化の要: List.Contains を bool配列(フラグ)に置き換え ---
            bool[] isSorted = new bool[totalVertCount];
            List<int> sortBoundaryIndices = new List<int>(boundaryVertIndices.Count);
            List<int> boundaryPart = new List<int>();

            // 境界エッジを高速に引くための辞書（境界頂点 -> その頂点に繋がる境界エッジ）
            var vertToBoundaryEdges = new Dictionary<int, List<int>>();
            foreach (int ei in boundaryEdges)
            {
                foreach (int v in Edges[ei])
                {
                    if (!vertToBoundaryEdges.ContainsKey(v)) vertToBoundaryEdges[v] = new List<int>();
                    vertToBoundaryEdges[v].Add(ei);
                }
            }

            int currentVertex = boundaryVertIndices[0];
            sortBoundaryIndices.Add(currentVertex);
            isSorted[currentVertex] = true;

            // 全境界頂点を処理するまでループ
            for (int i = 0; i < boundaryVertIndices.Count - 1; i++)
            {
                List<int> aroundEdges = vertToBoundaryEdges.ContainsKey(currentVertex)
                                        ? vertToBoundaryEdges[currentVertex]
                                        : new List<int>();
                int nextVertex = -1;

                // 次の境界頂点を探す
                foreach (int ei in aroundEdges)
                {
                    int v0 = Edges[ei][0];
                    int v1 = Edges[ei][1];
                    int candidate = (v0 == currentVertex) ? v1 : v0;

                    if (!isSorted[candidate])
                    {
                        nextVertex = candidate;
                        break;
                    }
                }

                // 見つからない場合は新しいループ(穴など)へジャンプ
                if (nextVertex == -1)
                {
                    boundaryPart.Add(sortBoundaryIndices.Count - 1);
                    // まだソートされていない境界頂点を1つ選ぶ
                    foreach (int bv in boundaryVertIndices)
                    {
                        if (!isSorted[bv])
                        {
                            nextVertex = bv;
                            break;
                        }
                    }
                }

                if (nextVertex != -1)
                {
                    sortBoundaryIndices.Add(nextVertex);
                    isSorted[nextVertex] = true;
                    currentVertex = nextVertex;
                }
            }
            boundaryPart.Add(sortBoundaryIndices.Count - 1);

            // 内部頂点の取得
            List<int> internalVertIndices = new List<int>();
            for (int i = 0; i < totalVertCount; i++)
            {
                if (!isSorted[i]) internalVertIndices.Add(i);
            }

            // 新旧インデックスの対応マップ (Dictionaryの代わりに配列を使って高速化)
            var relation = sortBoundaryIndices.Concat(internalVertIndices).ToList();
            int[] indexMap = new int[totalVertCount];
            for (int i = 0; i < relation.Count; i++)
            {
                indexMap[relation[i]] = i;
            }

            // 頂点の再配置
            List<Point3d> newVertices = relation.Select(i => Vertices[i]).ToList();

            // エッジの再構築
            List<int[]> newEdges = new List<int[]>(Edges.Count);
            int bCount = sortBoundaryIndices.Count;

            // boundaryPartを高速に引くためのフラグ
            bool[] isPartEnd = new bool[bCount];
            foreach (int p in boundaryPart) isPartEnd[p] = true;

            for (int i = 0; i < bCount; i++)
            {
                if (isPartEnd[i])
                {
                    // ループを閉じる
                    int partIdx = boundaryPart.IndexOf(i);
                    int startOfPart = (partIdx == 0) ? 0 : boundaryPart[partIdx - 1] + 1;
                    newEdges.Add(new int[] { i, startOfPart });
                }
                else
                {
                    newEdges.Add(new int[] { i, i + 1 });
                }
            }

            // 内部エッジ
            List<int> boundaryEdgeIndicesSet = new HashSet<int>(boundaryEdges).ToList(); // 境界エッジのインデックス
            for (int ei = 0; ei < Edges.Count; ei++)
            {
                if (!boundaryEdges.Contains(ei))
                {
                    newEdges.Add(new int[] { indexMap[Edges[ei][0]], indexMap[Edges[ei][1]] });
                }
            }

            // 面の再構築
            int faceCount = Faces.GetLength(0);
            int[,] newFaces = new int[faceCount, 3];
            for (int i = 0; i < faceCount; i++)
            {
                newFaces[i, 0] = indexMap[Faces[i, 0]];
                newFaces[i, 1] = indexMap[Faces[i, 1]];
                newFaces[i, 2] = indexMap[Faces[i, 2]];
            }

            // 重複頂点インデックスの更新
            List<List<int>> dVertIndices = new List<List<int>>();
            foreach (var group in DuplicatedVertIndices)
            {
                dVertIndices.Add(group.Select(oldIdx => indexMap[oldIdx]).ToList());
            }

            return (new CutMesh(newVertices, newFaces, newEdges, dVertIndices), relation);
        }
        /*---------------------------------------------VEdictionary更新必要関数-------------------------------------------*/

        //ある点につくエッジたちを返す
        public List<int> GetEdgesForVertex(int vertexIndex)
        {
            if (VertexToEdgesCache != null && VertexToEdgesCache.TryGetValue(vertexIndex, out var edges))
                return new List<int> (edges);

            return new List<int>();
        }

        // ある点の隣の点たちを返す
        public List<int> GetVerticesForVertex(int vertexIndex)
        {
            var result = new List<int>();

            if (VertexToEdgesCache == null || !VertexToEdgesCache.TryGetValue(vertexIndex, out var edgeIndices))
                return result;

            foreach (int ei in edgeIndices)
            {
                var edge = Edges[ei];
                // edge[0]が自分ならedge[1]を、edge[1]が自分ならedge[0]を追加
                int other = (edge[0] == vertexIndex) ? edge[1] : edge[0];
                result.Add(other);
            }

            return result;
        }





        /*---------------------------------------------VFdictionary更新必要関数-------------------------------------------*/
        //順不同である点を持つフェイスたちを返す
        public List<int>GetFacesForVertex(int vertexIndex)
        {
            if (VertexToFacesCache != null && VertexToFacesCache.TryGetValue(vertexIndex, out var faces))
                return new List<int> (faces);

            return new List<int>();
        }

        //EFdictionaryも更新必要。反時計か時計回りの順である点を持つfaceたちを返す。境界上の点ならその端から順。
        public List<int> GetOrderedFacesForVertex(int vertIndex)
        {
            //Point3d v = Vertices[vertIndex];
            List<int> connectedFaces = GetFacesForVertex(vertIndex);


            List<int> connectedEdges = GetEdgesForVertex(vertIndex);
            if (connectedFaces == null || connectedFaces.Count == 0)
                return new List<int>();
            if (connectedFaces.Count == 1)
            {
                return connectedFaces;
            }
            List<int> order = new List<int>();
            order.Add(connectedFaces[0]);
            List<int> edgeList = new List<int>();
            int nextFace = connectedFaces[0];
            for (int i = 0; i < connectedFaces.Count - 1; i++)
            {
                List<int> TheEdges = (GetEdgesForFace(nextFace).ToList()).Intersect(connectedEdges).ToList();
                if (edgeList.Contains(TheEdges[0])) { edgeList.Add(TheEdges[1]); TheEdges.RemoveAt(0); }
                else { edgeList.Add(TheEdges[0]); TheEdges.RemoveAt(1); }
                List<int> facesForEdge = GetFacesForEdge(TheEdges[0]);
                if (facesForEdge.Count == 1)
                {
                    List<int> DoubleEdges = (GetEdgesForFace(connectedFaces[0]).ToList()).Intersect(connectedEdges).ToList();
                    edgeList.Add(DoubleEdges[1]);
                    facesForEdge = GetFacesForEdge(DoubleEdges[1]);
                    facesForEdge.Remove(connectedFaces[0]);
                    nextFace = facesForEdge[0];
                    order.Reverse();
                }
                else
                {
                    facesForEdge.Remove(nextFace);
                    nextFace = facesForEdge[0];
                }
                order.Add(nextFace);
            }
            return order;




            /*
            // 頂点を基準にフェイス重心の角度を計算
            var faceAngles = new List<(int faceIndex, double angle)>();
            foreach (int fi in connectedFaces)
            {
                Point3d fc = GetFaceCenter(fi);
                Vector3d dir = fc - v;
                double angle = Math.Atan2(dir.Y, dir.X); // XY平面に投影して角度
                faceAngles.Add((fi, angle));
            }

            // 角度でソート（昇順 = CCW）
            faceAngles.Sort((a, b) => a.angle.CompareTo(b.angle));

            List<int> order = faceAngles.Select(f => f.faceIndex).ToList();
           
            List<int> boundaryVertIndices = BoundaryVertIndices();
            int count = order.Count;
            if (!(boundaryVertIndices.Contains(vertIndex)) || count <= 2)
            {
                return order;
            }


            //点が境界上の時かつそこに3つ以上のフェイスがあるとき
            else
            {
                List<int> newOrder = new List<int>();
                List<int> boundaryIndices = BoundaryEdgeIndices();
                for (int i = 0; i < count; i++)
                {
                    List<int> threeEdges = GetEdgesForFace(order[i]).ToList();
                    List<int> twoEdges = GetEdgesForFace(order[i]).ToList();
                    for (int j = 0; j < 3; j++)
                    {
                        if (Edges[threeEdges[j]][0] != vertIndex && Edges[threeEdges[j]][1] != vertIndex)
                        {
                            twoEdges.RemoveAt(j);
                        }
                    }
                    if (boundaryIndices.Contains(twoEdges[0]) || boundaryIndices.Contains(twoEdges[1]))
                    {
                        threeEdges = GetEdgesForFace(order[(i + 1) % count]).ToList();
                        twoEdges = GetEdgesForFace(order[(i + 1) % count]).ToList();
                        for (int j = 0; j < 3; j++)
                        {
                            if (Edges[threeEdges[j]][0] != vertIndex && Edges[threeEdges[j]][1] != vertIndex)
                            {
                                twoEdges.RemoveAt(j);
                            }
                        }
                        if (!(boundaryIndices.Contains(twoEdges[0]) || boundaryIndices.Contains(twoEdges[1])))
                        {
                            newOrder = order.Skip(i).Concat(order.Take(i)).ToList();
                        }
                    }
                }
                return newOrder;
                */
        }
            
        
        




        /*-------------------------------------------辞書更新不要なメソッド-------------------------------------------------------*/

        //verticesのindex２つからその2つを端点とするedgeのインデックスを返す
        public int FindEdgeIndex(int v0, int v1)
        {
            for (int i = 0; i < Edges.Count; i++)
            {
                var e = Edges[i];
                if ((e[0] == v0 && e[1] == v1) || (e[0] == v1 && e[1] == v0))
                    return i;
            }
            return -1;
        }

        public int GetEdgeForEndPoints(int v0, int v1)
        {
            List<int> edges = GetEdgesForVertex(v0);
            foreach( int edge in edges)
            {
                if (Edges[edge].Contains(v1)) return edge;
            }
            return -1;
        }

        //faceの持つedgeのインデックスたち（３つ）を返す
        public int[] GetEdgesForFace(int faceIndex)
        {
            int[] edgeIndices = new int[3];
            edgeIndices[0] = FindEdgeIndex(Faces[faceIndex, 0], Faces[faceIndex, 1]);
            edgeIndices[1] = FindEdgeIndex(Faces[faceIndex, 1], Faces[faceIndex, 2]);
            edgeIndices[2] = FindEdgeIndex(Faces[faceIndex, 2], Faces[faceIndex, 0]);
            return edgeIndices;
        }

        public int[] GetVerticesForFace(int faceIndex)
        {
            int[] vertIndices = new int[3];
            vertIndices[0] = Faces[faceIndex, 0];
            vertIndices[1] = Faces[faceIndex, 1];
            vertIndices[2] = Faces[faceIndex, 2];
            return vertIndices;
        }


        //edgeインデックスのラインを返す
        public Line GetEdgeLine(int edgeIndex)
        {
            var e = Edges[edgeIndex];
            return new Line(Vertices[e[0]], Vertices[e[1]]);
        }

        //あるfaceの中心の点を返す
        public Point3d GetFaceCenter(int faceIndex)
        {
            Point3d A = Vertices[Faces[faceIndex, 0]];
            Point3d B = Vertices[Faces[faceIndex, 1]];
            Point3d C = Vertices[Faces[faceIndex, 2]];
            return (A + B + C) / 3;
        }

        //Meshに変換
        public Rhino.Geometry.Mesh ConvertToMesh()
        {
            Rhino.Geometry.Mesh mesh = new Rhino.Geometry.Mesh();

            for (int i = 0; i < Vertices.Count; i++)
            {
                mesh.Vertices.Add(Vertices[i]);
            }
            for (int i = 0; i < Faces.GetLength(0); i++)
            {
                mesh.Faces.AddFace(Faces[i, 0], Faces[i, 1], Faces[i, 2]);
            }

            // 法線や境界の再計算
            mesh.Normals.ComputeNormals();
            mesh.Compact();
            return mesh;
        }

        //verticesのi番目の点がdupVertの何番目のグループの点なのかの対応のリスト
        public int[] VertOrderInDup()
        {
            int[] vertOrderInDup  = new int[Vertices.Count];
            for (int i = 0; i < vertOrderInDup.Length; i++)
            {
                vertOrderInDup[i] = -1;
            }
            for (int i = 0; i < DuplicatedVertIndices.Count; i++)
            {
                foreach (int vertIndex in DuplicatedVertIndices[i])
                {
                    vertOrderInDup[vertIndex] = i;
                }
            }
            return vertOrderInDup;
        }

        //dupVertのi番目のvertGroupが他のどのdupVertと隣り合っているかをまとめたリスト
        public List<List<int>> DupConnectedVertIndices()
        {
            int[] vertOrderInDup = VertOrderInDup();
            List<List<int>> dupConnectedVertIndices = new List<List<int>>();
            for (int i = 0; i < DuplicatedVertIndices.Count; i++)
            {
                HashSet<int> connectedDup = new HashSet<int>();
                List<int> vertGroup = DuplicatedVertIndices[i];
                foreach (int vertIndex in vertGroup)
                {
                    List<int> connectedVerts = GetVerticesForVertex(vertIndex);
                    foreach (int vert in connectedVerts)
                    {
                        connectedDup.Add(vertOrderInDup[vert]);
                    }
                }
                dupConnectedVertIndices.Add(connectedDup.ToList());
            }
            return dupConnectedVertIndices;
        }


        public Rhino.Geometry.Mesh ConvertToMesh2()
        {
            int[] vertOrderInDup = VertOrderInDup();
            Rhino.Geometry.Mesh mesh = new Rhino.Geometry.Mesh();

            for (int i = 0; i < DuplicatedVertIndices.Count; i++)
            {
                mesh.Vertices.Add(Vertices[DuplicatedVertIndices[i][0]]);
            }
            for (int i = 0; i < Faces.GetLength(0); i++)
            {
                mesh.Faces.AddFace(vertOrderInDup[Faces[i, 0]], vertOrderInDup[Faces[i, 1]], vertOrderInDup[Faces[i, 2]]);
            }

            // 法線や境界の再計算
            mesh.Normals.ComputeNormals();
            mesh.Compact();
            return mesh;
        }

        
        public CutMesh ConvertToNoSlitCutMesh()
        {
            List<Point3d> vertices = new List<Point3d>();
            List<List<int>> dup = new List<List<int>>();
            int[] vertOrderInDup = VertOrderInDup();
            int[,] faces = new int[Faces.GetLength(0),3];

            for (int i = 0; i < DuplicatedVertIndices.Count; i++)
            {
                vertices.Add(Vertices[DuplicatedVertIndices[i][0]]);
                dup.Add( new List<int> { i } );
            }
            for (int i = 0; i < Faces.GetLength(0); i++)
            {
                faces[i, 0] = vertOrderInDup[Faces[i, 0]];
                faces[i, 1] = vertOrderInDup[Faces[i, 1]];
                faces[i, 2] = vertOrderInDup[Faces[i, 2]];
            }
            List<int[]> edges = new List<int[]>();
            for (int i = 0; i < Edges.Count; i++)
            {
                int e0 = vertOrderInDup[Edges[i][0]];
                int e1 = vertOrderInDup[Edges[i][1]];
                bool flag = true;
                foreach (int[] edge in edges)
                {
                    if ((edge[0] == e0 && edge[1] == e1) || (edge[1] == e0 && edge[0] == e1)) { flag = false; }
                }
                if (flag) { edges.Add(new int[] {e0,e1}); }
            }
            return new CutMesh(vertices, faces, edges, dup);
        }

        //slitsをなくすが、外部境界に入ったスリットはそのまま
        public (CutMesh cutMesh, int[] vertToNoSlitVert) ConvertToNoSlitCutMesh2(List<int> outerVerts)
        {
            List<Point3d> vertices = new List<Point3d>();
            List<List<int>> dup = new List<List<int>>();
            int[] vertOrderInDup = VertOrderInDup();
            int[] vertToNoSlitVert = new int[vertOrderInDup.Length];
            int outerCount = outerVerts.Count;
            int index = DuplicatedVertIndices.FindIndex(sub => !sub.Any(v => outerVerts.Contains(v)));
            if (index == -1) {int vertCount = Vertices.Count;  return (this, Enumerable.Range(0, vertCount).ToArray()); }
            int add = outerCount - index;
            for (int i = 0; i < vertOrderInDup.Length; i++)
            {
                if (i < outerCount) { vertToNoSlitVert[i] = i; }
                else
                {
                    vertToNoSlitVert[i] = vertOrderInDup[i] + add;
                }
            }
            int[,] faces = new int[Faces.GetLength(0), 3];

            for (int i = 0; i < outerVerts.Count; i++)
            {
                vertices.Add(Vertices[outerVerts[i]]);
            }
            for (int i = index; i < DuplicatedVertIndices.Count; i++)
            {
                vertices.Add(Vertices[DuplicatedVertIndices[i][0]]);
            }
            for (int i = 0; i < Faces.GetLength(0); i++)
            {
                faces[i, 0] = vertToNoSlitVert[Faces[i, 0]];
                faces[i, 1] = vertToNoSlitVert[Faces[i, 1]];
                faces[i, 2] = vertToNoSlitVert[Faces[i, 2]];
            }
            List<int[]> edges = new List<int[]>();
            for (int i = 0; i < Edges.Count; i++)
            {
                int e0 = vertToNoSlitVert[Edges[i][0]];
                int e1 = vertToNoSlitVert[Edges[i][1]];
                bool flag = true;
                foreach (int[] edge in edges)
                {
                    if ((edge[0] == e0 && edge[1] == e1) || (edge[1] == e0 && edge[0] == e1)) { flag = false; }
                }
                if (flag) { edges.Add(new int[] { e0, e1 }); }
            }

            for (int i = 0; i < DuplicatedVertIndices.Count; i++)
            {
                if (DuplicatedVertIndices[i].Count > 1 && outerVerts.Contains(DuplicatedVertIndices[i][0]))
                {
                    List<int> dup1 = new List<int>();
                    foreach (int vert in DuplicatedVertIndices[i])
                    {
                        dup1.Add(vertToNoSlitVert[vert]);
                    }
                    dup.Add(dup1);
                }
                else { dup.Add(new List<int> { vertToNoSlitVert[DuplicatedVertIndices[i][0]] }); }
            }

            return (new CutMesh(vertices, faces, edges, dup), vertToNoSlitVert);
        }

        public List<BoundaryLoop> GetBoundaryLoops()
        {
            // 1. 境界エッジの抽出（LINQを使わず、キャッシュを直接参照）
            var boundaryEdgeIndices = new List<int>();
            foreach (var entry in EdgeToFacesCache)
            {
                if (entry.Value.Count == 1) // 境界エッジ判定
                {
                    boundaryEdgeIndices.Add(entry.Key);
                }
            }

            HashSet<int> unusedEdges = new HashSet<int>(boundaryEdgeIndices);
            List<BoundaryLoop> loops = new List<BoundaryLoop>();

            // 2. 境界頂点からエッジへの高速逆引きマップ
            // 境界エッジの総数は全エッジより圧倒的に少ないため、ここで絞り込む
            var vertToBoundaryEdges = new Dictionary<int, List<int>>();
            foreach (int ei in boundaryEdgeIndices)
            {
                int[] e = Edges[ei];
                if (!vertToBoundaryEdges.ContainsKey(e[0])) vertToBoundaryEdges[e[0]] = new List<int>();
                if (!vertToBoundaryEdges.ContainsKey(e[1])) vertToBoundaryEdges[e[1]] = new List<int>();
                vertToBoundaryEdges[e[0]].Add(ei);
                vertToBoundaryEdges[e[1]].Add(ei);
            }

            while (unusedEdges.Count > 0)
            {
                // HashSetから1つ取り出す
                var enumerator = unusedEdges.GetEnumerator();
                enumerator.MoveNext();
                int startEdgeIdx = enumerator.Current;
                unusedEdges.Remove(startEdgeIdx);

                var loopEdges = new List<int> { startEdgeIdx };
                var loopVerts = new List<int> { Edges[startEdgeIdx][0], Edges[startEdgeIdx][1] };

                // 前方向に追跡
                while (true)
                {
                    int currentVert = loopVerts[loopVerts.Count - 1];
                    int nextEdge = -1;

                    if (vertToBoundaryEdges.TryGetValue(currentVert, out var connected))
                    {
                        foreach (int ei in connected)
                        {
                            if (unusedEdges.Contains(ei))
                            {
                                nextEdge = ei;
                                break;
                            }
                        }
                    }

                    if (nextEdge == -1) break; // 次のエッジが見つからない

                    unusedEdges.Remove(nextEdge);
                    loopEdges.Add(nextEdge);
                    int vNext = (Edges[nextEdge][0] == currentVert) ? Edges[nextEdge][1] : Edges[nextEdge][0];
                    loopVerts.Add(vNext);
                }

                // 閉じたループの判定と微調整
                loopVerts.RemoveAt(loopVerts.Count - 1);

                loops.Add(new BoundaryLoop
                {
                    VertexIndices = loopVerts,
                    EdgeIndices = loopEdges
                });
            }

            return loops;
        }
    }


    public struct BoundaryLoop
    {
        public List<int> VertexIndices;
        public List<int> EdgeIndices;
    }





    public class GH_CutMesh : GH_Goo<CutMesh>, IGH_PreviewData
    {
        public GH_CutMesh() : base() { }
        public GH_CutMesh(CutMesh cutMesh) : base(cutMesh) { }
        public GH_CutMesh(Rhino.Geometry.Mesh mesh) : base(new CutMesh(mesh)) { }

        public override IGH_Goo Duplicate()
        {
            return new GH_CutMesh(Value.Clone());
        }


        public override string ToString()
        {
            return $"CutMesh {Value.Vertices.Count} Vertices, {Value.Faces.GetLength(0)} Faces, {Value.Edges.Count} Edges";
        }

        public override bool IsValid => Value != null;

        public override string IsValidWhyNot => IsValid ? string.Empty : "Invalid CutMesh";

        public override string TypeName => "CutMesh";

        public override string TypeDescription => "Custom data type for CutMesh";

        public override bool CastFrom(object source)
        {
            if (source is CutMesh cutMesh)
            {
                Value = cutMesh;
                return true;
            }
            return false;
        }

        public override bool CastTo<Q>(ref Q target)
        {
            if (typeof(Q).IsAssignableFrom(typeof(CutMesh)))
            {
                target = (Q)(object)Value;
                return true;
            }

            target = default;
            return false;
        }

        public BoundingBox ClippingBox => new BoundingBox(Value.Vertices);

        public void DrawViewportMeshes(GH_PreviewMeshArgs args)
        {
            if (Value == null || Value.Vertices == null || Value.Faces == null)
                return;

            var mesh = new Rhino.Geometry.Mesh();
            foreach (var pt in Value.Vertices)
                mesh.Vertices.Add(pt);

            int faceCount = Value.Faces.GetLength(0);
            int faceWidth = Value.Faces.GetLength(1);

            for (int i = 0; i < faceCount; i++)
            {
                    mesh.Faces.AddFace(Value.Faces[i, 0], Value.Faces[i, 1], Value.Faces[i, 2]);
            }

            mesh.Normals.ComputeNormals();
            mesh.Compact();

            // 🎨 フェイス描画（args.Material の色が使われる）
            args.Pipeline.DrawMeshShaded(mesh, args.Material);
            //args.Pipeline.DrawMeshShaded(mesh, args.Material);
        }

        public void DrawViewportWires(GH_PreviewWireArgs args)
        {
            if (Value == null || Value.Vertices == null || Value.Edges == null)
                return;

            var vertices = Value.Vertices;
            var edges = Value.Edges;

            // 🎨 エッジ色を選択色ベースで暗くする（フェイスと区別）
            var baseColor = args.Color;
            var edgeColor = System.Drawing.Color.FromArgb(
                baseColor.A,
                Math.Max(0, baseColor.R - 60),
                Math.Max(0, baseColor.G - 60),
                Math.Max(0, baseColor.B - 60)
            );

            for (int i = 0; i < edges.Count; i++)
            {
                var e = edges[i];
                var line = new Line(vertices[e[0]], vertices[e[1]]);

                // ★ 変更ポイント：エッジに接するFace数で線の太さを決定
                int faceCount = Value.GetFacesForEdge(i).Count;
                int thickness = (faceCount >= 2) ? 1 : 3;

                args.Pipeline.DrawLine(line, edgeColor, thickness);
            }
        }
    }
}
