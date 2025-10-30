using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Collections;
using Rhino.Render.ChangeQueue;
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

        //内側の点のインデックス
        /*
            InternalVertIndicesは以下のように入手できる         
            List<int> fullSet = Enumerable.Range(0, cutMesh.Vertices.Count).ToList();
            List<int> InternalVertIndices = fullSet.Except(boundaryVertIndices()).ToList();
        */

        //あるfaceのとなりのfacesたち
        public List<int> ConnectedFacesForFace(int faceIndex)
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
            boundaryPart.Add(boundaryVertIndices.Count-1);
            int countcount = 0;
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
