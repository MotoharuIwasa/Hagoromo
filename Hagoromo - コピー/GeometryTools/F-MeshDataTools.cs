using Grasshopper.Kernel;
using Grasshopper.Kernel.Geometry.Delaunay;
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
    public static class MeshDataTools
    {
        //TopoVerticesはfloat型なのでそれを計算中はdoubleで扱えるように変換しておく。よくnewTopoVerticesという変数で置いている。
        public static Point3d[] DoubleTopoVertices(Rhino.Geometry.Mesh mesh)
        {
            MeshTopologyVertexList topoVertices = mesh.TopologyVertices;
            Point3d[] newTopoVertices = new Point3d[topoVertices.Count];

            //TopoVerticesがfloatなのでdoubleにしたnewTopoVerticesで考える
            for (int i = 0; i < topoVertices.Count; i++)
            {
                newTopoVertices[i] = topoVertices[i];
            }
            return newTopoVertices;
        }

        //TriFaceIndices[i][j][k]はTopoVerticesのi番目の点を含むj番目の面のほかの2点のうちの1点のTopoVerticesでのindex
        public static int[][][] TriFaceIndices(Rhino.Geometry.Mesh mesh)
        {
            int count = mesh.TopologyVertices.Count;
            int[][] faces = new int[count][];
            int[][][] TriFaceID = new int[count][][];
            for (int i = 0; i < count; i++)
            {
                faces[i] = mesh.TopologyVertices.ConnectedFaces(i);
                TriFaceID[i] = new int[faces[i].Length][];
                int t = 0;
                foreach (int fi in faces[i])
                {
                    MeshFace face = mesh.Faces[fi];

                    // メッシュ頂点のID → トポロジー頂点IDに変換
                    List<int> topoIndices = new List<int>
                    {
                        mesh.TopologyVertices.TopologyVertexIndex(face.A),
                        mesh.TopologyVertices.TopologyVertexIndex(face.B),
                        mesh.TopologyVertices.TopologyVertexIndex(face.C)
                    };

                    int vertPos = topoIndices.IndexOf(i);

                    int i1 = topoIndices[(vertPos - 1 + topoIndices.Count) % topoIndices.Count];
                    int i2 = topoIndices[(vertPos + 1) % topoIndices.Count];

                    TriFaceID[i][t] = new int[2];
                    TriFaceID[i][t][0] = i1;
                    TriFaceID[i][t][1] = i2;
                    t += 1;
                }
            }
            return TriFaceID;
        }

        //各faceの節点のTopologyVerticesにおけるindexを得る。
        public static int[][] FaceTopoVertIndices(Rhino.Geometry.Mesh mesh)
        {
            int[][] faceTopoVertices = new int[mesh.Faces.Count][];
            for (int i = 0; i < mesh.Faces.Count; i++)
            {
                faceTopoVertices[i] = new int[3];
                MeshFace face = mesh.Faces[i];

                // 各頂点のインデックス（ジオメトリ的）
                int vi0 = face.A;
                int vi1 = face.B;
                int vi2 = face.C;

                // トポロジー頂点インデックスに変換
                int tvi0 = mesh.TopologyVertices.TopologyVertexIndex(vi0);
                int tvi1 = mesh.TopologyVertices.TopologyVertexIndex(vi1);
                int tvi2 = mesh.TopologyVertices.TopologyVertexIndex(vi2);
                faceTopoVertices[i][0] = tvi0;
                faceTopoVertices[i][1] = tvi1;
                faceTopoVertices[i][2] = tvi2;
            }
            return faceTopoVertices;
        }

        //元のメッシュとトポロジカル的には同じで、各ノードの位置が異なるメッシュを作成
        public static Rhino.Geometry.Mesh MakeMesh(Rhino.Geometry.Mesh mesh, Point3d[] newTopoVertices)
        {
            Rhino.Geometry.Mesh newMesh = new Rhino.Geometry.Mesh();
            //まずTopologyVerticesからVerticesを作成
            Point3f[] newVertices = new Point3f[mesh.Vertices.Count];
            for (int i = 0; i < mesh.TopologyVertices.Count; i++)
            {
                Point3d pt = newTopoVertices[i];
                int[] vertexIndices = mesh.TopologyVertices.MeshVertexIndices(i);

                for (int j = 0; j < vertexIndices.Length; j++)
                {
                    newVertices[vertexIndices[j]] = new Point3f((float)pt.X, (float)pt.Y, (float)pt.Z);
                }
            }

            // 頂点の変形
            foreach (Point3f pt in newVertices)
            {
                newMesh.Vertices.Add(pt);
            }
            // 面のコピー
            for (int i = 0; i < mesh.Faces.Count; i++)
            {
                newMesh.Faces.AddFace(mesh.Faces[i]);
            }
            // 必要なら法線を再計算
            newMesh.Normals.ComputeNormals();
            newMesh.Compact();
            return newMesh;
        }

        //meshの内部の点がそれぞれTopoVerticesの順番で何番目の点かを全部並べたリストを返す
        public static List<int> TopoInternalVertIndices(Rhino.Geometry.Mesh mesh)
        {
            List<int> internalVertexIndices = new List<int>();

            for (int i = 0; i < mesh.TopologyVertices.Count; i++)
            {
                bool isNaked = false;


                // その TopologyVertex に接続している Edge を確認
                int[] edgeIndices = mesh.TopologyVertices.ConnectedEdges(i);
                foreach (int ei in edgeIndices)
                {
                    // このエッジに接続している面を取得
                    int[] faceIndices = mesh.TopologyEdges.GetConnectedFaces(ei);
                    if (faceIndices.Length == 1)
                    {
                        isNaked = true;
                        break;
                    }
                }

                // naked でない頂点インデックスだけ追加
                if (!isNaked)
                {
                    internalVertexIndices.Add(i);
                }
            }
            return internalVertexIndices;
        }

        //ConnectedVertices[i]はTopoVerticesのi番目の点に隣接する点たち
        public static int[][] ConnectedVertices(Rhino.Geometry.Mesh mesh)
        {
            int count = mesh.TopologyVertices.Count;
            List<int[]> connectedVerticesList = new List<int[]>();
            for (int i = 0; i < count; i++)
            {
                connectedVerticesList.Add(mesh.TopologyVertices.ConnectedTopologyVertices(i));
            }
            int[][] connectedVertices = connectedVerticesList.ToArray();
            return connectedVertices;
        }

        //ConnectedFaces(mesh)[i]にはi番目のfaceと隣接するfaceたちのindexが格納
        public static int[][] ConnectedFaces(Rhino.Geometry.Mesh mesh)
        {
            int faceCount = mesh.Faces.Count;
            List<int>[] adjacency = new List<int>[faceCount];
            for (int i = 0; i < faceCount; i++)
                adjacency[i] = new List<int>();

            MeshTopologyEdgeList topoEdges = mesh.TopologyEdges;

            for (int e = 0; e < topoEdges.Count; e++)
            {
                int[] connectedFaces = topoEdges.GetConnectedFaces(e);

                if (connectedFaces.Length == 2)
                {
                    int f0 = connectedFaces[0];
                    int f1 = connectedFaces[1];

                    if (!adjacency[f0].Contains(f1))
                        adjacency[f0].Add(f1);

                    if (!adjacency[f1].Contains(f0))
                        adjacency[f1].Add(f0);
                }
            }

            int[][] result = new int[faceCount][];
            for (int i = 0; i < faceCount; i++)
                result[i] = adjacency[i].ToArray();

            return result;
        }

        //ある頂点を含むフェイスを反時計回りに並べる。境界上の点だったら境界の端から順に並ぶ
        public static List<int> GetOrderedFacesAroundVertex(Rhino.Geometry.Mesh mesh, int topoVertexIndex)
        {
            var tv = mesh.TopologyVertices;
            Point3d v = tv[topoVertexIndex];

            int[] connectedFaces = tv.ConnectedFaces(topoVertexIndex);
            if (connectedFaces == null || connectedFaces.Length == 0)
                return new List<int>();

            // 頂点を基準にフェイス重心の角度を計算
            var faceAngles = new List<(int faceIndex, double angle)>();
            foreach (int fi in connectedFaces)
            {
                Point3d fc = mesh.Faces.GetFaceCenter(fi);
                Vector3d dir = fc - v;
                double angle = Math.Atan2(dir.Y, dir.X); // XY平面に投影して角度
                faceAngles.Add((fi, angle));
            }

            // 角度でソート（昇順 = CCW）
            faceAngles.Sort((a, b) => a.angle.CompareTo(b.angle));

            List<int> order =  faceAngles.Select(f => f.faceIndex).ToList();
            List<int> internalVertIndices = TopoInternalVertIndices(mesh);
            int count = order.Count;
            if (internalVertIndices.Contains(topoVertexIndex) || count<=2)
            {
                return order;
            }

            //点が境界上の時かつそこに3つ以上のフェイスがあるとき
            else
            {
                List<int> newOrder = new List<int>();
                for (int i = 0; i < count; i++)
                {
                    if (FaceHasBoundaryEdge(mesh, order[i]))
                    {
                        if (!(FaceHasBoundaryEdge(mesh, order[(i + 1) % count])))
                        {
                            newOrder = order.Skip(i).Concat(order.Take(i)).ToList();
                        }
                    }
                }
                return newOrder;
            }
        }

        //あるフェイスが境界を持つならtrueを返す
        public static bool FaceHasBoundaryEdge(Rhino.Geometry.Mesh mesh, int faceIndex)
        {
            var edgeList = mesh.TopologyEdges;

            /*
            // faceIndex の頂点インデックスを取得
            int a = mesh.Faces[faceIndex].A;
            int b = mesh.Faces[faceIndex].B;
            int c = mesh.Faces[faceIndex].C;
            */

            // 面を構成するトポロジーエッジを取得
            var faceEdges = edgeList.GetEdgesForFace(faceIndex); // int[] のエッジインデックス

            foreach (int ei in faceEdges)
            {
                // そのエッジが境界エッジか？
                if (edgeList.GetConnectedFaces(ei).Length == 1)
                {
                    return true; // 境界エッジあり
                }
            }

            return false; // 境界エッジなし
        }

        //meshのf1番目のfaceとf2番目のfaceが隣接していたらその共有するedgeのmesh.TopologyEdgesにおけるindexを返す
        //共有する辺がなければ-1を返す。
        public static int SharedEdgeIndex(Rhino.Geometry.Mesh mesh, int f1, int f2)
        {
            var topoEdges = mesh.TopologyEdges;
            for (int e = 0; e < topoEdges.Count; e++)
            {
                var faces = topoEdges.GetConnectedFaces(e);
                if (faces.Length == 2 && ((faces[0] == f1 && faces[1] == f2) || (faces[0] == f2 && faces[1] == f1)))
                {
                    var line = topoEdges.EdgeLine(e);
                    return e;
                }
            }
            return -1;
        }

        //TopoEdgeのedgeIndex番目の辺を持つfaceのindexたちを返す。必ず1つか2つになるはず
        public static List<int> GetFacesForEdge(Rhino.Geometry.Mesh mesh, int edgeIndex)
        {
            if (mesh == null) throw new ArgumentNullException(nameof(mesh));
            if (edgeIndex < 0 || edgeIndex >= mesh.TopologyEdges.Count) throw new ArgumentOutOfRangeException(nameof(edgeIndex));

            var faces = new List<int>();

            // このエッジの頂点インデックス
            int v0 = mesh.TopologyEdges.GetTopologyVertices(edgeIndex)[0];
            int v1 = mesh.TopologyEdges.GetTopologyVertices(edgeIndex)[1];

            // meshの全フェイスを走査
            for (int i = 0; i < mesh.Faces.Count; i++)
            {
                MeshFace f = mesh.Faces[i];
                int[] verts = f.IsQuad ? new int[] { f.A, f.B, f.C, f.D } : new int[] { f.A, f.B, f.C };

                // 両方の頂点がフェイスに含まれていれば、そのフェイスはこのエッジに属する
                if (verts.Contains(v0) && verts.Contains(v1))
                    faces.Add(i);
            }

            return faces;
        }

        //meshのedgeIndicesのうちboundary部分以外のものを返す
        public static List<int> RemoveBoundaryEdges(Rhino.Geometry.Mesh mesh, List<int> edgeIndices)
        {
            MeshTopologyEdgeList edgeList = mesh.TopologyEdges;
            List<int> result = new List<int>();

            foreach (int ei in edgeIndices)
            {
                int[] connectedFaces = edgeList.GetConnectedFaces(ei); // このエッジに接しているフェイス
                if (connectedFaces.Length > 1) // 内部エッジ（境界でなければ追加）
                {
                    result.Add(ei);
                }
            }

            return result;
        }
    }
}
