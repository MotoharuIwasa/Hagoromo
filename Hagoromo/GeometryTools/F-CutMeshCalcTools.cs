using Grasshopper.Kernel;
using Grasshopper.Kernel.Geometry;
using Hagoromo.MathTools;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Collections;
using Rhino.Render.ChangeQueue;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Hagoromo.GeometryTools
{
    public static class CutMeshCalcTools
    {
        //interiorAngles[i][j]はi番目の点の周りのGetOrderedFacesAroundVertexの順でj番目のfaceにおける内角
        public static List<List<double>> InteriorAngles(CutMesh mesh)
        {
            List<List<double>> angles = new List<List<double>>();
            for (int i = 0; i < mesh.Vertices.Count; i++)
            {
                List<int> faces = mesh.GetFacesForVertex(i);
                List<double> angleList = new List<double>();
                for (int j = 0; j < faces.Count; j++)
                {
                    List<int> a = new List<int> { mesh.Faces[faces[j], 0], mesh.Faces[faces[j], 1], mesh.Faces[faces[j], 2] };

                    List<int> points = a.Where(x => x != i).ToList();
                    Point3d A = mesh.Vertices[i];
                    Vector3d AB = mesh.Vertices[points[0]] - A;
                    Vector3d AC = mesh.Vertices[points[1]] - A;
                    double angle = Vector3d.VectorAngle(AB, AC); // ラジアンで返る
                    angleList.Add(angle);
                }
                angles.Add(angleList);
            }
            return angles;
        }

        //ガウス曲率のリスト、境界上の点ではガウス曲率0とする。
        public static double[] GaussianCurvature(CutMesh mesh)
        {
            List<int> boundaryVertIndices = mesh.BoundaryVertIndices();
            List<List<double>> angles = InteriorAngles(mesh);
            double[] gaussian = new double[mesh.Vertices.Count];
            for (int i = 0; i < mesh.Vertices.Count; i++)
            {
                if (boundaryVertIndices.Contains(i))
                {
                    continue;
                }
                else
                {
                    List<double> angle = angles[i];
                    double a = 0;
                    for (int j = 0; j < angle.Count; j++)
                    {
                        a += angle[j];
                    }
                    gaussian[i] = 2 * Math.PI - a;
                }
            }
            return gaussian;
        }



        //-----------------------------------------------------------主曲率用----------------------------------------------------
        //faceのインデックスの順でfaceNormalVectorを求めるための準備、[i]には、{faceID, 外積の最初のエッジ、次のエッジ}が入っている。
        public static List<List<int>> FaceNormalOrders(CutMesh mesh)
        {
            int faceCount = mesh.Faces.GetLength(0);

            List<List<int>> faceRecord = new List<List<int>>();
            int[] firstFaceEdges = mesh.GetEdgesForFace(0);
            faceRecord.Add(new List<int> { 0, firstFaceEdges[0], firstFaceEdges[1] });

            List<int> doneFaces = new List<int> { 0 };
            List<int> currentFaces = new List<int> { 0 };

            while (currentFaces.Count > 0)
            {
                List<List<int>> faceOrders = new List<List<int>>();
                List<int> nextFaces = new List<int>();

                for (int i = faceRecord.Count-currentFaces.Count; i < faceRecord.Count; i++)
                {
                    int faceOne = mesh.GetFacesForEdge(faceRecord[i][1]).Where(f => f != faceRecord[i][0]).Cast<int?>().FirstOrDefault() ?? -1;
                    int faceTwo = mesh.GetFacesForEdge(faceRecord[i][2]).Where(f => f != faceRecord[i][0]).Cast<int?>().FirstOrDefault() ?? -1;
                    int edge = mesh.GetEdgesForFace(faceRecord[i][0]).First(f => ((f != faceRecord[i][1]) && (f != faceRecord[i][2])));
                    int faceThree = mesh.GetFacesForEdge(edge).Where(f => f != faceRecord[i][0]).Cast<int?>().FirstOrDefault() ?? -1;
                    if (doneFaces.Contains(faceOne)) { faceOne = -1; }
                    if (doneFaces.Contains(faceTwo)) { faceTwo = -1; }
                    if (doneFaces.Contains(faceThree)) { faceThree = -1; }

                    int[] vertices1 = mesh.Edges[faceRecord[i][1]];
                    int[] vertices2 = mesh.Edges[faceRecord[i][2]];
                    int commonVertex = vertices1.Intersect(vertices2).FirstOrDefault();

                    if (faceOne != -1)
                    {
                        nextFaces.Add(faceOne);
                        doneFaces.Add(faceOne);
                        int edge1 = (mesh.GetEdgesForFace(faceOne).Intersect(mesh.GetEdgesForVertex(commonVertex))).First(f => f != faceRecord[i][1]);
                        faceOrders.Add(new List<int> { faceOne, edge1, faceRecord[i][1] });
                    }

                    if (faceTwo != -1)
                    {
                        nextFaces.Add(faceTwo);
                        doneFaces.Add(faceTwo);
                        int edge2 = (mesh.GetEdgesForFace(faceTwo).Intersect(mesh.GetEdgesForVertex(commonVertex))).First(f => f != faceRecord[i][2]);
                        faceOrders.Add(new List<int> { faceTwo, faceRecord[i][2], edge2 });
                    }

                    if (faceThree  != -1)
                    {
                        nextFaces.Add(faceThree);
                        doneFaces.Add(faceThree);
                        int v1 = vertices1.First(f => f != commonVertex);
                        int v2 = vertices2.First(f => f != commonVertex);
                        int edge3 = mesh.GetEdgesForFace(faceThree).Intersect(mesh.GetEdgesForVertex(v2)).First(f => f != edge);
                        int edge4 = mesh.GetEdgesForFace(faceThree).Intersect(mesh.GetEdgesForVertex(v1)).First(f => f != edge);
                        faceOrders.Add(new List<int> { faceThree, edge3, edge4 });
                    }
                }
                faceRecord.AddRange(faceOrders);
                currentFaces = new List<int>(nextFaces);
            }
            faceRecord.Sort((a, b) => a[0].CompareTo(b[0]));
            return faceRecord;
        }

        //face間で向きをそろえた外積の値
        public static Vector3d[] FaceNormal(CutMesh mesh)
        {
            List<List<int>> faceRecord = FaceNormalOrders(mesh);
            Vector3d[] normals = new Vector3d[faceRecord.Count];
            for (int i = 0; i < faceRecord.Count; i++)
            {
                int[] edge1 = mesh.Edges[faceRecord[i][1]];
                int[] edge2 = mesh.Edges[faceRecord[i][2]];
                int v0index = edge1.Intersect(edge2).First();
                Point3d v0 = mesh.Vertices[v0index];
                Point3d v1 = mesh.Vertices[edge1.First(f => f != v0index)];
                Point3d v2 = mesh.Vertices[edge2.First(f => f != v0index)];
                Vector3d vec1 = v1 - v0;
                Vector3d vec2 = v2 - v0;
                Vector3d normal = Vector3d.CrossProduct(vec1, vec2);
                normals[i] = normal;
            }
            return normals;
        }

        //重み付きで各節点の法線ベクトルを求める、単位ベクトル
        public static Vector3d[] VertexNormal(CutMesh mesh)
        {
            Vector3d[] normals = new Vector3d[mesh.Vertices.Count];
            Vector3d[] faceNormal = FaceNormal(mesh);
            for(int i = 0;i < mesh.Vertices.Count; i++)
            {
                Vector3d sum = Vector3d.Zero;
                List<int> faces = mesh.GetFacesForVertex(i);
                foreach (int face in faces)
                {
                    List<int> edges = mesh.GetEdgesForVertex(i).Intersect(mesh.GetEdgesForFace(face)).ToList();
                    double l1 = mesh.GetEdgeLine(edges[0]).Length;
                    double l2 = mesh.GetEdgeLine(edges[0]).Length;
                    sum += faceNormal[face] / (l1 * l1 * l2 * l2);
                }
                sum.Unitize();
                normals[i] = sum;
            }
            return normals;
        }

        public static Vector3d[,] FaceBase(CutMesh mesh, Vector3d[] faceNormals)
        {
            Vector3d[,] faceBase = new Vector3d[mesh.Faces.GetLength(0), 2];
            for (int i = 0; i < mesh.Faces.GetLength(0); i++)
            {
                int v0index = mesh.Faces[i, 0];
                int v1index = mesh.Faces[i, 1];
                int v2index = mesh.Faces[i, 2];

                Vector3d e0 = mesh.Vertices[v2index] - mesh.Vertices[v1index];
                Vector3d e1 = mesh.Vertices[v0index] - mesh.Vertices[v2index];
                Vector3d e2 = mesh.Vertices[v1index] - mesh.Vertices[v0index];

                Vector3d u = new Vector3d(e0);
                u.Unitize();
                Vector3d n = faceNormals[i];
                Vector3d v = Vector3d.CrossProduct(n, u);
                v.Unitize();
                faceBase[i,0] = u;
                faceBase[i,1] = v;
            }
            return faceBase;
        }

        //主曲率の求め方step2のfaceのテンソルπを求める
        public static double[,] FaceCurvatureTensor(CutMesh mesh, Vector3d[] faceNormals, Vector3d[] vertexNormals, Vector3d[,] faceBase)
        {
            double[,] tensor = new double[mesh.Faces.GetLength(0),3];
            //Vector3d[,] faceBase = FaceBase(mesh, faceNormals);
            for (int i = 0; i < mesh.Faces.GetLength(0); i++)
            {
                int v0index = mesh.Faces[i, 0];
                int v1index = mesh.Faces[i, 1];
                int v2index = mesh.Faces[i, 2];

                Vector3d e0 = mesh.Vertices[v2index] - mesh.Vertices[v1index];
                Vector3d e1 = mesh.Vertices[v0index] - mesh.Vertices[v2index];
                Vector3d e2 = mesh.Vertices[v1index] - mesh.Vertices[v0index];

                Vector3d u = faceBase[i, 0];
                Vector3d v = faceBase[i, 1];

                Vector3d delta0 = vertexNormals[v2index] - vertexNormals[v1index];
                Vector3d delta1 = vertexNormals[v0index] - vertexNormals[v2index];
                Vector3d delta2 = vertexNormals[v1index] - vertexNormals[v0index];

                double[] b = new double[6] { delta0 * u, delta0 * v, delta1 * u, delta1 * v, delta2 * u, delta2 * v };
                double[,] Atrans = new double[3, 6]
                {
                    {e0 * u, 0     , e1 * u, 0     , e2 * u, 0      },
                    {e0 * v, e0 * u, e1 * v, e1 * u, e2 * v, e2 * u },
                    {0     , e0 * v, 0     , e1 * v, 0     , e2 * v }
                };
                double[] x = MatrixUtils.Multiply(MatrixUtils.Multiply(MatrixUtils.InverseMatrix(MatrixUtils.Multiply(Atrans, MatrixUtils.Transpose(Atrans))), Atrans), b);
                tensor[i, 0] = x[0];
                tensor[i, 1] = x[1];
                tensor[i, 2] = x[2];
            }
            return tensor;
        }

        //各頂点の法線ベクトルに垂直な正規直交基底ベクトルを作る
        public static Vector3d[,] VertexBase(CutMesh mesh, Vector3d[] vertexNormals)
        {
            Vector3d[,] vertexBase = new Vector3d[vertexNormals.Length, 2];
            for (int i = 0; i < vertexNormals.Length; i++)
            {
                Vector3d vertexNormal = vertexNormals[i];
                Vector3d zvec = new Vector3d(0, 0, 1);
                Vector3d u = Vector3d.CrossProduct(zvec, vertexNormal);
                if (u.Length < 1e-4)
                {
                    Vector3d xvec = new Vector3d(1, 0, 0);
                    u = Vector3d.CrossProduct(xvec, vertexNormal);
                }
                u.Unitize();
                Vector3d v = Vector3d.CrossProduct(vertexNormal, u);
                v.Unitize();
                vertexBase[i, 0] = u;
                vertexBase[i, 1] = v;
            }
            return vertexBase;
        }

        //[i][j]は[i]番目の頂点周りのGetFacesForVertexのFaceの順でのj番目のfaceのボロノイ面積
        public static double[][] VoronoiArea(CutMesh mesh)
        {
            double[][] voronoi = new double[mesh.Vertices.Count][];
            for (int i = 0; i < mesh.Vertices.Count; i++)
            {
                voronoi[i] = new double[mesh.GetFacesForVertex(i).Count];
            }
            for (int i = 0;i < mesh.Faces.GetLength(0); i++)
            {
                int v0index = mesh.Faces[i, 0];
                int v1index = mesh.Faces[i, 1];
                int v2index = mesh.Faces[i, 2];
                Vector3d e1 = mesh.Vertices[v2index] - mesh.Vertices[v0index];
                Vector3d e2 = mesh.Vertices[v1index] - mesh.Vertices[v0index];
                Vector3d e0 = e1 - e2;
                double area = Vector3d.CrossProduct(e1, e2).Length * 0.5;
                double r = 2 * area / (e0.Length + e1.Length + e2.Length);
                Vector3d tvec = e2 - (e1 * e2 / (e1 * e1)) * e1;
                Vector3d svec = e1 - (e1 * e2 / (e2 * e2)) * e2;
                double s = r / svec.Length;
                double t = r / tvec.Length;
                Vector3d b = s * e1 + t * e2;
                double a = Math.Sqrt(b.SquareLength - r * r);
                double v0area = a * r;
                double f = e1.Length - a;
                double v2area = f * r;
                double v1area = area - (v0area + v2area);
                voronoi[v0index][mesh.GetFacesForVertex(v0index).IndexOf(i)] = v0area;
                voronoi[v1index][mesh.GetFacesForVertex(v1index).IndexOf(i)] = v1area;
                voronoi[v2index][mesh.GetFacesForVertex(v2index).IndexOf(i)] = v2area;
            }
            return voronoi;
        }

        //主曲率の求め方step3の各節点のテンソルπを求める
        public static double[,] VertexCurvatureTensor(CutMesh mesh, Vector3d[] faceNormals, Vector3d[] vertexNormals, Vector3d[,] faceBase, double[][] voronoi)
        {
            Vector3d[,] vertexBase = VertexBase(mesh, vertexNormals);
            double[,] faceCurvatureTensor = FaceCurvatureTensor(mesh, faceNormals, vertexNormals, faceBase);
            double[,] vertexCurvatureTensor = new double[mesh.Vertices.Count, 3];
            for (int i = 0; i < mesh.Vertices.Count; i++)
            {
                Vector3d np = vertexNormals[i];
                List<int> faces = mesh.GetFacesForVertex(i);
                double voronoiSum = 0;
                for (int j  = 0; j < faces.Count; j++)
                {
                    voronoiSum += voronoi[i][j];
                }
                int ii = 0;
                foreach (int face in faces)
                {
                    Vector3d nf = faceNormals[face];
                    nf.Unitize();
                    Vector3d k = Vector3d.CrossProduct(nf, np);
                    double sin = k.Length;
                    k.Unitize();
                    double cos = nf * np;
                    double[,] K = new double[3, 3] { { 0, -k.Z, k.Y }, { k.Z, 0, -k.X }, { -k.Y, k.X, 0 } };

                    double[,] R = MatrixUtils.Add(MatrixUtils.Multiply(sin,K),MatrixUtils.Multiply(1-cos,MatrixUtils.Multiply(K, K)));
                    R[0,0] += 1;
                    R[1,1] += 1;
                    R[2,2] += 1;
                    Vector3d uf2 = MatrixUtils.Multiply(R, faceBase[face, 0]);
                    Vector3d vf2 = MatrixUtils.Multiply(R, faceBase[face, 1]);

                    double Cuu = vertexBase[i, 0] * uf2;
                    double Cuv = vertexBase[i, 0] * vf2;
                    double Cvu = vertexBase[i, 1] * uf2;
                    double Cvv = vertexBase[i, 1] * vf2;

                    double af = faceCurvatureTensor[face, 0];
                    double bf = faceCurvatureTensor[face, 1];
                    double cf = faceCurvatureTensor[face, 2];

                    vertexCurvatureTensor[i, 0] += voronoi[i][ii] * (af*Cuu*Cuu+2*bf*Cuu*Cuv+cf*Cuv*Cuv) / voronoiSum;
                    vertexCurvatureTensor[i, 1] += voronoi[i][ii] * (af*Cuu*Cvu+bf*(Cuu*Cvv+Cuv*Cvu)+cf*Cuv*Cvv) / voronoiSum;
                    vertexCurvatureTensor[i, 2] += voronoi[i][ii] * (af*Cvu*Cvu+2*bf*Cvu*Cvv+cf*Cvv*Cvv) / voronoiSum;
                    ii += 1;
                }
            }
            return vertexCurvatureTensor;
        }

        //各頂点の主曲率２つを求める
        public static double[,] PrincipleCurvature(CutMesh mesh)
        {
            double[,] principle = new double[mesh.Vertices.Count, 2];
            Vector3d[] faceNormals = FaceNormal(mesh);
            Vector3d[] vertexNormals = VertexNormal(mesh);
            Vector3d[,] faceBase = FaceBase(mesh, faceNormals);
            double[][] voronoi = VoronoiArea(mesh);
            double[,] vertexCurvatureTensor = VertexCurvatureTensor(mesh, faceNormals, vertexNormals, faceBase, voronoi);
            for (int i = 0; i < mesh.Vertices.Count; i++)
            {
                double apcp = vertexCurvatureTensor[i, 0] + vertexCurvatureTensor[i, 2];
                double bp = vertexCurvatureTensor[i, 1];
                principle[i, 0] = (apcp + Math.Sqrt(apcp * apcp + 4 * bp * bp)) * 0.5;
                principle[i, 1] = (apcp - Math.Sqrt(apcp * apcp + 4 * bp * bp)) * 0.5;
            }
            return principle;
        }
        //-----------------------------------------------------------主曲率用done----------------------------------------------------


        /*
        public static double[] PrincipalAbsMaxCurvature(CutMesh mesh)
        {
            double[] curvature = new double[mesh.Vertices.Count];
            for (int i = 0;i < mesh.Vertices.Count; i++)
            {
                List<int> faces = mesh.GetFacesForVertex(i);
            }
        }

        
        public static List<Vector3d> FaceNormal(CutMesh mesh)
        {
            List<Vector3d> normals = new List<Vector3d>();
            List<Point3d> centers = new List<Point3d>();
            for (int i = 0; i <= mesh.Faces.GetLength(0); i++)
            { 
                centers.Add(FaceCenter(mesh,i));
            }
            Point3d center = new Point3d(centers.Average(p => p.X), centers.Average(p => p.Y), centers.Average(p => p.Z));


            for (int i = 0; i <= mesh.Faces.GetLength(0); i++)
            {
                Point3d v0 = mesh.Vertices[mesh.Faces[i, 0]];
                Point3d v1 = mesh.Vertices[mesh.Faces[i, 1]];
                Point3d v2 = mesh.Vertices[mesh.Faces[i, 2]];
                Vector3d vec1 = v1 - v0;
                Vector3d vec2 = v2 - v0;
                Vector3d normal = Vector3d.CrossProduct(vec1, vec2);
                normal.Unitize();
            }
        }
        */

        public static Point3d FaceCenter(CutMesh mesh, int i)
        {
            Point3d v0 = mesh.Vertices[mesh.Faces[i, 0]];
            Point3d v1 = mesh.Vertices[mesh.Faces[i, 1]];
            Point3d v2 = mesh.Vertices[mesh.Faces[i, 2]];
            Point3d center = (v0 + v1 + v2) / 3;
            return center;
        }




        //測地曲率のリスト、境界上のみ定義する。BoundaryVertIndicesの順
        public static double[] GeodesicCurvature(CutMesh mesh)
        {
            List<int> boundaryVertIndices = mesh.BoundaryVertIndices();
            List<List<double>> angles = InteriorAngles(mesh);
            double[] geodesic = new double[boundaryVertIndices.Count];
            for (int i = 0; i < boundaryVertIndices.Count; i++)
            {
                double a = 0;
                List<double> angle = angles[boundaryVertIndices[i]];
                for (int j = 0; j < angle.Count; j++)
                {
                    a += angle[j];
                }
                geodesic[i] = Math.PI - a;
            }
            return geodesic;
        }

        //ある点周りの角度をすべて足した値を返す
        public static double[] AngleSum(CutMesh mesh)
        {
            List<List<double>> angles = InteriorAngles(mesh);
            double[] angleSum = new double[mesh.Vertices.Count];
            for (int i = 0; i < mesh.Vertices.Count; i++)
            {
                List<double> angle = angles[i];
                double a = 0;
                for (int j = 0; j < angle.Count; j++)
                {
                    a += angle[j];
                }
                angleSum[i] = a;
            }
            return angleSum;
        }

        //[i][j][k]はi番目の内部節点の周りのfacesのうちj番目のfaceのその節点以外の2点のうちk番目のindex
        public static int[][][] MakeInternalFaceID(CutMesh mesh)
        {
            List<int> boundaryVertIndices = mesh.BoundaryVertIndices();
            List<int> fullSet = Enumerable.Range(0, mesh.Vertices.Count).ToList();
            List<int> internalVertIndices = fullSet.Except(boundaryVertIndices).ToList();

            int[][][] faceID = new int[internalVertIndices.Count][][];
            for (int i = 0; i < internalVertIndices.Count; i++)
            {
                List<int> facesForVertex = mesh.GetFacesForVertex(internalVertIndices[i]);
                faceID[i] = new int[facesForVertex.Count][];
                for (int j =0; j < facesForVertex.Count; j++)
                {
                    faceID[i][j] = new int[2];
                    int[] facesVertices = new int[] { mesh.Faces[facesForVertex[j], 0], mesh.Faces[facesForVertex[j], 1], mesh.Faces[facesForVertex[j], 2] };
                    int[] twoVertices = facesVertices.Where(v => v != internalVertIndices[i]).ToArray();
                    faceID[i][j][0] = twoVertices[0];
                    faceID[i][j][1] = twoVertices[1];
                }
            }
            return faceID;
        }

        //節点番号順にたどったときのエッジ群を返す
        public static List<int> PolyEdge(CutMesh mesh, List<int> vertIndices, bool IsClosed)
        {
            List<int> polyEdge = new List<int>();
            for (int i = 0;i < vertIndices.Count-1;i++)
            {
                polyEdge.Add(mesh.GetEdgeForEndPoints(vertIndices[i], vertIndices[i+1]));
            }
            if (IsClosed) { polyEdge.Add(mesh.GetEdgeForEndPoints(vertIndices[0], vertIndices[vertIndices.Count-1])); }
            return polyEdge;
        }
    }
}