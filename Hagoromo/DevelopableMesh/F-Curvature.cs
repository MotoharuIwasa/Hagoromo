using Grasshopper.Kernel;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Collections;
using Rhino.Render.ChangeQueue;
using System;
using System.Collections.Generic;
using System.Linq;
using Hagoromo.MathTools;
using Hagoromo.GeometryTools;

namespace Hagoromo.DevelopableMesh
{
    public static class CurvatureTools
    {

        //meshのある一点での離散ガウス曲率の2乗を算出
        public static double CurvatureTwo(Rhino.Geometry.Mesh mesh, int topoIndex)
        {
            double angleSum = 0.0;
            int[] faces = mesh.TopologyVertices.ConnectedFaces(topoIndex);

            foreach (int fi in faces)
            {
                MeshFace face = mesh.Faces[fi];
                List<int> indices = new List<int> { face.A, face.B, face.C };
                if (face.IsQuad) indices.Add(face.D);

                List<int> topoIndices = new List<int>();
                foreach (int idx in indices)
                {
                    topoIndices.Add(mesh.TopologyVertices.TopologyVertexIndex(idx));
                }

                int vertPos = topoIndices.IndexOf(topoIndex);
                if (vertPos == -1) continue;

                Point3f p0 = mesh.Vertices[indices[(vertPos - 1 + indices.Count) % indices.Count]];
                Point3f p1 = mesh.Vertices[indices[vertPos]];
                Point3f p2 = mesh.Vertices[indices[(vertPos + 1) % indices.Count]];

                Vector3d v1 = p0 - p1;
                Vector3d v2 = p2 - p1;

                double angle = Vector3d.VectorAngle(v1, v2);
                angleSum += angle;
            }
            return (2 * Math.PI - angleSum) * (2 * Math.PI - angleSum);
        }

        //最も曲率が大きい点のTopoVerticesでのindexを返す
        public static int MeshBiggestCurvatureIndex(Rhino.Geometry.Mesh mesh, int[] internalVertexIndices)
        {
            int index = 0;
            double curvature = 0;
            double biggest = 0;
            for (int i = 0; i < internalVertexIndices.Length; i++)
            {
                curvature = CurvatureTwo(mesh, internalVertexIndices[i]);
                if (curvature > biggest)
                {
                    biggest = curvature;
                    index = internalVertexIndices[i];
                }
            }
            return index;
        }

        //元のメッシュからcurvatureの2乗の和をFとして最急降下法で最適化する際の次のステップのメッシュを得る。もう使ってない
        public static Rhino.Geometry.Mesh SDCrvNextMesh(Rhino.Geometry.Mesh mesh)
        {
            List<int> internalVertexIndices = MeshDataTools.TopoInternalVertIndices(mesh);

            //ガウス曲率の2乗のリストを作成
            List<double> crvSum = new List<double> { };
            int count = internalVertexIndices.Count;
            for (int i = 0; i < count; i++)
            {
                double curvatureTwo = CurvatureTwo(mesh, internalVertexIndices[i]);
                crvSum.Add(curvatureTwo);
            }
            double F = crvSum.Sum();

            double[] Jacobi = new double[3 * count];
            float epsilon = 1e-6f;
            for (int i = 0; i < count; i++)
            {
                Rhino.Geometry.Mesh deltaMesh = MeshCalcTools.DeltaMesh(mesh, internalVertexIndices[i], epsilon, 0, 0);
                Jacobi[i * 3] = (CurvatureTwo(deltaMesh, internalVertexIndices[i]) - crvSum[i]) / epsilon;
                deltaMesh = MeshCalcTools.DeltaMesh(mesh, internalVertexIndices[i], 0, epsilon, 0);
                Jacobi[i * 3 + 1] = (CurvatureTwo(deltaMesh, internalVertexIndices[i]) - crvSum[i]) / epsilon;
                deltaMesh = MeshCalcTools.DeltaMesh(mesh, internalVertexIndices[i], 0, 0, epsilon);
                Jacobi[i * 3 + 2] = (CurvatureTwo(deltaMesh, internalVertexIndices[i]) - crvSum[i]) / epsilon;
            }

            double[] A = MatrixUtils.GetLowerTriangleFromVector(Jacobi);
            double[] delta = MatrixUtils.Multiply(-0.1, Jacobi);
            Rhino.Geometry.Mesh nextMesh = mesh.DuplicateMesh();
            for (int i = 0; i < count; i++)
            {
                nextMesh = MeshCalcTools.DeltaMesh(nextMesh, internalVertexIndices[i], (float)delta[3 * i], (float)delta[3 * i + 1], (float)delta[3 * i + 2]);
            }
            return nextMesh;
        }

        //最急降下法で有限差分ではなく解析的に近似で解いた偏微分でヤコビを生成したバージョン。こっちのほうが性能良い。
        public static Rhino.Geometry.Mesh SDCrvNextMesh2(Rhino.Geometry.Mesh mesh)
        {
            List<int> internalVertexIndices = MeshDataTools.TopoInternalVertIndices(mesh);
            int count = internalVertexIndices.Count;
            double[] Jacobi = new double[3 * count];

            for (int i = 0; i < count; i++)
            {
                int[] faces = mesh.TopologyVertices.ConnectedFaces(internalVertexIndices[i]);
                double angleSum = 0;
                double deltaSumX = 0;
                double deltaSumY = 0;
                double deltaSumZ = 0;

                foreach (int fi in faces)
                {
                    MeshFace face = mesh.Faces[fi];
                    List<int> indices = new List<int> { face.A, face.B, face.C };

                    List<int> topoIndices = new List<int>();
                    foreach (int idx in indices)
                    {
                        topoIndices.Add(mesh.TopologyVertices.TopologyVertexIndex(idx));
                    }

                    int vertPos = topoIndices.IndexOf(internalVertexIndices[i]);
                    if (vertPos == -1) continue;

                    Point3f p0 = mesh.Vertices[indices[(vertPos - 1 + indices.Count) % indices.Count]];
                    Point3f p1 = mesh.Vertices[indices[vertPos]];
                    Point3f p2 = mesh.Vertices[indices[(vertPos + 1) % indices.Count]];

                    Vector3d v1 = p0 - p1;
                    Vector3d v2 = p2 - p1;
                    double nv1 = v1.Length;
                    double nv2 = v2.Length;

                    double angle = Vector3d.VectorAngle(v1, v2);
                    double ABsinTheta = nv1*nv2*Math.Sin(angle);
                    double k1 = 1-nv2 * Math.Cos(angle) / nv1;
                    double k2 = 1-nv1 * Math.Cos(angle) / nv2;
                    double k3 = k1 + k2;
                    deltaSumX += (k1 * p0.X + k2 * p2.X - k3 * p1.X) / ABsinTheta;
                    deltaSumY += (k1 * p0.Y + k2 * p2.Y - k3 * p1.Y) / ABsinTheta;
                    deltaSumZ += (k1 * p0.Z + k2 * p2.Z - k3 * p1.Z) / ABsinTheta;
                    angleSum += angle;
                }
                Jacobi[i * 3] = (2*angleSum-4*Math.PI)*deltaSumX;
                Jacobi[i * 3 + 1] = (2 * angleSum - 4 * Math.PI) * deltaSumY;
                Jacobi[i * 3 + 2] = (2 * angleSum - 4 * Math.PI) * deltaSumZ;
            }

            double[] delta = MatrixUtils.Multiply(-0.1, Jacobi);
            Rhino.Geometry.Mesh nextMesh = mesh.DuplicateMesh();
            for (int i = 0; i < count; i++)
            {
                nextMesh = MeshCalcTools.DeltaMesh(nextMesh, internalVertexIndices[i], (float)delta[3 * i], (float)delta[3 * i + 1], (float)delta[3 * i + 2]);
            }
            return nextMesh;
        }
        
        //各ステップでメッシュを介するのを省略する。これがSD関係のもので一番性能がいい。
        public static Point3d[] SDnewTopoV(Point3d[] newTopoVertices, List<int> internalVertexIndices, int[][][] TriFaceID)
        {
            int count = internalVertexIndices.Count;
            Vector3d[] Jacobi = new Vector3d[count];
            int[][] faces = new int[][] { };

            //i番目の点について
            for (int i = 0; i < count; i++)
            {
                double angleSum = 0;
                double deltaSumX = 0;
                double deltaSumY = 0;
                double deltaSumZ = 0;
                Point3d p1 = newTopoVertices[internalVertexIndices[i]];

                //i番目の点周りにあるfaceのそれぞれの面に対して
                for (int j = 0; j < TriFaceID[i].Length; j++)
                {
                    Point3d p0 = newTopoVertices[TriFaceID[i][j][0]];
                    Point3d p2 = newTopoVertices[TriFaceID[i][j][1]];

                    Vector3d v1 = p0 - p1;
                    Vector3d v2 = p2 - p1;
                    double nv1 = v1.Length;
                    double nv2 = v2.Length;

                    double angle = Vector3d.VectorAngle(v1, v2);
                    double ABsinTheta = nv1 * nv2 * Math.Sin(angle);
                    double k1 = 1 - nv2 * Math.Cos(angle) / nv1;
                    double k2 = 1 - nv1 * Math.Cos(angle) / nv2;
                    double k3 = k1 + k2;

                    deltaSumX += (k1 * p0.X + k2 * p2.X - k3 * p1.X) / ABsinTheta;
                    deltaSumY += (k1 * p0.Y + k2 * p2.Y - k3 * p1.Y) / ABsinTheta;
                    deltaSumZ += (k1 * p0.Z + k2 * p2.Z - k3 * p1.Z) / ABsinTheta;
                    angleSum += angle;
                }
                double s = 2 * angleSum - 4 * Math.PI;
                Jacobi[i] =new Vector3d(s * deltaSumX, s * deltaSumY, s * deltaSumZ);
            }

            //係数αは0.1としている
            for (int i = 0; i< count; i++)
            {
                newTopoVertices[internalVertexIndices[i]] -= 1*Jacobi[i];
            }

            return newTopoVertices;
        }
     
        //共役勾配法で次のステップのメッシュを得る。
        public static void CGCrvNextMesh(Point3d[] newTopoVertices, Vector3d[] Jacobi, Vector3d[] p, List<int> internalVertexIndices, int[][][] TriFaceID)
        {
            int count = internalVertexIndices.Count;
            //int[][] faces = new int[][] { };
            Vector3d[] nextJacobi = new Vector3d[count];
            Vector3d[] nextp = new Vector3d[count];

            //i番目の点について
            for (int i = 0; i < count; i++)
            {
                double angleSum = 0;
                double deltaSumX = 0;
                double deltaSumY = 0;
                double deltaSumZ = 0;
                Point3d p1 = newTopoVertices[internalVertexIndices[i]];

                //i番目の点周りにあるfaceのそれぞれの面に対して
                for (int j = 0; j < TriFaceID[i].Length; j++)
                {
                    Point3d p0 = newTopoVertices[TriFaceID[i][j][0]];
                    Point3d p2 = newTopoVertices[TriFaceID[i][j][1]];

                    Vector3d v1 = p0 - p1;
                    Vector3d v2 = p2 - p1;
                    double nv1 = v1.Length;
                    double nv2 = v2.Length;

                    double angle = Vector3d.VectorAngle(v1, v2);
                    double ABsinTheta = nv1 * nv2 * Math.Sin(angle);
                    double k1 = 1 - nv2 * Math.Cos(angle) / nv1;
                    double k2 = 1 - nv1 * Math.Cos(angle) / nv2;
                    double k3 = k1 + k2;

                    deltaSumX += (k1 * p0.X + k2 * p2.X - k3 * p1.X) / ABsinTheta;
                    deltaSumY += (k1 * p0.Y + k2 * p2.Y - k3 * p1.Y) / ABsinTheta;
                    deltaSumZ += (k1 * p0.Z + k2 * p2.Z - k3 * p1.Z) / ABsinTheta;
                    angleSum += angle;
                }
                double s = 2 * angleSum - 4 * Math.PI;

                nextJacobi[i] = new Vector3d(s * deltaSumX, s * deltaSumY, s * deltaSumZ);

            }

            double betaUpper = 0;
            double betaLower = 0;

            for (int i = 0; i < count; i++)
            {
                betaLower += Jacobi[i].SquareLength;
                betaUpper += (nextJacobi[i] - Jacobi[i]) * nextJacobi[i];
            }

            //係数αは1としている
            for (int i = 0; i < count; i++)
            {
                nextp[i] = -nextJacobi[i] + betaUpper * p[i] / betaLower;
                newTopoVertices[internalVertexIndices[i]] += 1*nextp[i];
                p[i] = nextp[i];
                Jacobi[i] = nextJacobi[i]; 
            }
        }

        /*
        public static void CGCrvNextCutMesh(List<Point3d> newTopoVertices, Vector3d[] Jacobi, Vector3d[] p, List<int> internalVertexIndices, int[][][] TriFaceID)
        {
            int count = internalVertexIndices.Count;
            Vector3d[] nextJacobi = new Vector3d[count];
            Vector3d[] nextp = new Vector3d[count];

            //i番目の点について
            for (int i = 0; i < count; i++)
            {
                double angleSum = 0;
                double deltaSumX = 0;
                double deltaSumY = 0;
                double deltaSumZ = 0;
                Point3d p1 = newTopoVertices[internalVertexIndices[i]];

                //i番目の点周りにあるfaceのそれぞれの面に対して
                for (int j = 0; j < TriFaceID[i].Length; j++)
                {
                    Point3d p0 = newTopoVertices[TriFaceID[i][j][0]];
                    Point3d p2 = newTopoVertices[TriFaceID[i][j][1]];

                    Vector3d v1 = p0 - p1;
                    Vector3d v2 = p2 - p1;
                    double nv1 = v1.Length;
                    double nv2 = v2.Length;

                    double angle = Vector3d.VectorAngle(v1, v2);
                    double ABsinTheta = nv1 * nv2 * Math.Sin(angle);
                    double k1 = 1 - nv2 * Math.Cos(angle) / nv1;
                    double k2 = 1 - nv1 * Math.Cos(angle) / nv2;
                    double k3 = k1 + k2;

                    deltaSumX += (k1 * p0.X + k2 * p2.X - k3 * p1.X) / ABsinTheta;
                    deltaSumY += (k1 * p0.Y + k2 * p2.Y - k3 * p1.Y) / ABsinTheta;
                    deltaSumZ += (k1 * p0.Z + k2 * p2.Z - k3 * p1.Z) / ABsinTheta;
                    angleSum += angle;
                }
                double s = 2 * angleSum - 4 * Math.PI;

                nextJacobi[i] = new Vector3d(s * deltaSumX, s * deltaSumY, s * deltaSumZ);

            }

            double betaUpper = 0;
            double betaLower = 0;

            for (int i = 0; i < count; i++)
            {
                betaLower += Jacobi[i].SquareLength;
                betaUpper += (nextJacobi[i] - Jacobi[i]) * nextJacobi[i];
            }

            //係数αは1としている
            for (int i = 0; i < count; i++)
            {
                nextp[i] = -nextJacobi[i] + betaUpper * p[i] / betaLower;
                newTopoVertices[internalVertexIndices[i]] += 1*nextp[i];
                p[i] = nextp[i];
                Jacobi[i] = nextJacobi[i]; 
            }
        }*/
    }
}