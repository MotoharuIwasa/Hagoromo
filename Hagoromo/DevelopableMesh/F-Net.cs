using Grasshopper.Kernel;
using Hagoromo.MathTools;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Collections;
using Rhino.Render.ChangeQueue;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Hagoromo.GeometryTools;

using System.Numerics;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra.Double.Solvers;
using MathNet.Numerics.LinearAlgebra.Factorization;
using MathNet.Numerics.Distributions;


namespace Hagoromo.DevelopableMesh
{
    public static class NetTools
    {
        //入力のメッシュ(開いたメッシュ)とトポロジカル的に等しいxy平面上のメッシュを作成。かつ重なりを許さない。
        //出力はnewTopoVerticesの順のままで、x,y座標を格納した2重の配列
        public static double[][] TutteTopoVertices(Rhino.Geometry.Mesh mesh, Polyline outline)
        {
            double tol = mesh.GetBoundingBox(true).Diagonal.Length * 1e-5;
            if (outline[0].DistanceTo(outline[outline.Count - 1]) < tol)
            {
                outline.RemoveAt(outline.Count - 1);
            }
            /*
            if (outline[0].DistanceTo(outline[outline.Count - 1]) < 1e-6)
            {
                outline.RemoveAt(outline.Count - 1);
            }
            */
            int nGon = outline.Count;

            AreaMassProperties amp = AreaMassProperties.Compute(mesh);
            double area = amp.Area;
            double radius = Math.Sqrt(area / Math.PI);
            double deltatheta = 2 * Math.PI / nGon;
            Point3d[] newTopoVertices = MeshDataTools.DoubleTopoVertices(mesh);

            int[] bTopoIndex = PtCrvTools.GetOutlineIndices(newTopoVertices, outline);

            // HashSet にして高速検索できるようにする
            var includedSet = new HashSet<int>(bTopoIndex);
            List<int> inTopoIndexList = new List<int>();
            for (int i = 0; i < newTopoVertices.Length; i++)
            {
                if (!includedSet.Contains(i))
                    inTopoIndexList.Add(i);
            }
            int[] inTopoIndex = inTopoIndexList.ToArray();

            double[][] P = new double[newTopoVertices.Length][];


            for (int i = 0; i < bTopoIndex.Length; i++)
            {
                int idx = bTopoIndex[i];
                if (0 <= idx && idx < P.Length)
                {
                    if (P[idx] == null)
                        P[idx] = new double[2];
                }
                else
                {
                    Rhino.RhinoApp.WriteLine($"Skipped invalid index {idx}");
                }
            }


            for (int i = 0; i < nGon; i++)
            {
                P[bTopoIndex[i]] = new double[2];
                P[bTopoIndex[i]][0] = radius * Math.Cos(deltatheta * i);
                P[bTopoIndex[i]][1] = radius * Math.Sin(deltatheta * i);
            }


            List<int[]> connectedVerticesList = new List<int[]>();
            for (int i = 0; i < newTopoVertices.Length; i++)
            {
                if (Array.IndexOf(bTopoIndex, i) == -1)
                {
                    connectedVerticesList.Add(mesh.TopologyVertices.ConnectedTopologyVertices(i));
                }
            }
            int[][] connectedVertices = connectedVerticesList.ToArray();

            double[][] A = new double[inTopoIndex.Length][];
            double[][] Wib = new double[inTopoIndex.Length][];
            for (int i = 0; i < inTopoIndex.Length; i++)
            {
                double deg = 1.0 / connectedVertices[i].Length;
                A[i] = new double[inTopoIndex.Length];
                for (int j = 0; j < inTopoIndex.Length; j++)
                {
                    if (i == j)
                    {
                        A[i][j] = 1;
                    }

                    else
                    {
                        if (Array.IndexOf(connectedVertices[i], inTopoIndex[j]) == -1)
                        {
                            A[i][j] = 0;
                        }
                        else
                        {
                            A[i][j] = -deg;
                        }
                    }
                }

                Wib[i] = new double[bTopoIndex.Length];
                for (int j = 0; j < bTopoIndex.Length; j++)
                {
                    if (Array.IndexOf(connectedVertices[i], bTopoIndex[j]) == -1)
                    {
                        Wib[i][j] = 0;
                    }
                    else
                    {
                        Wib[i][j] = deg;
                    }
                }
            }

            double[] Bx = new double[inTopoIndex.Length];
            double[] By = new double[inTopoIndex.Length];
            for (int i = 0; i < inTopoIndex.Length; i++)
            {
                for (int j = 0; j < bTopoIndex.Length; j++)
                {
                    Bx[i] += Wib[i][j] * radius * Math.Cos(deltatheta * j);
                    By[i] += Wib[i][j] * radius * Math.Sin(deltatheta * j);
                }
            }

            //Ax=Bx, Ax=Byの解がそれぞれ内部の点のx,y座標(inTopoVerticesの順)
            double[] Xx = SLEsSolvers.SolveSparseLinearSystem(A, Bx);
            double[] Xy = SLEsSolvers.SolveSparseLinearSystem(A, By);


            for (int i = 0; i < inTopoIndex.Length; i++)
            {
                P[inTopoIndex[i]] = new double[2];
                P[inTopoIndex[i]][0] = Xx[i];
                P[inTopoIndex[i]][1] = Xy[i];
            }
            return P;
        }

        //長さの差の２乗和をｆとしたときのjacobi
        public static double[][] LengthJacobi(int[][] connectedVertices, double[][] initialLength, double[][] newTopoVertices2d)
        {
            int count = newTopoVertices2d.Length;
            double[][] currentLength = MeshCalcTools.GetMeshEdgesLength(connectedVertices, newTopoVertices2d);
            double[][] nextJacobi = new double[count][];
            for (int i = 0; i < count; i++)
            {
                nextJacobi[i] = new double[2];
                for (int j = 0; j < connectedVertices[i].Length; j++)
                {
                    int index = connectedVertices[i][j];
                    double s = 1 - initialLength[i][j] / currentLength[i][j];
                    nextJacobi[i][0] += 2 * (newTopoVertices2d[i][0] - newTopoVertices2d[index][0]) * s;
                    nextJacobi[i][1] += 2 * (newTopoVertices2d[i][1] - newTopoVertices2d[index][1]) * s;
                }
            }
            return nextJacobi;
        }

        //共役勾配法で最適化、ただし裏表がひっくり返らないようにする制約条件なし
        //元の立体メッシュから求まる int[][] connectedVertices, double[][] initialLengthと現在の点 double[][] NewTopoVertices2d
        public static void NetCGLength(double[][] Jacobi, double[][] p, int[][] connectedVertices, double[][] initialLength, double[][] newTopoVertices2d)
        {
            int count = newTopoVertices2d.Length;
            double[][] nextJacobi = LengthJacobi(connectedVertices, initialLength, newTopoVertices2d);

            double betaUpper = 0;
            double betaLower = 0;
            for (int i = 0; i < count; i++)
            {
                betaLower += Jacobi[i][0] * Jacobi[i][0];
                betaLower += Jacobi[i][1] * Jacobi[i][1];
                betaUpper += (nextJacobi[i][0] - Jacobi[i][0]) * nextJacobi[i][0];
                betaUpper += (nextJacobi[i][1] - Jacobi[i][1]) * nextJacobi[i][1];
            }

            double[][] nextp = new double[count][];
            //係数αは0.1としている
            double alpha = 0.1;
            for (int i = 0; i < count; i++)
            {
                nextp[i] = new double[2];
                nextp[i][0] = -nextJacobi[i][0] + betaUpper * p[i][0] / betaLower;
                nextp[i][1] = -nextJacobi[i][1] + betaUpper * p[i][1] / betaLower;
                newTopoVertices2d[i][0] += alpha * nextp[i][0];
                newTopoVertices2d[i][1] += alpha * nextp[i][1];
                p[i] = nextp[i];
                Jacobi[i][0] = nextJacobi[i][0];
                Jacobi[i][1] = nextJacobi[i][1];
            }
        }



        //各面の符号付face areaを関数としたときの double[faceの数][3][2] jacobi
        //Jacobi[i][j][k]はi番目のfaceの関数giにおけるp_jのｋ(xかy)座標のところのJacobi
        public static double[][][] FaceAreaJacobi(int[][] faceTopoVertIndices, int[][] connectedVertices, double[][] initialLength, double[][] newTopoVertices2d)
        {
            int count = faceTopoVertIndices.Length;
            double[][][] nextJacobi = new double[count][][];
            for (int i = 0; i < nextJacobi.Length; i++)
            {
                nextJacobi[i] = new double[3][];
                for (int j = 0; j < 3; j++)
                {
                    nextJacobi[i][j] = new double[2];
                }
            }

            for (int i = 0; i < faceTopoVertIndices.Length; i++)
            {
                double[] p0 = newTopoVertices2d[faceTopoVertIndices[i][0]];
                double[] p1 = newTopoVertices2d[faceTopoVertIndices[i][1]];
                double[] p2 = newTopoVertices2d[faceTopoVertIndices[i][2]];
                nextJacobi[i][0][0] = p1[1] - p2[1];
                nextJacobi[i][0][1] = p1[0] - p2[0];
                nextJacobi[i][1][0] = p2[1] - p0[1];
                nextJacobi[i][1][1] = p2[0] - p0[0];
                nextJacobi[i][2][0] = p0[1] - p1[1];
                nextJacobi[i][2][1] = p0[0] - p1[0];
            }
            return nextJacobi;
        }

        //LengthベースででAreaが0以下という制約付きでsoftplus型の目的関数におけるJacobi
        public static double[][] LengthConsAreaJacobi(double lambda, double alpha, int[][] faceTopoVertIndices, int[][] connectedVertices, double[][] initialLength, double[][] newTopoVertices2d)
        {
            int count = newTopoVertices2d.Length;
            double[][] currentLength = MeshCalcTools.GetMeshEdgesLength(connectedVertices, newTopoVertices2d);
            double[][] totalJacobi = LengthJacobi(connectedVertices, initialLength, newTopoVertices2d);
            double[][][] areaJacobi = FaceAreaJacobi(faceTopoVertIndices, connectedVertices, initialLength, newTopoVertices2d);

            for (int i = 0; i < faceTopoVertIndices.Length; i++)
            {
                double[] p0 = newTopoVertices2d[faceTopoVertIndices[i][0]];
                double[] p1 = newTopoVertices2d[faceTopoVertIndices[i][1]];
                double[] p2 = newTopoVertices2d[faceTopoVertIndices[i][2]];
                double area = (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p2[0] - p0[0]) * (p1[1] - p0[1]);
                for (int j = 0; j < 3; j++)
                {
                    for (int k = 0; k < 2; k++)
                    {
                        /*
                        if (area > 0)
                        {
                            area = 0;
                        }
                        */
                        totalJacobi[faceTopoVertIndices[i][j]][k] += lambda * areaJacobi[i][j][k] / (1 + Math.Exp(-alpha * area));

                    }
                }
            }
            return totalJacobi;
        }

        //共役勾配法で最適化、ただし裏表がひっくり返らないようにする制約条件あり
        public static void NetCGLengthCons(double[][] Jacobi, double[][] p, int[][] faceTopoVertIndices, int[][] connectedVertices, double[][] initialLength, double[][] newTopoVertices2d)
        {
            double lambda = 0.1;
            double alpha2 = 0.001;
            double[][] nextJacobi = LengthConsAreaJacobi(lambda, alpha2, faceTopoVertIndices, connectedVertices, initialLength, newTopoVertices2d);
            int count = newTopoVertices2d.Length;

            double betaUpper = 0;
            double betaLower = 0;
            for (int i = 0; i < count; i++)
            {
                betaLower += Jacobi[i][0] * Jacobi[i][0];
                betaLower += Jacobi[i][1] * Jacobi[i][1];
                betaUpper += (nextJacobi[i][0] - Jacobi[i][0]) * nextJacobi[i][0];
                betaUpper += (nextJacobi[i][1] - Jacobi[i][1]) * nextJacobi[i][1];
            }

            double[][] nextp = new double[count][];

            //係数αは0.1としている
            double alpha = 0.01;
            for (int i = 0; i < count; i++)
            {
                nextp[i] = new double[2];
                nextp[i][0] = -nextJacobi[i][0] + betaUpper * p[i][0] / betaLower;
                nextp[i][1] = -nextJacobi[i][1] + betaUpper * p[i][1] / betaLower;
                newTopoVertices2d[i][0] += alpha * nextp[i][0];
                newTopoVertices2d[i][1] += alpha * nextp[i][1];
                p[i] = nextp[i];
                Jacobi[i][0] = nextJacobi[i][0];
                Jacobi[i][1] = nextJacobi[i][1];
            }
        }

        /*
        //L-BFGSで最適化、裏表がひっくり返らないように符号付面積による制約条件あり
        public static void NetLBFGSLength(double[][] Jacobi, double[][] p, int[][] faceTopoVertIndices, int[][] connectedVertices, double[][] initialLength, double[][] newTopoVertices2d)
        {
            int lambda = 10;
            int alpha = 5;
            double[][] nextJacobi = LengthConsAreaJacobi(lambda, alpha, faceTopoVertIndices, connectedVertices, initialLength, newTopoVertices2d);

            int memorySize = 5;

        }
        */

        public static double[][] CalcAngleJacobi(double r0, double s0, double X1, double X2, double Y1, double Y2)
        {
            double X1Y1 = X1 * X1 + Y1 * Y1;
            double X2Y2 = X2 * X2 + Y2 * Y2;
            double X1mY1 = X1 * X1 - Y1 * Y1;
            double f = Math.Sqrt(X1Y1 / X2Y2) / (X1mY1);
            double p = X1 * X2 + Y1 * Y2;
            double q = X1 * Y2 - X2 * Y1;
            double a = f * p * p - r0 * p + f * q * q - s0 * q;
            double b = f * (f * p - r0);
            double c = f * (f * q - s0);
            double[] d = { a, b, c };
            double[,] A = new double[2, 3];
            double[,] B = new double[2, 3];
            double[,] C = new double[2, 3];
            A[0, 0] = f * (X2 / X2Y2 - X1 / X1Y1 + 2 * X1 / X1mY1);
            A[0, 1] = -(X1 + X2);
            A[0, 2] = Y1 + Y2;
            A[1, 0] = f * (Y2 / X2Y2 - Y1 / X1Y1 + 2 * Y1 / X1mY1);
            A[1, 1] = -(Y1 + Y2);
            A[1, 2] = X2 - X1;
            B[0, 0] = f * X1 * (1 / X1Y1 - 2 / X1mY1);
            B[0, 1] = X2;
            B[0, 2] = Y2;
            B[1, 0] = f * Y1 * (1 / X1Y1 + 2 / X1mY1);
            B[1, 1] = Y2;
            B[1, 2] = -X2;
            C[0, 0] = -f * X2 / X2Y2;
            C[0, 1] = X1;
            C[0, 2] = -Y1;
            C[1, 0] = -f * Y2 / X2Y2;
            C[1, 1] = Y1;
            C[1, 2] = X1;
            double[][] D = new double[3][];
            D[0] = MatrixUtils.Multiply(A, d);
            D[1] = MatrixUtils.Multiply(B, d);
            D[2] = MatrixUtils.Multiply(C, d);
            return D;
        }

        //符号付Angleで単位円状上での距離の2乗の和を目的関数としたときのjacobi。これにより制約条件が不要に
        public static double[][] AngleJacobi(int[][] faceTopoVertIndices, double[][][] initialAngle, double[][] newTopoVertices2d)
        {
            double[][] jacobi = new double[newTopoVertices2d.Length][];
            for (int i = 0; i < jacobi.Length; i++)
            {
                jacobi[i] = new double[2];
            }

            for (int i = 0; i < faceTopoVertIndices.Length; i++)
            {
                double r0 = initialAngle[i][0][0];
                double s0 = initialAngle[i][0][1];
                double X1 = newTopoVertices2d[faceTopoVertIndices[i][1]][0] - newTopoVertices2d[faceTopoVertIndices[i][0]][0];
                double Y1 = newTopoVertices2d[faceTopoVertIndices[i][1]][1] - newTopoVertices2d[faceTopoVertIndices[i][0]][1];
                double X2 = newTopoVertices2d[faceTopoVertIndices[i][2]][0] - newTopoVertices2d[faceTopoVertIndices[i][0]][0];
                double Y2 = newTopoVertices2d[faceTopoVertIndices[i][2]][1] - newTopoVertices2d[faceTopoVertIndices[i][0]][1];
                double[][] D = CalcAngleJacobi(r0, s0, X1, X2, Y1, Y2);
                for (int j = 0; j < 3; j++)
                {
                    jacobi[faceTopoVertIndices[i][j]][0] += D[j][0];
                    jacobi[faceTopoVertIndices[i][j]][1] += D[j][1];
                }


                r0 = initialAngle[i][1][0];
                s0 = initialAngle[i][1][1];
                X1 = newTopoVertices2d[faceTopoVertIndices[i][2]][0] - newTopoVertices2d[faceTopoVertIndices[i][1]][0];
                Y1 = newTopoVertices2d[faceTopoVertIndices[i][2]][1] - newTopoVertices2d[faceTopoVertIndices[i][1]][1];
                X2 = newTopoVertices2d[faceTopoVertIndices[i][0]][0] - newTopoVertices2d[faceTopoVertIndices[i][1]][0];
                Y2 = newTopoVertices2d[faceTopoVertIndices[i][0]][1] - newTopoVertices2d[faceTopoVertIndices[i][1]][1];
                D = CalcAngleJacobi(r0, s0, X1, X2, Y1, Y2);
                for (int j = 0; j < 3; j++)
                {
                    jacobi[faceTopoVertIndices[i][j]][0] += D[j][0];
                    jacobi[faceTopoVertIndices[i][j]][1] += D[j][1];
                }

            }
            return jacobi;
        }

        public static void NetCGAngle(double[][] Jacobi, double[][] p, int[][] faceTopoVertIndices, double[][][] initialAngle, double[][] newTopoVertices2d)
        {
            int count = newTopoVertices2d.Length;
            double[][] nextJacobi = AngleJacobi(faceTopoVertIndices, initialAngle, newTopoVertices2d);

            double betaUpper = 0;
            double betaLower = 0;
            for (int i = 0; i < count; i++)
            {
                betaLower += Jacobi[i][0] * Jacobi[i][0];
                betaLower += Jacobi[i][1] * Jacobi[i][1];
                betaUpper += (nextJacobi[i][0] - Jacobi[i][0]) * nextJacobi[i][0];
                betaUpper += (nextJacobi[i][1] - Jacobi[i][1]) * nextJacobi[i][1];
            }

            double[][] nextp = new double[count][];
            //係数αは0.1としている
            double alpha = 0.00001;
            for (int i = 0; i < count; i++)
            {
                nextp[i] = new double[2];
                nextp[i][0] = -nextJacobi[i][0] + betaUpper * p[i][0] / betaLower;
                nextp[i][1] = -nextJacobi[i][1] + betaUpper * p[i][1] / betaLower;
                newTopoVertices2d[i][0] += alpha * nextp[i][0];
                newTopoVertices2d[i][1] += alpha * nextp[i][1];
                p[i] = nextp[i];
                Jacobi[i][0] = nextJacobi[i][0];
                Jacobi[i][1] = nextJacobi[i][1];
            }
        }
    

        /*---------------------------------------------------BFF------------------------------------------------------------*/
        //等角写像による展開図作成(BFFという手法)
        public static CutMesh NetBFF(CutMesh mesh)
        {
            CutMesh cutMesh = mesh.Sort();
            int boundaryVertCount = cutMesh.BoundaryVertIndices().Count;
            //List<List<double>> angles = CutMeshCalcTools.InteriorAngles(cutMesh);
            double[,] A = BuildLaplace(cutMesh);
            //for (int i = 0; i < A.GetLength(0); ++i) A[i, i] += 1e-8;
            //DenseMatrix mat = DenseMatrix.OfArray(A);
            //Cholesky<double> cholA = mat.Cholesky();   // 内部で高速アルゴリズムを使用

            double[] kdiff = new double[boundaryVertCount];
            double[] gaussian = CutMeshCalcTools.GaussianCurvature(cutMesh);
            //double[] gaussian2 = new double[gaussian.Length];
            double[] k = CutMeshCalcTools.GeodesicCurvature(cutMesh);
            double[] u = new double[boundaryVertCount];
            //double[] u = NeumannToDirichlet(A, MatrixUtils.Multiply(-1,gaussian), kdiff, boundaryVertCount);

            int boundaryEdgeCount = boundaryVertCount;
            double[] l = new double[boundaryEdgeCount];
            double[] lnew = new double[boundaryEdgeCount];
            for (int i = 0; i < boundaryEdgeCount; i++)
            {
                Vector3d a = cutMesh.Vertices[i] - cutMesh.Vertices[(i + 1) % boundaryEdgeCount];
                double length = a.Length;
                l[i] = length;
                //lnew[i] = l[i] * Math.Exp((u[i] + u[(i + 1) % boundaryVertCount]) * 0.5);
                lnew[i] = l[i];
            }

            double[] gammaRe = BestFitCurve(l, lnew, k);

            int innerVertCount = cutMesh.Vertices.Count - boundaryVertCount;
            //Matrix<double> L = cholA.Factor;
            //Matrix<double> innerL = DenseMatrix.Create(innerVertCount, innerVertCount, (i, j) => L[boundaryVertCount+i, boundaryVertCount+j]);
            //Cholesky<double> cholAinner = innerL.Cholesky();
            double[,] Ainner = MatrixUtils.ExtractPartMatrix(A, boundaryVertCount, boundaryVertCount, innerVertCount, innerVertCount);
            double[,] Aib = MatrixUtils.ExtractPartMatrix(A, boundaryVertCount, 0, innerVertCount, boundaryVertCount);
            List<Point3d> newVertices = ExtendCurve(Ainner, A, Aib, gammaRe);

            cutMesh.Vertices = newVertices;
            cutMesh.DuplicatedVertIndices = Enumerable.Range(0, cutMesh.Vertices.Count)
                              .Select(i => new List<int> { i })
                              .ToList();

            return cutMesh;
        }

        //A
        public static double[,] BuildLaplace(CutMesh mesh)
        {
            int count = mesh.Edges.Count;
            int size = mesh.Vertices.Count;
            double[,] A = new double[size,size];
            for (int i = 0; i < count; i++)
            {
                List<int> faces = mesh.GetFacesForEdge(i);
                int ii = mesh.Edges[i][0];
                int jj = mesh.Edges[i][1];
                List<int> iijj = new List<int> { ii, jj };
                for (int j = 0; j < faces.Count; j++)
                {
                    double angle = 0;
                    if (!(iijj.Contains(mesh.Faces[faces[j], 0])))
                    {
                        Point3d oo = mesh.Vertices[mesh.Faces[faces[j], 0]];
                        Vector3d vec1 = mesh.Vertices[ii] - oo;
                        Vector3d vec2 = mesh.Vertices[jj] - oo;
                        angle = Vector3d.VectorAngle(vec1, vec2);
                    }
                    if (!(iijj.Contains(mesh.Faces[faces[j], 1])))
                    {
                        Point3d oo = mesh.Vertices[mesh.Faces[faces[j], 1]];
                        Vector3d vec1 = mesh.Vertices[ii] - oo;
                        Vector3d vec2 = mesh.Vertices[jj] - oo;
                        angle = Vector3d.VectorAngle(vec1, vec2);
                    }
                    if (!(iijj.Contains(mesh.Faces[faces[j], 2])))
                    {
                        Point3d oo = mesh.Vertices[mesh.Faces[faces[j], 2]];
                        Vector3d vec1 = mesh.Vertices[ii] - oo;
                        Vector3d vec2 = mesh.Vertices[jj] - oo;
                        angle = Vector3d.VectorAngle(vec1, vec2);
                    }
                    A[ii, jj] += -0.5/Math.Tan(angle);
                }
                A[jj, ii] = A[ii, jj];
            }

            for (int i = 0; i < size; i++)
            {
                List<int> vertices = mesh.GetVerticesForVertex(i);
                foreach (int j in vertices)
                {
                    A[i, i] -= A[i, j];
                }
            }
            return A;
        }  

        public static double[] NeumannToDirichlet(double[,] A, double[] fai, double[] h, int boundaryVertCount)
        {
            double[] h2 = new double[fai.Length];
            for (int i = 0; i < boundaryVertCount; i++)
            {
                h2[i] = h[i];
            }
            double[] b = MatrixUtils.Subtract(fai,h2);

            double[,] Apart = MatrixUtils.ExtractPartMatrix(A, 1, 1, A.GetLength(0) - 1, A.GetLength(1) - 1);
            double[] bpart = new double[b.Length - 1];
            for (int i = 0; i < b.Length - 1; i++)
            {
                bpart[i] = b[i + 1];
            }

            var Amat = DenseMatrix.OfArray(Apart);
            var bmat = MathNet.Numerics.LinearAlgebra.Double.Vector.Build.DenseOfArray(bpart);
            double[] apart = Amat.Solve(bmat).ToArray();
            double[] a = new double[apart.Length + 1];
            for (int i = 1; i < a.Length; i++)
            {
                a[i] = apart[i - 1];
            }
            //double[] a = SLEsSolvers.SolveByLDL(Atri, b);
            //a[0] = 0;
            //double[] u = a.Take(boundaryVertCount).ToArray();

            //double[] a = SLEsSolvers.SolveWithMathNet(A, b);
            return a.Take(boundaryVertCount).ToArray();
        }

        public static double[] BestFitCurve(double[] l, double[] lnew, double[] knew)
        {
            int count = l.Length;
            double[,] Ttrans = new double[count, 2];
            double theta = 0;
            double[,] Ninv = new double[count, count];
            for (int i = 0; i < count; i++)
            {
                theta += knew[i];
                Ttrans[i,0] = Math.Cos(theta);
                Ttrans[i,1] = Math.Sin(theta);
                Ninv[i, i] = lnew[i];
            }
            double[,] T = MatrixUtils.Transpose(Ttrans);
            double[,] TNTinv = MatrixUtils.InverseMatrix(MatrixUtils.Multiply(T, MatrixUtils.Multiply(Ninv,Ttrans)));
            double[,] NTTNTT = MatrixUtils.Multiply(Ninv, MatrixUtils.Multiply(Ttrans, MatrixUtils.Multiply(TNTinv, T)));

            double[] lcorrect = MatrixUtils.Subtract(lnew, MatrixUtils.Multiply(NTTNTT, lnew));
            double[] gammaRe = new double[count];
            for(int i = 1; i < count; ++i)
            {
                gammaRe[i] = gammaRe[i - 1] + lcorrect[i - 1] * Ttrans[i - 1,0];
            }
            return gammaRe;
        }

        public static double[][] BestFitCurve2(double[] l, double[] lnew, double[] knew)
        {
            int count = l.Length;
            double[,] Ttrans = new double[count, 2];
            double theta = 0;
            double[,] Ninv = new double[count, count];
            for (int i = 0; i < count; i++)
            {
                theta += knew[i];
                Ttrans[i, 0] = Math.Cos(theta);
                Ttrans[i, 1] = Math.Sin(theta);
                Ninv[i, i] = lnew[i];
            }
            double[,] T = MatrixUtils.Transpose(Ttrans);
            double[,] TNTinv = MatrixUtils.InverseMatrix(MatrixUtils.Multiply(T, MatrixUtils.Multiply(Ninv, Ttrans)));
            double[,] NTTNTT = MatrixUtils.Multiply(Ninv, MatrixUtils.Multiply(Ttrans, MatrixUtils.Multiply(TNTinv, T)));

            double[] lcorrect = MatrixUtils.Subtract(lnew, MatrixUtils.Multiply(NTTNTT, lnew));
            double[][] gamma = new double[count][];
            gamma[0] = new double[2] { 0, 0 };
            for (int i = 1; i < count; ++i)
            {
                gamma[i] = new double[2];
                gamma[i][0] = gamma[i - 1][0] + lcorrect[i - 1] * Ttrans[i - 1, 0];
                gamma[i][1] = gamma[i - 1][1] + lcorrect[i - 1] * Ttrans[i - 1, 1];
            }
            return gamma;
        }

        public static List<Point3d> ExtendCurve(double[,] Ainner, double[,] A, double[,] Aib, double[] gammaRe)
        {
            double[] y = MatrixUtils.Multiply(-1,MatrixUtils.Multiply(Aib, gammaRe));
            double[] ainner = SLEsSolvers.SolveWithMathNet(Ainner, y);
            int innerCount = ainner.Length;
            int boundaryCount = gammaRe.Length;
            int VerticesCount = innerCount + boundaryCount;
            double[] a = new double[VerticesCount];
            for (int i = 0; i < boundaryCount; ++i)
            {
                a[i] = gammaRe[i];
            }
            for (int i = 0;i < innerCount; ++i)
            {
                a[i + boundaryCount] = ainner[i];
            }

            double[] h = new double[A.GetLength(0)];
            for (int i = 0; i < boundaryCount; i++)
            {
                h[i] = 0.5*(gammaRe[(i+1) % boundaryCount] - gammaRe[(i - 1+ boundaryCount)%boundaryCount]);
            }
            double[] b = SLEsSolvers.SolveWithMathNet(A, h);
            List<Point3d> newVertices = new List<Point3d> { };
            for (int i = 0; i < VerticesCount; i++)
            {
                newVertices.Add(new Point3d (a[i], b[i], 0 ));
            }
            return newVertices;
        }

        /*---------------------------------------------------BFF and CGLength------------------------------------------------------------*/

        public static CutMesh NetBFFandCGLength(CutMesh mesh)
        {
            CutMesh cutMesh = mesh.Sort();
            int boundaryVertCount = cutMesh.BoundaryVertIndices().Count;
            double[,] A = BuildLaplace(cutMesh);

            double[] k = CutMeshCalcTools.GeodesicCurvature(cutMesh);
            double[] u = new double[boundaryVertCount];

            int boundaryEdgeCount = boundaryVertCount;
            double[] l = new double[boundaryEdgeCount];
            for (int i = 0; i < boundaryEdgeCount; i++)
            {
                Vector3d a = cutMesh.Vertices[i] - cutMesh.Vertices[(i + 1) % boundaryEdgeCount];
                double length = a.Length;
                l[i] = length;
            }

            double[] gammaRe = BestFitCurve(l, l, k);

            int innerVertCount = cutMesh.Vertices.Count - boundaryVertCount;
            double[,] Ainner = MatrixUtils.ExtractPartMatrix(A, boundaryVertCount, boundaryVertCount, innerVertCount, innerVertCount);
            double[,] Aib = MatrixUtils.ExtractPartMatrix(A, boundaryVertCount, 0, innerVertCount, boundaryVertCount);
            List<Point3d> newVertices = ExtendCurve(Ainner, A, Aib, gammaRe);

            int count = cutMesh.Vertices.Count;
            int[][] connectedVertices = new int[count][];
            for (int i =0; i < count; i++)
            {
                int[] connected = cutMesh.GetVerticesForVertex(i).ToArray();
                connectedVertices[i] = new int[connected.Length];
                connectedVertices[i] = connected;
            }
            double[][] initialLength = MeshCalcTools.GetMeshEdgesLength(connectedVertices, cutMesh.Vertices.ToArray());
            double[][] Jacobi = new double[count][];
            for (int i = 0; i < count; i++)
            {
                Jacobi[i] = new double[2];
            }
            Jacobi[0][0] = 1;
            double[][] p = new double[count][];
            for (int i = 0; i < count; i++)
            {
                p[i] = new double[2];
            }
            double[][] newVertices2d = new double[count][];
            for (int i = 0; i < count; i++)
            {
                newVertices2d[i]  = new double[2];
                newVertices2d[i][0] = newVertices[i][0];
                newVertices2d[i][1] = newVertices[i][1];
            }

            for (int iii = 0; iii < 1000; iii++)
            {
                double[][] nextJacobi = LengthJacobi(connectedVertices, initialLength, newVertices2d);

                double betaUpper = 0;
                double betaLower = 0;
                for (int i = 0; i < count; i++)
                {
                    betaLower += Jacobi[i][0] * Jacobi[i][0];
                    betaLower += Jacobi[i][1] * Jacobi[i][1];
                    betaUpper += (nextJacobi[i][0] - Jacobi[i][0]) * nextJacobi[i][0];
                    betaUpper += (nextJacobi[i][1] - Jacobi[i][1]) * nextJacobi[i][1];
                }

                double[][] nextp = new double[count][];
                //係数αは0.1としている
                double alpha = 0.1;
                for (int i = 0; i < count; i++)
                {
                    nextp[i] = new double[2];
                    nextp[i][0] = -nextJacobi[i][0] + betaUpper * p[i][0] / betaLower;
                    nextp[i][1] = -nextJacobi[i][1] + betaUpper * p[i][1] / betaLower;
                    newVertices2d[i][0] += alpha * nextp[i][0];
                    newVertices2d[i][1] += alpha * nextp[i][1];
                    p[i] = nextp[i];
                    Jacobi[i][0] = nextJacobi[i][0];
                    Jacobi[i][1] = nextJacobi[i][1];
                }
            }
            for (int i = 0; i < count; i++)
            {
                newVertices[i] = new Point3d( newVertices2d[i][0], newVertices2d[i][1], 0 );
            }

            cutMesh.Vertices = newVertices;
            cutMesh.DuplicatedVertIndices = Enumerable.Range(0, cutMesh.Vertices.Count)
                              .Select(i => new List<int> { i })
                              .ToList();
            return cutMesh;
        }
    }
}
