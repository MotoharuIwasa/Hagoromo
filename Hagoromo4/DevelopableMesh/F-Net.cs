using Cloo.Bindings;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Geometry.Delaunay;
using Grasshopper.Kernel.Types;
using Hagoromo.GeometryTools;
using Hagoromo.MathTools;
using HeuristicLab.Common;
using HeuristicLab.Core;
using MathNet.Numerics.Distributions;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra.Double.Solvers;
using MathNet.Numerics.LinearAlgebra.Factorization;
using MathNet.Numerics.Optimization;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Collections;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Windows.Forms.VisualStyles;
using System.Xml.Linq;
using static Hagoromo.DevelopableMesh.DevelopableTools;
using static Hagoromo.GeometryTools.CutMeshCalcTools;


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
    



        //---------------------------------------------------BFF------------------------------------------------------------
        //等角写像による展開図作成(BFFという手法)、cutMeshはsortした状態で入れておく
        public static List<Point3d> NetBFF(CutMesh cutMesh)
        {
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
            return newVertices;
        }

        //穴ありに対応したBFF
        public static List<Point3d> NetBFF(CutMesh mesh, int boundaryVertCount)
        {
            CutMesh cutMesh = mesh.Sort();

            double[,] A = BuildLaplace(cutMesh);
            //for (int i = 0; i < A.GetLength(0); ++i) A[i, i] += 1e-8;
            //DenseMatrix mat = DenseMatrix.OfArray(A);
            //Cholesky<double> cholA = mat.Cholesky();   // 内部で高速アルゴリズムを使用
            double[] kdiff = new double[boundaryVertCount];

            //------------------------------------あやしい-----------------------------------
            int boundaryVertCountTrue = cutMesh.BoundaryVertIndices().Count;
            double[] gaussian = GaussianCurvature(cutMesh);
           
            for (int i = boundaryVertCount; i < boundaryVertCountTrue; i++)
            {
                gaussian[i] = 2 * Math.PI;
            }
            
            //double[] gaussian2 = new double[gaussian.Length];
            double[] k2 = GeodesicCurvature(cutMesh);
            double[] k = k2.Take(boundaryVertCount).ToArray();
            //----------------------------------あやしい終--------------------------------------

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

            return newVertices;
        }

        //ほぼNetBFF(CutMesh mesh, int boundaryVertCount)と同じ結果になる。趣旨もほぼ一緒
        public static List<Point3d> NetBFF2(CutMesh mesh, int boundaryVertCount)
        {
            // 1. メッシュのソート（外周ループを先頭に）
            CutMesh cutMesh = mesh.Sort(); // ※ここは前回の回答通り、外周が先頭0~boundaryVertCountに来ている前提

            // 2. ラプラシアン行列の構築（全頂点）
            double[,] A = BuildLaplace(cutMesh);

            // 3. 測地曲率と辺の長さを取得（外周部分のみ）
            // 可展面展開なので、GaussianCurvatureやScaleFactor(u)の計算はスキップしてOK（等長写像とみなす）
            double[] kAll = GeodesicCurvature(cutMesh);
            double[] k = kAll.Take(boundaryVertCount).ToArray(); // 外周分だけ

            double[] l = new double[boundaryVertCount];
            for (int i = 0; i < boundaryVertCount; i++)
            {
                Vector3d v = cutMesh.Vertices[i] - cutMesh.Vertices[(i + 1) % boundaryVertCount];
                l[i] = v.Length;
            }

            // 4. 外周の形状決定 (2次元座標を取得する！)
            // lnew = l (長さ保存) とします
            double[][] boundaryGamma = BestFitCurve2(l, l, k);

            // X座標とY座標の配列に分ける
            double[] boundaryX = new double[boundaryVertCount];
            double[] boundaryY = new double[boundaryVertCount];
            for (int i = 0; i < boundaryVertCount; i++)
            {
                boundaryX[i] = boundaryGamma[i][0];
                boundaryY[i] = boundaryGamma[i][1];
            }

            // 5. 内部（および穴）の座標決定
            // 行列の分割
            int innerVertCount = cutMesh.Vertices.Count - boundaryVertCount;
            double[,] Ainner = MatrixUtils.ExtractPartMatrix(A, boundaryVertCount, boundaryVertCount, innerVertCount, innerVertCount);
            double[,] Aib = MatrixUtils.ExtractPartMatrix(A, boundaryVertCount, 0, innerVertCount, boundaryVertCount);

            // ExtendCurveを2次元対応版に変更
            List<Point3d> newVertices = ExtendCurveV2(Ainner, Aib, boundaryX, boundaryY);

            return newVertices;
        }

        // 修正版 ExtendCurve
        public static List<Point3d> ExtendCurveV2(double[,] Ainner, double[,] Aib, double[] boundaryX, double[] boundaryY)
        {
            // --- X座標の計算 ---
            // Ainner * x_inner = -Aib * x_boundary
            double[] rhsX = MatrixUtils.Multiply(-1, MatrixUtils.Multiply(Aib, boundaryX));
            double[] innerX = SLEsSolvers.SolveWithMathNet(Ainner, rhsX);

            // --- Y座標の計算 ---
            // Ainner * y_inner = -Aib * y_boundary
            // ここが以前のコードと違う点です。Yも同じ重み行列で解きます。
            double[] rhsY = MatrixUtils.Multiply(-1, MatrixUtils.Multiply(Aib, boundaryY));
            double[] innerY = SLEsSolvers.SolveWithMathNet(Ainner, rhsY);

            // --- 統合 ---
            int boundaryCount = boundaryX.Length;
            int innerCount = innerX.Length;
            List<Point3d> newVertices = new List<Point3d>();

            // ソート順序に従って格納（外周 -> 内部・穴）
            for (int i = 0; i < boundaryCount; ++i)
            {
                newVertices.Add(new Point3d(boundaryX[i], boundaryY[i], 0));
            }
            for (int i = 0; i < innerCount; ++i)
            {
                newVertices.Add(new Point3d(innerX[i], innerY[i], 0));
            }

            return newVertices;
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





        //---------------------------------------------------BFF and CGLength------------------------------------------------------------

        //NetBFFした後にCGで長さ最適化
        public static List<Point3d> NetBFFandCGLength(CutMesh mesh, int boundaryVertCount)
        {
            CutMesh cutMesh = mesh.Sort();
            List<Point3d> newVertices = new List<Point3d> ();
            if (boundaryVertCount == -1) { newVertices = NetBFF(cutMesh); }
            else { newVertices = NetBFF2(cutMesh, boundaryVertCount); }

            //----------------------------------------ここから長さ最適化---------------------------------------------------
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

            return newVertices;
        }


        //NetBFFした後にCGNR法で長さ最適化
        public static CutMesh NetBFFandCGNRLength(CutMesh mesh, int boundaryVertCount, int iterations = 100)
        {
            CutMesh cutMesh = mesh.Sort();
            CutMesh pre3DMesh = cutMesh.Clone();

            List<Point3d> newVertices = new List<Point3d>();
            if (boundaryVertCount == -1) { newVertices = NetBFF(cutMesh); }
            else { newVertices = NetBFF2(cutMesh, boundaryVertCount); }
            List<List<int>> newDuplicatedVertIndices = Enumerable.Range(0, cutMesh.Vertices.Count)
                              .Select(i => new List<int> { i })
                              .ToList();
            CutMesh current2DMesh = new CutMesh(newVertices, pre3DMesh.Faces, pre3DMesh.Edges, newDuplicatedVertIndices);

            //----------------------------------------ここから長さ最適化---------------------------------------------------
            int vertCount = mesh.Vertices.Count;
            int n = vertCount * 2;
            //ｍは制約条件の個数。今回は長さ保存のみ
            int edgeEnergyCount = mesh.Edges.Count;
            int m = edgeEnergyCount;

            /*---------------------------------------最初のmeshにおいて知っておきたい情報を計算０００---------------------*/
            double[] initialLength = EdgeLength(pre3DMesh);

            /*------------------CGNR法のサイクル開始-------------------------------------------------------*/
            for (int uu = 0; uu < iterations; uu++)
            {
                List<MatrixElement> J_data = new List<MatrixElement>();
                //制約条件の式で求めた値の-1倍がb
                double[] b = new double[m];


                /*----------------------------------------------------Jacobiと関数値の算出０００-----------------------------------------------------------------------*/
                //まず長さのエネルギー関数のjacobiと関数値を計算
                var edgeLengthCons = EdgeLengthCons2(initialLength, current2DMesh, 1, 0);
                J_data.AddRange(edgeLengthCons.J_data);
                double[] consEdgeLength = edgeLengthCons.ConsValue;
                for (int i = 0; i < edgeEnergyCount; i++)
                {
                    b[i] = -consEdgeLength[i];
                }

                /*---------------------------------------CG法によってΔxを求める--------------------------------------*/
                //J * input,サイズ n のベクトルを受け取り、サイズ m のベクトルを返す
                Func<double[], double[]> multiplyJ = (input) =>
                {
                    double[] output = new double[m]; // 結果は制約数サイズ

                    // スパース行列の計算: ゼロじゃない要素だけループする
                    foreach (var elem in J_data)
                    {
                        // J * v の定義通り: 行ごとの積和
                        output[elem.Row] += elem.Val * input[elem.Col];
                    }
                    return output;
                };

                //J^T * input,サイズ m のベクトルを受け取り、サイズ n のベクトルを返す
                Func<double[], double[]> multiplyJT = (input) =>
                {
                    double[] output = new double[n]; // 結果は変数数サイズ

                    // 転置行列の計算: RowとColを入れ替えて考える
                    foreach (var elem in J_data)
                    {
                        // J^T * v なので、inputのインデックスは「Row」になる
                        output[elem.Col] += elem.Val * input[elem.Row];
                    }
                    return output;
                };

                double[] deltaX = new double[n]; // 初期値 0

                //maxItrはデフォルトで100、torelanceはデフォルトで1e-6
                SolveCGLS(m, n, multiplyJ, multiplyJT, b, deltaX);




                /*-------------------------------------------------求めたΔxを現在位置に足す０００---------------------------------*/
                for (int i = 0; i < vertCount; i++)
                {
                    Vector3d deltaVec = new Vector3d(deltaX[2 * i], deltaX[2 * i + 1], 0);
                    current2DMesh.Vertices[i] += deltaVec;
                }
            }
            return current2DMesh;
        }

        /*
        //NetBFFした後にCGNR法で長さ最適化with符号付面積>0の制約
        public static CutMesh NetBFFandLBFGS(CutMesh mesh, int boundaryVertCount, int iterations = 100)
        {
            CutMesh cutMesh = mesh.Sort();
            CutMesh pre3DMesh = cutMesh.Clone();

            double[,] A = BuildLaplace(cutMesh);
            //for (int i = 0; i < A.GetLength(0); ++i) A[i, i] += 1e-8;
            //DenseMatrix mat = DenseMatrix.OfArray(A);
            //Cholesky<double> cholA = mat.Cholesky();   // 内部で高速アルゴリズムを使用
            double[] kdiff = new double[boundaryVertCount];

            //------------------------------------あやしい-----------------------------------
            int boundaryVertCountTrue = cutMesh.BoundaryVertIndices().Count;
            double[] gaussian = GaussianCurvature(cutMesh);

            for (int i = boundaryVertCount; i < boundaryVertCountTrue; i++)
            {
                gaussian[i] = 0.5 * Math.PI;
            }

            //double[] gaussian2 = new double[gaussian.Length];
            double[] k2 = GeodesicCurvature(cutMesh);
            double[] k = k2.Take(boundaryVertCount).ToArray();
            //----------------------------------あやしい終--------------------------------------

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
            List<List<int>> newDuplicatedVertIndices = Enumerable.Range(0, cutMesh.Vertices.Count)
                              .Select(i => new List<int> { i })
                              .ToList();
            CutMesh current2DMesh = new CutMesh(newVertices, pre3DMesh.Faces, pre3DMesh.Edges, newDuplicatedVertIndices);



            //----------------------------------------ここから長さ最適化---------------------------------------------------
            int vertCount = mesh.Vertices.Count;
            int n = vertCount * 2;
            //ｍは制約条件の個数。今回は長さ保存のみ
            int edgeEnergyCount = mesh.Edges.Count;
            int m = edgeEnergyCount;

            //---------------------------------------最初のmeshにおいて知っておきたい情報を計算０００---------------------
            double[] initialLength = EdgeLength(pre3DMesh);

            //------------------CGNR法のサイクル開始-------------------------------------------------------
            for (int uu = 0; uu < iterations; uu++)
            {
                List<MatrixElement> J_data = new List<MatrixElement>();
                //制約条件の式で求めた値の-1倍がb
                double[] b = new double[m];


                //----------------------------------------------------Jacobiと関数値の算出０００-----------------------------------------------------------------------
                //まず長さのエネルギー関数のjacobiと関数値を計算
                var edgeLengthCons = EdgeLengthCons2(initialLength, current2DMesh, 1, 0);
                J_data.AddRange(edgeLengthCons.J_data);
                double[] consEdgeLength = edgeLengthCons.ConsValue;
                for (int i = 0; i < edgeEnergyCount; i++)
                {
                    b[i] = -consEdgeLength[i];
                }

                //---------------------------------------CG法によってΔxを求める--------------------------------------
                //J * input,サイズ n のベクトルを受け取り、サイズ m のベクトルを返す
                Func<double[], double[]> multiplyJ = (input) =>
                {
                    double[] output = new double[m]; // 結果は制約数サイズ

                    // スパース行列の計算: ゼロじゃない要素だけループする
                    foreach (var elem in J_data)
                    {
                        // J * v の定義通り: 行ごとの積和
                        output[elem.Row] += elem.Val * input[elem.Col];
                    }
                    return output;
                };

                //J^T * input,サイズ m のベクトルを受け取り、サイズ n のベクトルを返す
                Func<double[], double[]> multiplyJT = (input) =>
                {
                    double[] output = new double[n]; // 結果は変数数サイズ

                    // 転置行列の計算: RowとColを入れ替えて考える
                    foreach (var elem in J_data)
                    {
                        // J^T * v なので、inputのインデックスは「Row」になる
                        output[elem.Col] += elem.Val * input[elem.Row];
                    }
                    return output;
                };

                double[] deltaX = new double[n]; // 初期値 0

                //maxItrはデフォルトで100、torelanceはデフォルトで1e-6
                SolveCGLS(m, n, multiplyJ, multiplyJT, b, deltaX);




                //-------------------------------------------------求めたΔxを現在位置に足す０００---------------------------------
                for (int i = 0; i < vertCount; i++)
                {
                    Vector3d deltaVec = new Vector3d(deltaX[2 * i], deltaX[2 * i + 1], 0);
                    current2DMesh.Vertices[i] += deltaVec;
                }
            }
            //return current2DMesh;



            double[] x = GetCurrentPositions();

            // 1. エネルギー計算関数
            // E = (1/2) * ||R_len||^2 + (-μ * Σ ln(Area))
            Func<double[], double> calcEnergy = (currentX) =>
            {
                // A. 長さエネルギー (最小二乗)
                // エッジごとの (今の長さ - 自然長)^2 を合計
                double E_len = 0.0;
                foreach (var edge in edges)
                {
                    double len = MeasureLength(currentX, edge);
                    double diff = len - edge.RestLength;
                    E_len += 0.5 * diff * diff;
                }

                // B. 面積バリアエネルギー
                double E_area = 0.0;
                double mu = 0.01; // バリアの強さ
                foreach (var tri in triangles)
                {
                    double area = MeasureSignedArea(currentX, tri);
                    if (area <= 0) return double.PositiveInfinity; // 既に死んでいる場合
                    E_area += -mu * Math.Log(area);
                }

                return E_len + E_area;
            };

            // 2. 勾配計算関数 (ここが前の J^T R に相当！)
            // g = J^T * R + ∇E_area
            Func<double[], double[]> calcGradient = (currentX) =>
            {
                double[] g = new double[n]; // 0初期化

                // A. 長さ勾配 (J^T * R)
                // 行列を使わず、バネの力を直接足し込む
                foreach (var edge in edges)
                {
                    // ... (前の回答の g_len 計算ロジック) ...
                    // force = (currentLen - restLen) * direction
                    // g[v1] += force; g[v2] -= force;
                }

                // B. 面積勾配 (-μ/A * ∇A)
                double mu = 0.01;
                foreach (var tri in triangles)
                {
                    double area = MeasureSignedArea(currentX, tri);
                    // areaが0に近いと発散するので注意
                    double coeff = -mu / area;

                    // ∇A (面積の勾配) を計算して足し込む
                    // 2Dの場合、三角形p1,p2,p3の面積A = 0.5*((x2-x1)(y3-y1) - (x3-x1)(y2-y1))
                    // これを各座標で偏微分したベクトルを coeff 倍して g に足す
                }

                return g;
            };

            // 3. 幾何チェック関数 (Line Search用)
            Func<double[], bool> isValidGeometry = (currentX) =>
            {
                foreach (var tri in triangles)
                {
                    if (MeasureSignedArea(currentX, tri) <= 1e-9)
                        return false; // 面積が小さすぎる or 裏返ってる
                }
                return true;
            };

            // --- 実行 ---
            Solve(n, x, calcEnergy, calcGradient, isValidGeometry, maxIter: 50, m: 10);






        }
        */

        // 履歴データの構造体
        private class HistoryItem
        {
            public double[] s;   // x の変化量 (delta X)
            public double[] y;   // g の変化量 (delta Gradient)
            public double rho;   // 1 / (y^T * s)
        }

        /// <summary>
        /// L-BFGS法による最適化実行
        /// </summary>
        /// <param name="n">変数の数 (頂点数 * 2 or 3)</param>
        /// <param name="x">初期位置 (兼 結果格納用)</param>
        /// <param name="calcEnergy">現在の x における全エネルギー値を返す関数</param>
        /// <param name="calcGradient">現在の x における全勾配ベクトル(g)を返す関数</param>
        /// <param name="isValidGeometry">幾何学的制約(裏返り等)をチェックする関数 (LineSearch用)</param>
        /// <param name="m">履歴サイズ (通常 5〜20)</param>
        public static void Solve(
            int n,
            double[] x,
            Func<double[], double> calcEnergy,
            Func<double[], double[]> calcGradient,
            Func<double[], bool> isValidGeometry,
            int maxIter = 100,
            int m = 10,
            double tolerance = 1e-6)
        {
            // 履歴リスト
            LinkedList<HistoryItem> history = new LinkedList<HistoryItem>();

            // 初期の勾配とエネルギー
            double[] g = calcGradient(x);
            double energy = calcEnergy(x);

            // 勾配の大きさチェック (最初から収束しているか？)
            if (Dot(g, g) < tolerance * tolerance) return;

            // メインループ
            for (int k = 0; k < maxIter; k++)
            {
                // 1. 探索方向 d = -H * g を求める (Two-Loop Recursion)
                double[] d = ComputeDirection(g, history, n);

                // 2. ラインサーチ (バックトラッキング)
                // 適切なステップ幅 alpha を探して x を更新する
                double alpha = LineSearch(n, x, d, g, energy, calcEnergy, isValidGeometry);

                // 更新できなかったら終了 (局所解)
                if (alpha == 0.0) break;

                // 3. 更新前の情報を保持
                double[] x_old = (double[])x.Clone();
                double[] g_old = (double[])g.Clone();

                // 4. 位置更新: x = x + alpha * d
                for (int i = 0; i < n; i++) x[i] += alpha * d[i];

                // 5. 新しい状態の計算
                double newEnergy = calcEnergy(x);
                double[] newG = calcGradient(x);

                // 6. 履歴の更新 (s = x_new - x_old, y = g_new - g_old)
                double[] s = new double[n];
                double[] y = new double[n];
                for (int i = 0; i < n; i++)
                {
                    s[i] = x[i] - x_old[i];
                    y[i] = newG[i] - g_old[i];
                }

                // 曲率条件 (ys > 0) のチェック
                // これが正でないとヘッセ行列が正定値にならない(=不安定)ので履歴に入れない
                double ys = Dot(y, s);
                if (ys > 1e-10)
                {
                    var item = new HistoryItem { s = s, y = y, rho = 1.0 / ys };
                    history.AddLast(item);
                    if (history.Count > m) history.RemoveFirst(); // 古い履歴を捨てる
                }

                // 次のループへの準備
                energy = newEnergy;
                g = newG;

                // 収束判定
                if (Dot(g, g) < tolerance * tolerance) break;
            }
        }

        // --- Two-Loop Recursion (L-BFGSの核心) ---
        // 過去の s, y を使って、擬似的に H^(-1) * g を計算する
        private static double[] ComputeDirection(double[] g, LinkedList<HistoryItem> history, int n)
        {
            // q = g (コピー)
            double[] q = (double[])g.Clone();

            // alphas配列 (履歴の数だけ必要)
            double[] alphas = new double[history.Count];

            // [Backward Loop] 最新 -> 過去
            int idx = history.Count - 1;
            foreach (var item in history.Reverse())
            {
                // alpha = rho * (s^T * q)
                double alpha = item.rho * Dot(item.s, q);
                alphas[idx] = alpha;

                // q = q - alpha * y
                for (int i = 0; i < n; i++) q[i] -= alpha * item.y[i];
                idx--;
            }

            // [Scaling] 初期のヘッセ行列 H0 の推定
            // r = H0 * q
            double[] r = (double[])q.Clone();
            if (history.Count > 0)
            {
                var last = history.Last.Value;
                // gamma = (s^T y) / (y^T y)
                double gamma = Dot(last.s, last.y) / Dot(last.y, last.y);
                for (int i = 0; i < n; i++) r[i] *= gamma;
            }

            // [Forward Loop] 過去 -> 最新
            idx = 0;
            foreach (var item in history)
            {
                double alpha = alphas[idx];
                // beta = rho * (y^T * r)
                double beta = item.rho * Dot(item.y, r);

                // r = r + s * (alpha - beta)
                double coeff = alpha - beta;
                for (int i = 0; i < n; i++) r[i] += coeff * item.s[i];
                idx++;
            }

            // 方向 d = -r (勾配と逆向き)
            for (int i = 0; i < n; i++) r[i] = -r[i];

            return r; // これが d
        }

        // --- Line Search (Backtracking / Armijo) ---
        private static double LineSearch(
            int n,
            double[] x,
            double[] d,
            double[] g,
            double currentEnergy,
            Func<double[], double> calcEnergy,
            Func<double[], bool> isValidGeometry)
        {
            double alpha = 1.0;
            double c1 = 1e-4;
            double g_dot_d = Dot(g, d); // 勾配方向への射影 (必ず負になるはず)

            // もし d が上昇方向ならリセット（数値誤差対策）
            if (g_dot_d > 0) return 0.0;

            double[] x_try = new double[n];

            // 最大20回くらいトライ
            for (int i = 0; i < 20; i++)
            {
                // x_try = x + alpha * d
                for (int k = 0; k < n; k++) x_try[k] = x[k] + alpha * d[k];

                // 1. 幾何学的チェック (裏返り検知)
                // バリア関数を使う場合は、ここが重要。裏返ったらエネルギー計算すら危険なので即NG。
                if (!isValidGeometry(x_try))
                {
                    alpha *= 0.5;
                    continue;
                }

                // 2. エネルギー減少チェック (Armijo条件)
                // E_new <= E_old + c1 * alpha * (g^T d)
                double newEnergy = calcEnergy(x_try);

                if (newEnergy <= currentEnergy + c1 * alpha * g_dot_d)
                {
                    return alpha; // 採用
                }

                // ダメなら歩幅を半分にする
                alpha *= 0.5;
            }

            return 0.0; // 失敗
        }

        // ヘルパー: 内積
        private static double Dot(double[] a, double[] b)
        {
            double sum = 0.0;
            for (int i = 0; i < a.Length; i++) sum += a[i] * b[i];
            return sum;
        }





        /*---------------------------------------------------CGNR法に依る展開図作成、穴ありなどに対応可能-----------------------------------------------------------*/
        public static CutMesh NetCGNR(CutMesh mesh, int iterations, double[] w)
        {
            /*------------------------------------------------------重みづけ０００-----------------------------------*/
            //w0がエッジの重みづけ、w1がz軸との内積の方の重みづけ
            double w0 = w[0];
            double w1 = w[1];

            /*---------------------------------------n(変数の数),m(constraintsの数)の設定０００-------------------------*/
            int vertCount = mesh.Vertices.Count;
            int n = vertCount * 3;
            //ｍは制約条件の個数。今回は長さ保存最初、ｚ正方向都の内積後が後
            int edgeEnergyCount = mesh.Edges.Count;
            int faceCount = mesh.Faces.GetLength(0);
            int faceEnergyCount = faceCount;
            int m = edgeEnergyCount + faceEnergyCount;

            /*---------------------------------------最初のmeshにおいて知っておきたい情報を計算０００---------------------*/
            double[] initialLength = EdgeLength(mesh);

            /*------------------CGNR法のサイクル開始-------------------------------------------------------*/
            for (int u = 0; u < iterations; u++)
            {
                List<MatrixElement> J_data = new List<MatrixElement>();
                //制約条件の式で求めた値の-1倍がb
                double[] b = new double[m];


                /*----------------------------------------------------Jacobiと関数値の算出０００-----------------------------------------------------------------------*/
                //まず長さのエネルギー関数のjacobiと関数値を計算
                var edgeLengthCons = EdgeLengthCons(initialLength, mesh, w0, 0);
                J_data.AddRange(edgeLengthCons.J_data);
                double[] consEdgeLength = edgeLengthCons.ConsValue;
                for (int i = 0; i < edgeEnergyCount; i++)
                {
                    b[i] = -consEdgeLength[i];
                }

                //次にfaceNormalとz軸正方向の単位ベクトルの内積のエネルギー関数のjacobiと関数値を計算
                var faceNormalCons = FaceNormalCons(mesh, w1, edgeEnergyCount);
                J_data.AddRange(faceNormalCons.J_data);
                double[] consFaceNormal = faceNormalCons.ConsValue;
                for (int i = 0; i < faceEnergyCount; i++)
                {
                    b[edgeEnergyCount + i] = -consFaceNormal[i];
                }




                /*---------------------------------------CG法によってΔxを求める--------------------------------------*/
                //J * input,サイズ n のベクトルを受け取り、サイズ m のベクトルを返す
                Func<double[], double[]> multiplyJ = (input) =>
                {
                    double[] output = new double[m]; // 結果は制約数サイズ

                    // スパース行列の計算: ゼロじゃない要素だけループする
                    foreach (var elem in J_data)
                    {
                        // J * v の定義通り: 行ごとの積和
                        output[elem.Row] += elem.Val * input[elem.Col];
                    }
                    return output;
                };

                //J^T * input,サイズ m のベクトルを受け取り、サイズ n のベクトルを返す
                Func<double[], double[]> multiplyJT = (input) =>
                {
                    double[] output = new double[n]; // 結果は変数数サイズ

                    // 転置行列の計算: RowとColを入れ替えて考える
                    foreach (var elem in J_data)
                    {
                        // J^T * v なので、inputのインデックスは「Row」になる
                        output[elem.Col] += elem.Val * input[elem.Row];
                    }
                    return output;
                };

                double[] deltaX = new double[n]; // 初期値 0

                //maxItrはデフォルトで100、torelanceはデフォルトで1e-6
                SolveCGLS(m, n, multiplyJ, multiplyJT, b, deltaX);




                /*-------------------------------------------------求めたΔxを現在位置に足す０００---------------------------------*/
                for (int i = 0; i < vertCount; i++)
                {
                    Vector3d deltaVec = new Vector3d(deltaX[3 * i], deltaX[3 * i + 1], deltaX[3 * i + 2]);
                    mesh.Vertices[i] += deltaVec;
                }
            }
            return mesh;
        }

        //エッジの長さのエネルギー関数のjacobi(List<MatrixElement>)と関数自体の値(double[])、重みづけw
        //ほかのconstraintsの行が前にあるときはその行分をoffsetの値とする
        public static (List<MatrixElement> J_data, double[] ConsValue) EdgeLengthCons(double[] initialLength, CutMesh currentMesh, double w, int offset)
        {
            List<MatrixElement> J_data = new List<MatrixElement>();
            double[] currentLength = EdgeLength(currentMesh);
            int count = currentLength.Length;
            double[] ConsValue = new double[count];
            //まず長さのエネルギー関数のjacobiを計算
            for (int i = 0; i < count; i++)
            {
                //まず長さのエネルギー関数の値を計算
                double length = currentLength[i];
                ConsValue[i] = w * (length - initialLength[i]);

                //次に長さのエネルギー関数のjacobiを計算
                int[] verts = currentMesh.Edges[i];
                int vert1 = verts[0];
                int vert2 = verts[1];
                Vector3d vector = w * (currentMesh.Vertices[vert1] - currentMesh.Vertices[vert2]) / length;
                J_data.Add(new MatrixElement(i + offset, vert1 * 3, vector.X));
                J_data.Add(new MatrixElement(i + offset, vert1 * 3 + 1, vector.Y));
                J_data.Add(new MatrixElement(i + offset, vert1 * 3 + 2, vector.Z));
                J_data.Add(new MatrixElement(i + offset, vert2 * 3, -vector.X));
                J_data.Add(new MatrixElement(i + offset, vert2 * 3 + 1, -vector.Y));
                J_data.Add(new MatrixElement(i + offset, vert2 * 3 + 2, -vector.Z));
            }
            return (J_data, ConsValue);
        }

        public static (List<MatrixElement> J_data, double[] ConsValue) EdgeLengthCons2(double[] initialLength, CutMesh currentMesh, double w, int offset)
        {
            List<MatrixElement> J_data = new List<MatrixElement>();
            double[] currentLength = EdgeLength(currentMesh);
            int count = currentLength.Length;
            double[] ConsValue = new double[count];

            for (int i = 0; i < count; i++)
            {
                //まず長さのエネルギー関数の値を計算
                double length = currentLength[i];
                ConsValue[i] = w * (length - initialLength[i]);

                //次に長さのエネルギー関数のjacobiを計算
                int[] verts = currentMesh.Edges[i];
                int vert1 = verts[0];
                int vert2 = verts[1];
                Vector3d vector = w * (currentMesh.Vertices[vert1] - currentMesh.Vertices[vert2]) / length;
                J_data.Add(new MatrixElement(i + offset, vert1 * 2, vector.X));
                J_data.Add(new MatrixElement(i + offset, vert1 * 2 + 1, vector.Y));
                J_data.Add(new MatrixElement(i + offset, vert2 * 2, -vector.X));
                J_data.Add(new MatrixElement(i + offset, vert2 * 2 + 1, -vector.Y));
            }
            return (J_data, ConsValue);
        }
        //faceNormalをn(単位ベクトルとは限らない)として、n*z-|n|が関数の値
        public static (List<MatrixElement> J_data, double[] ConsValue) FaceNormalCons(CutMesh currentMesh, double w, int offset)
        {
            int count = currentMesh.Faces.GetLength(0);
            List<List<int>> faceRecord = FaceNormalOrders(currentMesh);
            //サブリストがfaceのID順に並ぶようにソートする
            faceRecord = faceRecord.OrderBy(subList => subList[0]).ToList();
            Vector3d[] faceNormals = FaceNormal(currentMesh);
            double[] consValue = new double[count];
            List<MatrixElement> J_data = new List<MatrixElement>();
            for (int i = 0; i < count; i++)
            {
                Vector3d faceNormal = faceNormals[i];
                double normalLength = faceNormal.Length;
                consValue[i] = w * (faceNormal.Z - normalLength);

                int[] edge1 = currentMesh.Edges[faceRecord[i][1]];
                int[] edge2 = currentMesh.Edges[faceRecord[i][2]];
                int v0index = edge1.Intersect(edge2).First();
                int v1index = edge1.First(f => f != v0index);
                int v2index = edge2.First(f => f != v0index);
                Point3d v0 = currentMesh.Vertices[v0index];
                Point3d v1 = currentMesh.Vertices[v1index];
                Point3d v2 = currentMesh.Vertices[v2index];
                Vector3d vec0 = v1 - v2;
                Vector3d vec1 = v2 - v0;
                Vector3d vec2 = v0 - v1;
                double x0jacobi = w * (faceNormal.Y * vec0.Z + (normalLength - faceNormal.Z) * vec0.Y) / normalLength;
                double y0jacobi = -w * (faceNormal.X * vec0.Z + (normalLength - faceNormal.Z) * vec0.X) / normalLength;
                double z0jacobi = w * (faceNormal.X * vec0.Y - faceNormal.Y * vec0.X) / normalLength;

                double x1jacobi = w * (faceNormal.Y * vec1.Z + (normalLength - faceNormal.Z) * vec1.Y) / normalLength;
                double y1jacobi = -w * (faceNormal.X * vec1.Z + (normalLength - faceNormal.Z) * vec1.X) / normalLength;
                double z1jacobi = w * (faceNormal.X * vec1.Y - faceNormal.Y * vec1.X) / normalLength;

                double x2jacobi = w * (faceNormal.Y * vec2.Z + (normalLength - faceNormal.Z) * vec2.Y) / normalLength;
                double y2jacobi = -w * (faceNormal.X * vec2.Z + (normalLength - faceNormal.Z) * vec2.X) / normalLength;
                double z2jacobi = w * (faceNormal.X * vec2.Y - faceNormal.Y * vec2.X) / normalLength;

                J_data.Add(new MatrixElement(i + offset, v0index * 3, x0jacobi));
                J_data.Add(new MatrixElement(i + offset, v0index * 3 + 1, y0jacobi));
                J_data.Add(new MatrixElement(i + offset, v0index * 3 + 2, z0jacobi));
                J_data.Add(new MatrixElement(i + offset, v1index * 3, x1jacobi));
                J_data.Add(new MatrixElement(i + offset, v1index * 3 + 1, y1jacobi));
                J_data.Add(new MatrixElement(i + offset, v1index * 3 + 2, z1jacobi));
                J_data.Add(new MatrixElement(i + offset, v2index * 3, x2jacobi));
                J_data.Add(new MatrixElement(i + offset, v2index * 3 + 1, y2jacobi));
                J_data.Add(new MatrixElement(i + offset, v2index * 3 + 2, z2jacobi));

            }
            return (J_data, consValue);
        }

        public static CutMesh CutMeshNetLBFGS(CutMesh cutMesh, double w0, double w1)
        {
            
            CutMesh newMesh = cutMesh.Clone();
            List<Line> lines = new List<Line>();
            double[] initialLength = EdgeLength(cutMesh);
            Vector3d[] normals = FaceNormal(cutMesh);
            double[] initialAbsArea = new double[normals.Length];
            for (int i = 0; i < normals.Length; i++)
            {
                initialAbsArea[i] = normals[i].Length;
            }

            Mesh mesh = cutMesh.ConvertToMesh2();
            Polyline[] nakedEdges = mesh.GetNakedEdges();
            Polyline outline = nakedEdges[0];
            double[][] newTopoVertices2D = NetTools.TutteTopoVertices(mesh, outline);
            Point3d[] newTopoVertices = PtCrvTools.Convert2Dto3D(newTopoVertices2D);

            //更新後のメッシュを作成
            Rhino.Geometry.Mesh tutteMesh = MeshDataTools.MakeMesh(mesh, newTopoVertices);
            for (int i = 0; i < newMesh.DuplicatedVertIndices.Count; i++)
            {
                List<int> vertGroup = newMesh.DuplicatedVertIndices[i];
                foreach (int vert in vertGroup)
                {
                    newMesh.Vertices[vert] = tutteMesh.Vertices[i];
                }
            }

            //newMesh.Vertices = NetTools.NetBFF2(cutMesh,outerBoundaryVertCount);
            newMesh.DuplicatedVertIndices = Enumerable.Range(0, cutMesh.Vertices.Count)
                              .Select(i => new List<int> { i })
                              .ToList();

            //----------------------------------------------以下newMeshに対して長さ最適化かつ裏返り防止LBFGS-----------------------------
            int vertCount = newMesh.Vertices.Count;
            List<Point3d> points = newMesh.Vertices;
            int variableCount = vertCount * 2;
            var system = new SimulationSystem(variableCount);

            // 初期座標のセット
            for (int i = 0; i < vertCount; i++)
            {
                system.X[2 * i] = points[i].X;
                system.X[2 * i + 1] = points[i].Y;
            }

            // データの準備
            List<int[]> edges = newMesh.Edges;
            int[,] facesWithOrder = FacesWithOrder(newMesh);

            // --- 項目の追加 ---

            //----------------------------------等式の制約条件はこのように加える-----------------------------------------
            // 1. 長さ保存 (重み: 1.0)
            system.AddEnergy(new EdgeLengthEnergy2D(w0, edges, initialLength));

            //system.AddEnergy(new AreaEnergy2D(w[1],facesWithOrder,initialAbsArea));


            //----------------------------不等式の制約条件はこのように加える-----------------------------------------------

            var areaConstraint = new AreaBarrier(w1, facesWithOrder);

            // 同じものを「エネルギー」としても「制約」としても登録する
            system.AddEnergy(areaConstraint);
            system.AddConstraint(areaConstraint);
            //----------------------------不等式の制約条件はこの二つセットで加える 終-----------------------------------------------

            system.Step(10000);

            /*
            //ほとんど長さ保存ができているのでこの状態でさらに長さ最適化を行って微調整を行う
            var system2 = new SimulationSystem(variableCount);
            for (int i = 0; i < variableCount; i++)
            {
                system2.X[i] = system.X[i];
            }
            system2.AddEnergy(new EdgeLengthEnergy2D(1, edges, initialLength));
            system2.Step();
            */

            //求めたxをnewMeshのverticesに適用する
            for (int i = 0; i < vertCount; i++)
            {
                newMesh.Vertices[i] = new Point3d(system.X[2 * i], system.X[2 * i + 1], 0);
            }

            return newMesh;
        }

        public static CutMesh CutMeshNetLBFGS2(CutMesh cutMesh2, double w0, double w1)
        {
            CutMesh cutMesh = cutMesh2.Clone();
            cutMesh = cutMesh.Sort();
            cutMesh.DuplicatedVertIndices = cutMesh.DuplicatedVertIndices.Select(list => list.OrderBy(x => x).ToList()).OrderBy(list => list[0]).ToList();
            double[] initialLength = EdgeLength(cutMesh);

            List<int> boundary = cutMesh.BoundaryEdgeIndices();
            int boundaryCount = boundary.Count;
            List<List<int>> boundaryGroup = new List<List<int>>();
            List<int> group = new List<int> { 0 };
            for (int i = 0; i < boundaryCount - 1; i++)
            {
                HashSet<int> visited = new HashSet<int>();
                visited.Add(cutMesh.Edges[boundary[i]][0]);
                visited.Add(cutMesh.Edges[boundary[i]][1]);
                visited.Add(cutMesh.Edges[boundary[i + 1]][0]);
                visited.Add(cutMesh.Edges[boundary[i + 1]][1]);
                if (visited.Count <= 3) { group.Add(i + 1); }
                else { boundaryGroup.Add(group); group = new List<int> { i + 1 }; }
            }
            boundaryGroup.Add(group);

            //外部境界線は考えなくてよい。ここでは一番エッジ本数が長いものを外部境界線としている
            List<List<int>> boundaryVertsGroup = new List<List<int>>();
            List<double> area = new List<double>();

            for (int i = 0; i < boundaryGroup.Count; i++)
            {
                var loop = boundaryGroup[i];
                var verts = cutMesh.EdgesToVerts(loop);
                boundaryVertsGroup.Add(verts);

                // 面積計算
                var pointsList = verts.Select(v => cutMesh.Vertices[v]).ToList();
                var polyline = new Polyline(pointsList);
                var mesh = Mesh.CreateFromClosedPolyline(polyline);
                if (mesh == null) { area.Add(0); }
                else { area.Add(Math.Abs(AreaMassProperties.Compute(mesh).Area)); };
            }

            // 最大面積のインデックス
            int maxIndex = area.IndexOf(area.Max());

            List<int> outerVerts = boundaryVertsGroup[maxIndex];
            // そのループだけ削除
            boundaryVertsGroup.RemoveAt(maxIndex);


            int[] vertOrderInDup = cutMesh.VertOrderInDup();
            List<List<int>> holes = new List<List<int>>();
            foreach (List<int> loop in boundaryVertsGroup)
            {
                List<bool> isDup = new List<bool>();
                List<int> loopClone = new List<int>(loop);
                foreach (int vert in loop)
                {
                    int dupIndex = vertOrderInDup[vert];
                    int dupVertCount = cutMesh.DuplicatedVertIndices[dupIndex].Count;
                    if (dupVertCount == 1) { isDup.Add(false); }
                    else { isDup.Add(true); }
                }
                if (isDup.All(x => x == false))
                {
                    holes.Add(loop);
                    continue;
                }

                int j = 0;
                List<bool> isDupClone = new List<bool>(isDup);
                while (j < isDup.Count && !isDup[j])
                {
                    loopClone.RemoveAt(0);
                    loopClone.Add(loop[j]);
                    isDupClone.RemoveAt(0);
                    isDupClone.Add(false);
                    j++;
                }
                isDupClone.Add(true);
                loopClone.Add(loopClone[0]);

                var result = new List<List<int>>();
                int? start = null;
                for (int i = 0; i < isDupClone.Count; i++)
                {
                    if (isDupClone[i])
                    {
                        if (start != null)
                        {
                            // start と i の間の false のインデックスを追加
                            List<int> block = Enumerable.Range(start.Value, i - start.Value).ToList();
                            List<int> vertBlock = new List<int>();
                            if (block.Count > 3) { foreach (int blockIndex in block) { vertBlock.Add(loopClone[blockIndex]); } holes.Add(vertBlock); }
                        }
                        start = i; // 新しい t の位置を記録
                    }
                }
            }

            //holeを塞ぐために付け足すものをリストアップ
            List<Point3d> addVerts = new List<Point3d>();
            List<int[]> addEdges = new List<int[]>();
            List<int[]> addFaces = new List<int[]>();
            int lastVertIndex = cutMesh.Vertices.Count - 1;
            foreach (List<int> hole in holes)
            {
                Point3d center = new Point3d();
                lastVertIndex += 1;
                for (int i = 0; i < hole.Count; i++)
                {
                    int vert = hole[i];
                    center += cutMesh.Vertices[vert];
                    addEdges.Add(new int[] { vert, lastVertIndex });
                    if (i == hole.Count - 1) 
                    {
                        if (vertOrderInDup[vert] != vertOrderInDup[hole[0]]) { addFaces.Add(new int[] { vert, hole[0], lastVertIndex }); continue; }
                        else { continue; } 
                    }
                    addFaces.Add(new int[] { vert, hole[i + 1], lastVertIndex });
                }
                center /= hole.Count;
                addVerts.Add(center);
                cutMesh.DuplicatedVertIndices.Add(new List<int> { lastVertIndex });
            }
            cutMesh.Vertices.AddRange(addVerts);
            cutMesh.Edges.AddRange(addEdges);
            int originalCount = cutMesh.Faces.GetLength(0);
            int addCount = addFaces.Count;

            int[,] faces = new int[originalCount + addCount, 3];

            // 元の faces をコピー
            for (int i = 0; i < originalCount; i++)
            {
                faces[i, 0] = cutMesh.Faces[i, 0];
                faces[i, 1] = cutMesh.Faces[i, 1];
                faces[i, 2] = cutMesh.Faces[i, 2];
            }

            // addFaces をコピー
            for (int i = 0; i < addCount; i++)
            {
                faces[originalCount + i, 0] = addFaces[i][0];
                faces[originalCount + i, 1] = addFaces[i][1];
                faces[originalCount + i, 2] = addFaces[i][2];
            }
            cutMesh.Faces = faces;
            cutMesh.ReloadVertexToEdgesCache();
            cutMesh.ReloadEdgeToFacesCache();
            cutMesh.ReloadVertexToFacesCache();

            vertOrderInDup = cutMesh.VertOrderInDup();
            var newMeshSet = cutMesh.ConvertToNoSlitCutMesh2(outerVerts);
            CutMesh newMesh = newMeshSet.cutMesh;
            int[] vertToNoSlitVert = newMeshSet.vertToNoSlitVert;
            newMesh.Vertices = NetBFF(newMesh);
            for (int i = 0; i < cutMesh.Vertices.Count; i++)
            {
                cutMesh.Vertices[i] = newMesh.Vertices[vertToNoSlitVert[i]];
            }

            cutMesh.Vertices.RemoveRange(cutMesh.Vertices.Count - addVerts.Count, addVerts.Count);
            cutMesh.Edges.RemoveRange(cutMesh.Edges.Count - addEdges.Count, addEdges.Count);
            faces = new int[originalCount, 3];
            for (int i = 0; i < originalCount; i++)
            {
                faces[i, 0] = cutMesh.Faces[i, 0];
                faces[i, 1] = cutMesh.Faces[i, 1];
                faces[i, 2] = cutMesh.Faces[i, 2];
            }
            cutMesh.Faces = faces;
            cutMesh.ReloadVertexToEdgesCache();
            cutMesh.ReloadEdgeToFacesCache();
            cutMesh.ReloadVertexToFacesCache();

            //----------------------------------------------以下newMeshに対して長さ最適化かつ裏返り防止LBFGS-----------------------------
            int vertCount = cutMesh.Vertices.Count;
            List<Point3d> points = cutMesh.Vertices;
            int variableCount = vertCount * 2;
            var system = new SimulationSystem(variableCount);

            // 初期座標のセット
            for (int i = 0; i < vertCount; i++)
            {
                system.X[2 * i] = points[i].X;
                system.X[2 * i + 1] = points[i].Y;
            }

            // データの準備
            List<int[]> edges = cutMesh.Edges;
            int[,] facesWithOrder = FacesWithOrder(cutMesh);

            // --- 項目の追加 ---

            //----------------------------------等式の制約条件はこのように加える-----------------------------------------
            // 1. 長さ保存 (重み: 1.0)
            system.AddEnergy(new EdgeLengthEnergy2D(w0, edges, initialLength));

            //system.AddEnergy(new AreaEnergy2D(w[1],facesWithOrder,initialAbsArea));


            //----------------------------不等式の制約条件はこのように加える-----------------------------------------------

            var areaConstraint = new AreaBarrier(w1, facesWithOrder);

            // 同じものを「エネルギー」としても「制約」としても登録する
            system.AddEnergy(areaConstraint);
            system.AddConstraint(areaConstraint);
            //----------------------------不等式の制約条件はこの二つセットで加える 終-----------------------------------------------

            system.Step(200);

            /*
            //ほとんど長さ保存ができているのでこの状態でさらに長さ最適化を行って微調整を行う
            var system2 = new SimulationSystem(variableCount);
            for (int i = 0; i < variableCount; i++)
            {
                system2.X[i] = system.X[i];
            }
            system2.AddEnergy(new EdgeLengthEnergy2D(1, edges, initialLength));
            system2.Step();
            */

            //求めたxをnewMeshのverticesに適用する
            for (int i = 0; i < vertCount; i++)
            {
                cutMesh.Vertices[i] = new Point3d(system.X[2 * i], system.X[2 * i + 1], 0);
            }

            cutMesh.DuplicatedVertIndices = Enumerable.Range(0, cutMesh.Vertices.Count).Select(i => new List<int> { i }).ToList();
            return cutMesh;
        }
    }
}
