using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra.Double.Solvers;
using MathNet.Numerics.LinearAlgebra.Factorization;
using System;
using System.Linq;

namespace Hagoromo.MathTools
{
    //連立一次方程式を解くクラス
    public static class SLEsSolvers
    {
        //連立一次方程式Ax=bをLDL分解で解く,ただしAは実対称行列の下三角を上の行から順に行ずつで並べた配列、求めたxを返す
        public static double[] SolveByLDL(double[] A, double[] b)
        {
            int n = b.Length;//nは行列のサイズ//
            double[] U = new double[n * (n + 1) / 2];//Uは上三角成分のみを下の行から順に行ずつで並べた配列//
            double[] L = new double[n * (n - 1) / 2];//Lは下三角成分(1のところも省く)を上の行から順に行ずつで並べた配列//

            //次のfor文で帰納的にU,Lを求めるが最初にその初項を設定//
            int t = n * (n - 1) / 2;
            for (int j = 1; j <= n; j++)
            {
                U[t + j - 1] = A[j * (j - 1) / 2]; //u_1j(=a_1j) =a_j1//
                                                   // U[t] = Math.Abs(U[t]) < 1e-10 ? 0.0 : U[t];
                L[j * (j - 3) / 2 + 1] = U[t + j - 1] / U[t]; //l_j1=u_1j/u_11//
            }

            //U,Lをそれぞれ求める//
            double s = 0;
            int id = 0;
            for (int i = 2; i <= n; i++)
            {
                for (int j = i; j <= n; j++)
                {
                    s = 0;
                    for (int k = 1; k <= i - 1; k++)
                    {
                        s += L[i * (i - 3) / 2 + k] * U[(n - k) * (n - k + 1) / 2 + j - k];
                    }
                    id = (n - i) * (n - i + 1) / 2 + j - i;
                    U[id] = A[j * (j - 1) / 2 + i - 1] - s;//u_ijを求める//

                    if (i != j)
                    {
                        //U[(n - i) * (n - i + 1) / 2] = Math.Abs(U[(n - i) * (n - i + 1) / 2]) < 1e-10 ? 0.0 : U[(n - i) * (n - i + 1) / 2];
                        L[j * (j - 3) / 2 + i] = U[id] / U[(n - i) * (n - i + 1) / 2];//l_jiを求める//
                    }
                }
            }

            //ガウスの前進消去法でyを求める、yが順に求まり次第、メモリ削減のためにそれをbに上書きしていく//
            //y1=b1なので初項は省略できる//
            for (int i = 2; i <= n; i++)
            {
                s = 0;
                for (int j = 1; j <= i - 1; j++)
                {
                    s += L[i * (i - 3) / 2 + j] * b[j - 1];
                }
                b[i - 1] -= s;
            }

            //後退代入でxを求める、xが後ろから順に求まり次第、メモリ削減のためにそれをbに上書きしていく//
            //U[0] = Math.Abs(U[0]) < 1e-10 ? 0.0 : U[0];
            b[n - 1] = b[n - 1] / U[0];
            for (int i = 1; i <= n - 1; i++)
            {
                s = 0;
                for (int j = n - i + 1; j <= n; j++)
                {
                    s += U[i * (i + 3) / 2 + j - n] * b[j - 1];
                }
                //U[i * (i + 1) / 2] = Math.Abs(U[i * (i + 1) / 2]) < 1e-10 ? 0.0 : U[i * (i + 1) / 2];
                b[n - i - 1] = (b[n - i - 1] - s) / U[i * (i + 1) / 2];
            }
            return b;
        }

        public static double[] SolveSparseLinearSystem(double[][] A, double[] B)
        {
            int rowCount = A.Length;
            int colCount = A[0].Length;

            // 疎行列として A を構築
            var sparseMatrix = SparseMatrix.OfIndexed(rowCount, colCount,
                from i in Enumerable.Range(0, rowCount)
                from j in Enumerable.Range(0, colCount)
                where A[i][j] != 0
                select Tuple.Create(i, j, A[i][j])
            );

            // ベクトル B を作成
            var bVector = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.DenseOfArray(B);

            // ソルバーで解く（LU 分解など）
            var solver = sparseMatrix.Solve(bVector);

            return solver.ToArray();
        }

        public static double[] CholeskyDecompose(double[] A, int n)
        {
            // A: 正定値対称行列の下三角を格納した配列
            // n: 行列サイズ
            // 戻り値: L の下三角を同じ形式で格納した配列

            double[] L = new double[A.Length];

            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j <= i; j++)
                {
                    double sum = 0.0;
                    for (int k = 0; k < j; k++)
                    {
                        int ik = i * (i + 1) / 2 + k;
                        int jk = j * (j + 1) / 2 + k;
                        sum += L[ik] * L[jk];
                    }

                    int ij = i * (i + 1) / 2 + j;

                    if (i == j)
                    {
                        // 対角成分
                        L[ij] = Math.Sqrt(A[ij] - sum);
                    }
                    else
                    {
                        int jj = j * (j + 1) / 2 + j; // j行j列のインデックス
                        L[ij] = (A[ij] - sum) / L[jj];
                    }
                }
            }

            return L;
        }

        //対称行列Bの下三角行列を上の行から順に並べたAとしBx = bとなるxを返す
        public static double[] SolveWithMathNet(double[] A, double[] b)
        {
            int n = b.Length;
            // 下三角のA[]を通常の行列に復元
            var full = DenseMatrix.Create(n, n, 0.0);
            int idx = 0;
            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j <= i; j++)
                {
                    full[i, j] = A[idx];
                    full[j, i] = A[idx]; // 対称成分
                    idx++;
                }
            }

            var bVec = DenseVector.OfArray(b);

            // Cholesky（正定値対称ならこれが一番速い）
            var chol = full.Cholesky();
            var x = chol.Solve(bVec);

            return x.ToArray();
        }

        public static double[] SolveWithMathNet(Cholesky<double> chol, double[] b)
        {
            var bvec = Vector<double>.Build.DenseOfArray(b);

            Vector<double> x = chol.Solve(bvec);    // 高速ソルバで解く
            return x.ToArray();
        }

        public static double[] SolveWithMathNet(double[,] arrA, double[] arrb)
        {
            /*
            Matrix<double> A = DenseMatrix.OfArray(arrA);
            Vector<double> b = Vector.Build.DenseOfArray(arrb);
            if (A.RowCount != A.ColumnCount) throw new ArgumentException("A must be square.");
            if (A.RowCount != b.Count) throw new ArgumentException("A and b size mismatch.");

            // 1) まず Cholesky（最速・安定：SPD の場合のみ成功）
            try
            {
                var chol = A.Cholesky();              // SPD でないと例外
                return chol.Solve(b).ToArray();
            }
            catch
            {
                // 続行（想定どおりのフォールバック）
            }

            // 2) LU 分解（部分ピボットあり）— 対称でなくてもOK、非SPDでもOK
            //    ここで対称性を活かした LDL^T ではないが、安定で速い実装。
            var lu = A.LU();
            return lu.Solve(b).ToArray();
            */


            if (arrA == null || arrb == null)
                throw new ArgumentNullException("Input arrays must not be null.");

            Matrix<double> A = DenseMatrix.OfArray(arrA);
            Vector<double> b = Vector.Build.DenseOfArray(arrb);

            if (A.RowCount != A.ColumnCount)
                throw new ArgumentException("Matrix A must be square.");
            if (A.RowCount != b.Count)
                throw new ArgumentException("Matrix A and vector b size mismatch.");

            
            // 1. Cholesky (SPD only)
            try
            {
                var chol = A.Cholesky();
                return chol.Solve(b).ToArray();
            }
            catch (Exception ex) when (ex is ArgumentException || ex is InvalidOperationException)
            {
                // Continue to next method
            }
            

            // 2. LU (pivoted, general square matrix)
            try
            {
                var lu = A.LU();
                return lu.Solve(b).ToArray();
            }
            catch (Exception ex)
            {
                // Continue to next method
            }
            

            // 3. QR (stable for full-rank matrices)
            try
            {
                var qr = A.QR();
                return qr.Solve(b).ToArray();
            }
            catch (Exception ex)
            {
                // Continue to next method
            }
            

            // 4. SVD (most stable, handles rank-deficient cases)
            try
            {
                var svd = A.Svd(true);
                return svd.Solve(b).ToArray();
            }
            catch (Exception ex)
            {
                throw new InvalidOperationException("All solvers failed. Matrix may be singular or ill-conditioned.", ex);
            }
        }

        public static double[] SolveRobust(double[,] arrA, double[] arrb)
        {
            if (arrA == null || arrb == null)
                throw new ArgumentNullException("Input arrays must not be null.");

            Matrix<double> A = DenseMatrix.OfArray(arrA);
            Vector<double> b = Vector.Build.DenseOfArray(arrb);

            if (A.RowCount != A.ColumnCount)
                throw new ArgumentException("Matrix A must be square.");
            if (A.RowCount != b.Count)
                throw new ArgumentException("Matrix A and vector b size mismatch.");

            // 1. Cholesky (SPD only)
            try
            {
                var chol = A.Cholesky();
                return chol.Solve(b).ToArray();
            }
            catch (Exception ex) when (ex is ArgumentException || ex is InvalidOperationException)
            {
                // Continue to next method
            }

            // 2. LU (pivoted, general square matrix)
            try
            {
                var lu = A.LU();
                return lu.Solve(b).ToArray();
            }
            catch (Exception ex)
            {
                // Continue to next method
            }

            // 3. QR (stable for full-rank matrices)
            try
            {
                var qr = A.QR();
                return qr.Solve(b).ToArray();
            }
            catch (Exception ex)
            {
                // Continue to next method
            }

            // 4. SVD (most stable, handles rank-deficient cases)
            try
            {
                var svd = A.Svd(true);
                return svd.Solve(b).ToArray();
            }
            catch (Exception ex)
            {
                throw new InvalidOperationException("All solvers failed. Matrix may be singular or ill-conditioned.", ex);
            }
        }
    }
}


