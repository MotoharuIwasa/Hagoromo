using System;
using System.Collections.Generic;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Collections;
using Rhino.Render.ChangeQueue;


namespace Hagoromo.MathTools
{
    //行列に対して行う操作をまとめたクラス
    public static class MatrixUtils
    {
        // 行列の加算a+b
        public static double[,] Add(double[,] a, double[,] b)
        {
            int rows = a.GetLength(0);
            int cols = a.GetLength(1);
            var result = new double[rows, cols];

            for (int i = 0; i < rows; i++)
                for (int j = 0; j < cols; j++)
                    result[i, j] = a[i, j] + b[i, j];

            return result;
        }

        // ベクトルの加算a＋b
        public static double[] Add(double[] a, double[] b)
        {
            int cols = a.Length;
            var result = new double[cols];
            for (int i = 0; i < cols; i++)
            {
                result[i] = a[i] + b[i];
            }
            return result;
        }

        // 行列の減算a-b
        public static double[,] Subtract(double[,] a, double[,] b)
        {
            int rows = a.GetLength(0);
            int cols = a.GetLength(1);
            var result = new double[rows, cols];

            for (int i = 0; i < rows; i++)
                for (int j = 0; j < cols; j++)
                    result[i, j] = a[i, j] - b[i, j];

            return result;
        }

        // ベクトルの減算a-b
        public static double[] Subtract(double[] a, double[] b)
        {
            int cols = a.Length;
            var result = new double[cols];
            for (int i = 0; i < cols; i++)
            {
                result[i] = a[i] - b[i];
            }
            return result;
        }

        // 行列の乗算a*b
        public static double[,] Multiply(double[,] a, double[,] b)
        {
            int aRows = a.GetLength(0);
            int aCols = a.GetLength(1);
            int bCols = b.GetLength(1);
            var result = new double[aRows, bCols];

            for (int i = 0; i < aRows; i++)
                for (int j = 0; j < bCols; j++)
                    for (int k = 0; k < aCols; k++)
                        result[i, j] += a[i, k] * b[k, j];

            return result;
        }

        //ベクトル同士の積、つまり内積
        public static double Multiply(double[] a, double[] b)
        {
            double sum = 0;
            for (int i = 0; i < a.Length; i++) { sum += a[i] * b[i]; }
            return sum;
        }

        // 行列 × ベクトル
        public static double[] Multiply(double[,] a, double[] v)
        {
            int aRows = a.GetLength(0);
            int aCols = a.GetLength(1);

            if (v.Length != aCols)
                throw new ArgumentException("行列の列数とベクトルの次元が一致していません。");

            var result = new double[aRows];

            for (int i = 0; i < aRows; i++)
                for (int j = 0; j < aCols; j++)
                    result[i] += a[i, j] * v[j];

            return result;
        }

        //ベクトルのスカラー倍
        public static double[] Multiply(double scalar, double[] v)
        {
            double[] result = new double[v.Length];
            for (int i = 0; i < v.Length; i++)
            {
                result[i] = v[i] * scalar;
            }
            return result;
        }

        //行列の各要素をスカラー倍
        public static double[,] Multiply(double scalar, double[,] A)
        {
            double[,] result = new double[A.GetLength(0),A.GetLength(1)];
            for (int i = 0; i < A.GetLength(0); i++)
            {
                for (int j = 0;j < A.GetLength(1); j++)
                result[i,j] = A[i,j] * scalar;
            }
            return result;
        }

        public static Vector3d Multiply(double[,] A, Vector3d v)
        {
            Vector3d result = new Vector3d();
            result.X = A[0, 0] * v.X + A[0, 1] * v.Y + A[0, 2] * v.Z;
            result.Y = A[1, 0] * v.X + A[1, 1] * v.Y + A[1, 2] * v.Z;
            result.Z = A[2, 0] * v.X + A[2, 1] * v.Y + A[2, 2] * v.Z;
            return result;
        }

        // 転置
        public static double[,] Transpose(double[,] matrix)
        {
            int rows = matrix.GetLength(0);
            int cols = matrix.GetLength(1);
            var result = new double[cols, rows];

            for (int i = 0; i < rows; i++)
                for (int j = 0; j < cols; j++)
                    result[j, i] = matrix[i, j];

            return result;
        }

        //逆行列
        public static double[,] InverseMatrix(double[,] matrix)
        {
            int n = matrix.GetLength(0);
            if (n != matrix.GetLength(1))
                throw new ArgumentException("Matrix must be square.");

            // 拡大行列 [A | I]
            double[,] augmented = new double[n, 2 * n];
            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < n; j++)
                {
                    augmented[i, j] = matrix[i, j];
                }
                augmented[i, i + n] = 1.0; // 単位行列
            }

            // 掃き出し法
            for (int i = 0; i < n; i++)
            {
                double pivot = augmented[i, i];
                if (Math.Abs(pivot) < 1e-12)
                    throw new InvalidOperationException("Matrix is singular or nearly singular.");

                // ピボット行を1に正規化
                for (int j = 0; j < 2 * n; j++)
                    augmented[i, j] /= pivot;

                // 他の行をピボット列で消去
                for (int k = 0; k < n; k++)
                {
                    if (k == i) continue;
                    double factor = augmented[k, i];
                    for (int j = 0; j < 2 * n; j++)
                        augmented[k, j] -= factor * augmented[i, j];
                }
            }

            // 逆行列を抽出
            double[,] inverse = new double[n, n];
            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < n; j++)
                {
                    inverse[i, j] = augmented[i, j + n];
                }
            }

            return inverse;
        }

        // 行列sourceを別の行列destinationのstartRow,startColスタートの場所にペーストする関数
        public static void CopyMatrix(double[,] source, double[,] destination, int startRow, int startCol)
        {
            int rows = source.GetLength(0); // ソース行列の行数
            int cols = source.GetLength(1); // ソース行列の列数

            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < cols; j++)
                {
                    // 指定された開始位置に対して、ソース行列の要素をコピー
                    destination[startRow + i, startCol + j] = source[i, j];
                }
            }
        }

        // 行列sourceの-1倍を別の行列destinationのstartRow,startColスタートの場所にペーストする関数
        public static void CopyNegMatrix(double[,] source, double[,] destination, int startRow, int startCol)
        {
            int rows = source.GetLength(0); // ソース行列の行数
            int cols = source.GetLength(1); // ソース行列の列数

            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < cols; j++)
                {
                    // 指定された開始位置に対して、ソース行列の要素をコピー
                    destination[startRow + i, startCol + j] = -source[i, j];
                }
            }
        }

        // 行列を別の行列の部分に足し合わせる関数
        public static void AddPartMatrix(double[,] source, double[,] destination, int startRow, int startCol)
        {
            int rows = source.GetLength(0); // ソース行列の行数
            int cols = source.GetLength(1); // ソース行列の列数

            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < cols; j++)
                {
                    // 指定された開始位置に対して、ソース行列の要素をコピー
                    destination[startRow + i, startCol + j] += source[i, j];
                }
            }
        }

        public static double[,] ExtractPartMatrix(double[,] matrix, int startRow, int startCol, int rowCount, int colCount)
        {
            double[,] subMatrix = new double[rowCount, colCount];

            for (int i = 0; i < rowCount; i++)
            {
                for (int j = 0; j < colCount; j++)
                {
                    subMatrix[i, j] = matrix[startRow + i, startCol + j];
                }
            }

            return subMatrix;
        }

        /*p=Kuのp,uの並び替えに伴ってKを並び変えて4つのブロックで出力する関数、matrixの行、列に対してflagsを対応させ、
         * false,false/false,true/true,false/true,trueとなる部分をそれぞれ抽出*/
        public static double[][,] MakeModifiedMatrix(double[,] matrix, bool[] flags)
        {
            int false_count = 0;
            int flags_size = flags.Length;
            for (int i = 0; i < flags_size; i++)
            {
                if (!flags[i])
                {
                    false_count += 1;
                }
            }
            int true_count = flags_size - false_count;
            int[] false_id = new int[false_count];
            int[] true_id = new int[true_count];
            int false_id_count = 0;
            int true_id_count = 0;
            for (int i = 0; i < flags_size; i++)
            {
                if (!flags[i])
                {
                    false_id[false_id_count] = i;
                    false_id_count += 1;
                }
                if (flags[i])
                {
                    true_id[true_id_count] = i;
                    true_id_count += 1;
                }
            }

            double[,] matrix_ff = new double[false_count, false_count];
            double[,] matrix_fs = new double[false_count, true_count];
            double[,] matrix_sf = new double[true_count, false_count];
            double[,] matrix_ss = new double[true_count, true_count];

            for (int i = 0; i < false_count; i++)
            {
                for (int j = 0; j < false_count; j++)
                {
                    matrix_ff[i, j] = matrix[false_id[i], false_id[j]];
                }
            }

            for (int i = 0; i < true_count; i++)
            {
                for (int j = 0; j < true_count; j++)
                {
                    matrix_ss[i, j] = matrix[true_id[i], true_id[j]];
                }
            }

            for (int i = 0; i < false_count; i++)
            {
                for (int j = 0; j < true_count; j++)
                {
                    matrix_fs[i, j] = matrix[false_id[i], true_id[j]];
                }
            }

            for (int i = 0; i < true_count; i++)
            {
                for (int j = 0; j < false_count; j++)
                {
                    matrix_sf[i, j] = matrix[true_id[i], false_id[j]];
                }
            }

            double[][,] matrices = new double[][,] { matrix_ff, matrix_fs, matrix_sf, matrix_ss };
            return matrices;
        }

        //Aは実対称行列としてその下三角を上の行から順に行ずつで並べた配列を返す
        public static double[] ExtractLowerTriangle(double[,] A)
        {
            int n = A.GetLength(0);

            if (n != A.GetLength(1))
                throw new ArgumentException("Matrix must be square.");

            int count = n * (n + 1) / 2;
            double[] lowerTriangle = new double[count];
            int index = 0;

            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j <= i; j++)
                {
                    lowerTriangle[index] = A[i, j];
                    index += 1;
                }
            }

            return lowerTriangle;
        }

        //ベクトルvとその転置vTの積v*vTの結果の下三角行列を上の行から順に並べた配列を返す
        public static double[] GetLowerTriangleFromVector(double[] v)
        {
            int n = v.Length;
            double[,] vvT = new double[n, n];
            double[] result = new double[n*(n+1)/2];

            // vvᵀの計算
            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < i+1; j++)
                {
                    result[i * (i + 1) / 2 + j] = v[i] * v[j];
                }
            }
            return result;
        }

    }
}
