using System;

namespace Hagoromo.MathTools
{
    //行列に対して行う操作をまとめたクラス
    public static class MatrixUtils
    {
        // 行列の加算
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

        // 行列の乗算
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
    }
}
