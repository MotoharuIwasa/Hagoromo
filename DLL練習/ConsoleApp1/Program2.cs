using System;

class Program
{
    public static void Main()
    {
        // 例として 4x4 の行列を定義
        double[,] matrix = new double[,]
        {
            { 1, 2, 3, 4 },
            { 2, 5, 6, 7 },
            { 3, 6, 8, 9 },
            { 4, 7, 9, 10 }
        };

        // フラグ: false のインデックスは 0 と 2、true は 1 と 3
        bool[] flags = new bool[] { false, true, false, true };

        // 関数呼び出し
        var result = MakeModifiedMatrix(matrix, flags);

        // 結果の出力
        string[] labels = { "matrix_ff", "matrix_fs", "matrix_sf", "matrix_ss" };
        for (int k = 0; k < result.Length; k++)
        {
            Console.WriteLine($"--- {labels[k]} ---");
            PrintMatrix(result[k]);
            Console.WriteLine();
        }
    }

    public static double[][,] MakeModifiedMatrix(double[,] matrix, bool[] flags)
    {
        int false_count = 0;
        int flags_size = flags.Length;
        for (int i = 0; i < flags_size; i++)
            if (!flags[i]) false_count++;

        int true_count = flags_size - false_count;
        int[] false_id = new int[false_count];
        int[] true_id = new int[true_count];
        int false_id_count = 0;
        int true_id_count = 0;

        for (int i = 0; i < flags_size; i++)
        {
            if (!flags[i]) false_id[false_id_count++] = i;
            else true_id[true_id_count++] = i;
        }

        double[,] matrix_ff = new double[false_count, false_count];
        double[,] matrix_fs = new double[false_count, true_count];
        double[,] matrix_sf = new double[true_count, false_count];
        double[,] matrix_ss = new double[true_count, true_count];

        for (int i = 0; i < false_count; i++)
            for (int j = 0; j < false_count; j++)
                matrix_ff[i, j] = matrix[false_id[i], false_id[j]];

        for (int i = 0; i < true_count; i++)
            for (int j = 0; j < true_count; j++)
                matrix_ss[i, j] = matrix[true_id[i], true_id[j]];

        for (int i = 0; i < false_count; i++)
            for (int j = 0; j < true_count; j++)
                matrix_fs[i, j] = matrix[false_id[i], true_id[j]];

        for (int i = 0; i < true_count; i++)
            for (int j = 0; j < false_count; j++)
                matrix_sf[i, j] = matrix[true_id[i], false_id[j]];

        return new double[][,] { matrix_ff, matrix_fs, matrix_sf, matrix_ss };
    }

    public static void PrintMatrix(double[,] mat)
    {
        int rows = mat.GetLength(0);
        int cols = mat.GetLength(1);
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                Console.Write($"{mat[i, j],6}");
            }
            Console.WriteLine();
        }
    }
}