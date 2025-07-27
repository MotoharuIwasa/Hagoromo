using System;

class Program
{
    static void Main(string[] args)
    {
        // 例として 2x3 の行列を定義
        double[,] matrix = {
            { 2, 0, 0, 0, 3, 0,-2, 0, 0, 0, 3, 0 },
            { 0, 5, 0,-7, 0, 0, 0,-5, 0,-7, 0, 0 },
            { 0, 0,11, 0, 0, 0, 0, 0,-11, 0, 0, 0 },
            { 0,-7, 0,13, 0, 0, 0,7, 0,17, 0, 0 },
            { 3, 0, 0, 0,19, 0,-3, 0, 0, 0,29, 0 },
            { 0, 0, 0, 0, 0,23, 0, 0, 0, 0, 0,-23 },
            {-2, 0, 0, 0,-3, 0, 2, 0, 0, 0,-3, 0 },
            { 0,-5, 0, 7, 0, 0, 0, 5, 0, 7, 0, 0 },
            { 0, 0,-11, 0, 0, 0, 0, 0,11, 0, 0, 0 },
            { 0,-7, 0,17, 0, 0, 0, 7, 0, 13, 0, 0 },
            { 3, 0, 0, 0,29, 0,-3, 0, 0, 0,19, 0 },
            { 0, 0, 0, 0, 0,-23, 0, 0, 0, 0, 0,23 },

        };

        /*
        int rows = 2048;
        int cols = 12;
        int[,] flags = new int[rows, cols];

        for (int ii = 0; ii < rows; ii++)
        {
            for (int jj = 0; jj < cols; jj++) // 11ビット分
            {
                // i の j ビット目を取り出して格納（ビットシフト）
                flags[ii, jj] = (ii >> (cols - 1 - jj)) & 1;
            }
        }*/

        int rows = 1;
        int cols = 12;
        int[,] flags = { { 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1 } };

        double[,] m = (double[,])matrix.Clone();
        double[,] s = new double[,] { };
        for (int ii = 0; ii < rows; ii++)
        {
            
            for (int jj = 0; jj < cols; jj++) // 11ビット分
            {
                if (flags[ii, jj] == 0)
                {
                    s = DoubleMatrix(m, jj);
                    m = (double[,])s.Clone();
                }
            }

            PrintMatrix(s);
            /*
            double[] d = { m[0, 0], -m[0, 6], m[6, 6] };
            double[] h = { m[1, 1], -m[1, 7], m[7, 7] };
            double[] i = { m[2, 2], -m[2, 8], m[8, 8] };
            double[] f = { m[3, 3], m[9, 9] };
            double[] b = { m[4, 4], m[10, 10] };
            double[] j = { m[5, 5], -m[5, 11], m[11, 11] };
            double[] c = { m[0, 4], m[0, 10], -m[6, 4], -m[6, 10] };
            double[] g = { -m[1, 3], -m[1, 9], m[3, 7], m[7, 9] };
            double e = m[3, 9];
            double a = m[4, 10];
            double[] flag = { flags[ii, 0], flags[ii, 1], flags[ii, 2], flags[ii, 3], flags[ii, 4], flags[ii, 5], flags[ii, 6], flags[ii, 7], flags[ii, 8], flags[ii, 9], flags[ii, 10], flags[ii, 11] };

            Console.WriteLine($"==== ii = {ii} ====");

            PrintArray("d", d);
            PrintArray("h", h);
            PrintArray("i", i);
            PrintArray("f", f);
            PrintArray("b", b);
            PrintArray("j", j);
            PrintArray("c", c);
            PrintArray("g", g);
            PrintArray("flag", flag);

            Console.WriteLine();*/
        }
    }

    // 行列を2倍する関数
    static double[,] DoubleMatrix(double[,] mat,int m)
    {
        int rows = mat.GetLength(0);
        int cols = mat.GetLength(1);
        double[,] result = new double[rows, cols];
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                result[i, j] = mat[i, j] - (mat[i, m] * mat[m, j] / mat[m, m]);
            }
        }

        return result;
    }

    static void PrintArray(string name, double[] array)
    {
        Console.Write($"{name}: ");
        foreach (var val in array)
        {
            Console.Write($"{val,8:F2} ");
        }
        Console.WriteLine();
    }

    /*
    static int[,] abxMatrix(double[,] mat)
    {
        int rows = mat.GetLength(0);
        int cols = mat.GetLength(1);
        int[,] result2 = new int[rows, cols];

        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                if (mat[i, j] != 0)
                {
                    result2[i, j] = 1;
                }
                else
                {
                    result2[i, j] = 0;
                }
            }
        }

        return result2;
    }

    static int[,] abMatrix(int[,] mat1, int[,] mat2)
    {
        int rows = mat1.GetLength(0);
        int cols = mat1.GetLength(1);
        int[,] result = new int[rows, cols];

        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                if (mat2[i, j] == 0 || mat1[i, j] == 1)
                {
                    result[i, j] = 0;
                }
                else
                {
                    result[i, j] = 1;
                }
                    
            }
        }

        return result;
    }*/

    // 行列をコンソールに出力
    static void PrintMatrix(double[,] mat)
    {
        int rows = mat.GetLength(0);
        int cols = mat.GetLength(1);

        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                Console.Write($"{mat[i, j],6:F2}");

            }
            Console.WriteLine();
        }
    }
}