using System;
using System.Diagnostics;

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
            U[t + j - 1] = A[j*(j-1)/2]; //u_1j(=a_1j) =a_j1//
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
                    s += L[i * (i - 3) / 2 + k] * U[(n - k) * (n - k + 1)/2 + j - k];
                }
                id = (n - i) * (n - i + 1) / 2 + j - i;
                U[id] = A[j * (j - 1)/2 + i - 1] - s;//u_ijを求める//

                if (i != j)
                {
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
        b[n - 1] = b[n - 1] / U[0];
        for (int i = 1; i <= n - 1; i++)
        {
            s = 0;
            for (int j = n - i + 1; j <= n; j++)
            {
                s += U[i * (i + 3) / 2 + j - n] * b[j - 1];
            }
            b[n - i - 1] = (b[n - i - 1] - s) / U[i * (i + 1) / 2];
        }
        return b;
    }

    static void Main(string[] args)
    {
        // 対称3×3行列の下三角要素：A = [4 1 2; 1 3 5; 2 5 6]
        double[] A = new double[] { 3,2,4,5,7,5,4,3,5,6 };
        double[] b = new double[] { 0, 0,0,0};

        double[] x = SLEsSolvers.SolveByLDL(A, b);

        Console.WriteLine("求まった解ベクトル x:");
        for (int i = 0; i < x.Length; i++)
        {
            Console.WriteLine($"x[{i}] = {x[i]}");
        }

        Console.WriteLine("Enterキーで終了...");
        Console.ReadLine();
    }
}