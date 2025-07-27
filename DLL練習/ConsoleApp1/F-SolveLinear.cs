using System;
using Hagoromo.MathTools;
using Hagoromo.DataStructure;
using Hagoromo.StiffnessTools;
using System.Collections.Generic;
using Grasshopper.Kernel.Parameters;
using Grasshopper.Kernel;
using System.Diagnostics;
using Rhino.Commands;


namespace Hagoromo.Solver
{
    public static class SolveLinear
    {
        public static double[] SolveDisp(Alldata alldata)
        {
            fAlldata falldata = new fAlldata(alldata);
            double[,] k_ff = LinearK.MakeModifiedGlobalK(falldata).Item1;
            double[] p_f = GetFalsePart(falldata.fNodeVConArray, falldata.fNodeIConArray);
            double[] p_f1 = GetFalsePart(falldata.fNodeVConArray, falldata.fNodeIConArray);
            double[] k_ffd = MatrixUtils.ExtractLowerTriangle(k_ff);
            double[] U_f = SLEsSolvers.SolveByLDL(k_ffd, p_f);/*error:ここでU_f、p_fがnullになる*/
            double[] U = MakeU(falldata.fNodeVConArray, U_f, falldata.fNodeIConArray);
            return U;
        }

        public static T[] GetTruePart<T>(T[] a, bool[] flags)
        {
            int count = flags.Length;
            List<T> b = new List<T>();
            for (int i = 0; i < count; i++)
            {
                if (flags[i])
                {
                    b.Add(a[i]);
                }
                else
                {
                    continue;
                }
            }
            T[] c = b.ToArray();
            return c;
        }

        public static T[] GetFalsePart<T>(T[] a, bool[] flags)
        {
            int count = flags.Length;
            List<T> b = new List<T>();
            for (int i = 0; i < count; i++)
            {
                if (!(flags[i]))
                {
                    b.Add(a[i]);
                }
                else
                {
                    continue;
                }
            }
            T[] c = b.ToArray();
            return c;
        }

        public static double[] MakeU(double[] vcondata, double[] U_f, bool[] icondata)
        {
            int count = icondata.Length;
            double[] U = (double[])vcondata.Clone();
            int j = 0;
            for (int i = 0; i < count; i++)
            {
                if (!icondata[i])
                {
                    U[i] = U_f[j];
                    j++;
                }
                else
                {
                    continue;
                }
            }
            return U;
        }
        
    }
}

