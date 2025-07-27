using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Parameters;
using Hagoromo.DataStructure;
using Hagoromo.MathTools;
using Hagoromo.StiffnessTools;
using Rhino.Commands;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;


namespace Hagoromo.Solver
{
    public static class SolveLinear
    {
        public static double[] SolveDisp(fAlldata falldata)
        {
            double[] gravity = falldata.fGravityArray;
            var globalK = LinearK.MakeModifiedGlobalK(falldata);
            double[,] k_ff = globalK.Item1;
            double[,] k_fs = globalK.Item2;

            //重力は各部材の両端にその重力方向の荷重を分配する。曲げの分配は考慮しない。
            double[] vcon = (double[])falldata.fNodeVConArray.Clone();
            vcon = MatrixUtils.Add(vcon, GravityDistribute(falldata));
            vcon = MatrixUtils.Add(vcon, CMQDistribute(falldata));
            double[] p_f = GetFalsePart(vcon, falldata.fNodeIConArray);

            //double[] p_f = GetFalsePart(falldata.fNodeVConArray, falldata.fNodeIConArray);
            //強制変位の考慮、もしiconが1でそこのvconが0でなければ強制変位とみなす。
            double[] U_s = GetTruePart(falldata.fNodeVConArray, falldata.fNodeIConArray);
            double[] f = MatrixUtils.Subtract(p_f, MatrixUtils.Multiply(k_fs, U_s));
            double[] k_ffd = MatrixUtils.ExtractLowerTriangle(k_ff);
            double[] U_f = SLEsSolvers.SolveByLDL(k_ffd, f);
            double[] U = MakeU(falldata.fNodeVConArray, U_f, falldata.fNodeIConArray);
            return U;
        }

        public static double[,] SolveStress(fAlldata falldata, double[] U)
        {
            object[] fElemArray = falldata.fElemArray;
            int count = fElemArray.Length / 32;
            double[,] p = new double[count, 12];
            for (int i = 0; i < count; i++)
            {
                int nodeid_s = (int)fAlldata.GetElemItem(falldata.fElemArray, i, 1, 0) - 1;
                int nodeid_e = (int)fAlldata.GetElemItem(falldata.fElemArray, i, 1, 1) - 1;

                double[] partU = new double[12];
                for (int j = 0; j < 6; j++)
                {
                    partU[j] = U[nodeid_s * 6 + j];
                    partU[j + 6] = U[nodeid_e * 6 + j];
                }
                /*
                double[] partU2 = new double[12];
                for (int j = 0; j < 6; j++)
                {
                    partU2[j] = U[nodeid_s * 6 + j];
                    partU2[j + 6] = U[nodeid_e * 6 + j];
                }
                */

                double[,] ke = LinearK.MakeKe(falldata, i);

                double[,] Coord = LinearK.MakeCoord(fElemArray, i);
                double[,] T = new double[12, 12];
                MatrixUtils.AddPartMatrix(Coord, T, 0, 0);
                MatrixUtils.AddPartMatrix(Coord, T, 3, 3);
                MatrixUtils.AddPartMatrix(Coord, T, 6, 6);
                MatrixUtils.AddPartMatrix(Coord, T, 9, 9);

                /*
                double[] partU = (double[])partU2.Clone();
                partU[0] = partU2[1];
                partU[1] = partU2[0];
                partU[3] = partU2[4];
                partU[4] = partU2[3];
                partU[6] = partU2[7];
                partU[7] = partU2[6];
                partU[9] = partU2[10];
                partU[10] = partU2[9];
                */

                double[] part_vcon = ModifyCMQ(falldata, i);
                double[] partp = MatrixUtils.Multiply(ke, MatrixUtils.Multiply(T, partU));
                
                partp = MatrixUtils.Subtract(partp, part_vcon);
                
                for (int k = 0; k < 12; k++)
                {//出力の順番が正しくなるように調整
                    if (k == 0 || k == 3 || k == 6 || k == 9)
                    {
                        p[i, k] = partp[k + 1];
                        //p[i,k] = partp[k + 1]-part_vcon[k];
                    }
                    else if (k == 1 || k == 4 || k == 7 || k == 10)
                    {
                        p[i, k] = partp[k - 1];
                        //p[i,k] = partp[k - 1]-part_vcon[k];
                    }
                    else
                    {
                        p[i, k] = partp[k];
                        //p[i,k] = partp[k]-part_vcon[k];
                    }
                }
            }
            return p;
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

        public static double[] GravityDistribute(fAlldata falldata)
        {
            int length = falldata.fNodeVConArray.Length;
            double[] vcon = new double[length];
            double[] gravity = falldata.fGravityArray;
            int elem_count = falldata.fElemArray.Length / 32;
            for (int i = 0; i < elem_count; i++)
            {
                int sectid = (int)falldata.fElemArray[32 * i] - 1;
                int propid = (int)falldata.fSectArray[sectid, 0] - 1;
                int startid = (int)falldata.fElemArray[32 * i + 1] - 1;
                int endid = (int)falldata.fElemArray[32 * i + 2] - 1;
                //weight = l*A*ρ/2　2で割るのは両端に半分ずつ分配するから
                double weight = (double)falldata.fElemArray[32 * i + 16] * (double)falldata.fSectArray[sectid, 1] * falldata.fPropArray[propid, 0] / 2;
                vcon[startid * 6] += weight * gravity[0];
                vcon[startid * 6 + 1] += weight * gravity[1];
                vcon[startid * 6 + 2] += weight * gravity[2];
                vcon[endid * 6] += weight * gravity[0];
                vcon[endid * 6 + 1] += weight * gravity[1];
                vcon[endid * 6 + 2] += weight * gravity[2];
            }
            return vcon;
        }

        //この関数の出力は節点が部材端から受ける力
        public static double[] ModifyCMQ(fAlldata falldata, int i)
        {
            double[] part_vcon = new double[12];
            bool[] constraints = ((object[])fAlldata.GetElemItem(falldata.fElemArray, i, 2)).Cast<bool>().ToArray();
            double[] cmq_input = ((object[])fAlldata.GetElemItem(falldata.fElemArray, i, 4)).Cast<double>().ToArray();
            double l = (double)fAlldata.GetElemItem(falldata.fElemArray, i, 3, 1);

            //座標系の設定の都合上入れ替えなければいけない
            double[] cmq = (double[])cmq_input.Clone();
            cmq[0] = cmq_input[1];
            cmq[1] = cmq_input[0];
            cmq[3] = cmq_input[4];
            cmq[4] = cmq_input[3];
            cmq[6] = cmq_input[7];
            cmq[7] = cmq_input[6];
            cmq[9] = cmq_input[10];
            cmq[10] = cmq_input[9];

            //軸力、財軸方向の力がかかっている支持点でその力を負担。その支持点がローラーならもう片方の支持点で負担
            if (cmq[2] != 0)
            {
                if (constraints[2]) { part_vcon[2] += cmq[2]; }
                else { part_vcon[8] += cmq[2]; }
            }

            if (cmq[8] != 0)
            {
                if (constraints[8]) { part_vcon[8] += cmq[8]; }
                else { part_vcon[2] += cmq[8]; }
            }

            //せんだん力(x)、力がかかっている支持点でその力を負担。その支持点がローラーなら別途考察
            if (cmq[0] != 0)
            {
                if (constraints[0]) { part_vcon[0] += cmq[0]; }
                else
                {
                    part_vcon[6] += cmq[0];
                    if (constraints[4])
                    {
                        if (constraints[10])
                        {
                            part_vcon[4] += cmq[0] * l / 2;
                            part_vcon[10] += cmq[0] * l / 2;
                        }
                        else { part_vcon[4] += cmq[0] * l; }
                    }
                    else { part_vcon[10] += cmq[0] * l; }
                }
            }

            if (cmq[6] != 0)
            {
                if (constraints[6]) { part_vcon[6] += cmq[6]; }
                else
                {
                    part_vcon[0] += cmq[6];
                    if (constraints[10])
                    {
                        if (constraints[4])
                        {
                            part_vcon[10] -= cmq[6] * l / 2;
                            part_vcon[4] -= cmq[6] * l / 2;
                        }
                        else { part_vcon[10] -= cmq[6] * l; }
                    }
                    else { part_vcon[4] -= cmq[6] * l; }
                }
            }

            //せんだん力(y)、力がかかっている支持点でその力を負担。その支持点がローラーなら別途考察
            if (cmq[1] != 0)
            {
                if (constraints[1]) { part_vcon[1] += cmq[1]; }
                else
                {
                    part_vcon[7] += cmq[1];
                    if (constraints[3])
                    {
                        if (constraints[9])
                        {
                            part_vcon[3] += cmq[1] * l / 2;
                            part_vcon[9] += cmq[1] * l / 2;
                        }
                        else { part_vcon[3] += cmq[1] * l; }
                    }
                    else { part_vcon[9] += cmq[1] * l; }
                }
            }

            if (cmq[7] != 0)
            {
                if (constraints[7]) { part_vcon[7] += cmq[7]; }
                else
                {
                    part_vcon[1] += cmq[7];
                    if (constraints[9])
                    {
                        if (constraints[3])
                        {
                            part_vcon[9] -= cmq[7] * l / 2;
                            part_vcon[3] -= cmq[7] * l / 2;
                        }
                        else { part_vcon[9] -= cmq[7] * l; }
                    }
                    else { part_vcon[3] -= cmq[7] * l; }
                }
            }

            //曲げモーメント(x)
            if (cmq[3] != 0)
            {
                if (constraints[3]) { part_vcon[3] -= cmq[3]; }
                else
                {
                    if ((!constraints[1]) || (!constraints[7])) { part_vcon[9] += cmq[3]; }
                    else
                    {
                        if (constraints[9])
                        {
                            part_vcon[1] += 3 * cmq[3] / (2 * l);
                            part_vcon[7] -= 3 * cmq[3] / (2 * l);
                            part_vcon[9] -= cmq[3] / 2;
                        }
                        else
                        {
                            part_vcon[1] += cmq[3] / l;
                            part_vcon[7] -= cmq[3] / l;
                        }
                    }
                }
            }

            if (cmq[9] != 0)
            {
                if (constraints[9]) { part_vcon[9] -= cmq[9]; }
                else
                {
                    if ((!constraints[7]) || (!constraints[1])) { part_vcon[3] += cmq[9]; }
                    else
                    {
                        if (constraints[3])
                        {
                            part_vcon[7] -= 3 * cmq[9] / (2 * l);
                            part_vcon[1] += 3 * cmq[9] / (2 * l);
                            part_vcon[3] -= cmq[9] / 2;
                        }
                        else
                        {
                            part_vcon[7] -= cmq[9] / l;
                            part_vcon[1] += cmq[9] / l;
                        }
                    }
                }
            }

            //曲げモーメント(y)
            if (cmq[4] != 0)
            {
                if (constraints[4]) { part_vcon[4] -= cmq[4]; }
                else
                {
                    if ((!constraints[0]) || (!constraints[6])) { part_vcon[10] += cmq[4]; }
                    else
                    {
                        if (constraints[10])
                        {
                            part_vcon[0] += 3 * cmq[4] / (2 * l);
                            part_vcon[6] -= 3 * cmq[4] / (2 * l);
                            part_vcon[10] -= cmq[4] / 2;
                        }
                        else
                        {
                            part_vcon[0] += cmq[4] / l;
                            part_vcon[6] -= cmq[4] / l;
                        }
                    }
                }
            }

            if (cmq[10] != 0)
            {
                if (constraints[10]) { part_vcon[10] -= cmq[10]; }
                else
                {
                    if ((!constraints[6]) || (!constraints[0])) { part_vcon[4] += cmq[10]; }
                    else
                    {
                        if (constraints[4])
                        {
                            part_vcon[6] -= 3 * cmq[10] / (2 * l);
                            part_vcon[0] += 3 * cmq[10] / (2 * l);
                            part_vcon[4] -= cmq[10] / 2;
                        }
                        else
                        {
                            part_vcon[6] -= cmq[10] / l;
                            part_vcon[0] += cmq[10] / l;
                        }
                    }
                }
            }

            //ねじれモーメント
            if (cmq[5] != 0)
            {
                if (constraints[5]) { part_vcon[5] += cmq[5]; }
                else { part_vcon[11] += cmq[5]; }
            }

            if (cmq[11] != 0)
            {
                if (constraints[11]) { part_vcon[11] += cmq[11]; }
                else { part_vcon[5] += cmq[11]; }
            }

            return part_vcon;
        }

        public static double[] CMQDistribute(fAlldata falldata)
        {
            int length = falldata.fNodeVConArray.Length;
            double[] vcon = new double[length];
            int elem_count = falldata.fElemArray.Length / 32;
            for (int i = 0; i < elem_count; i++)
            {
                double[] part_vcon = ModifyCMQ(falldata, i);
                bool[] constraints_input = ((object[])fAlldata.GetElemItem(falldata.fElemArray, i, 2)).Cast<bool>().ToArray();

                //座標系の設定の都合上入れ替えなければいけない
                bool[] constraints = (bool[])constraints_input.Clone();
                constraints[0] = constraints_input[1];
                constraints[1] = constraints_input[0];
                constraints[3] = constraints_input[4];
                constraints[4] = constraints_input[3];
                constraints[6] = constraints_input[7];
                constraints[7] = constraints_input[6];
                constraints[9] = constraints_input[10];
                constraints[10] = constraints_input[9];

                //材端がフリーな部分は節点に部材応力分の力が流れないので0にする
                for (int j = 0; j < 12; j++)
                {
                    if (!constraints[j])
                    {
                        part_vcon[j] = 0;
                    }
                }

                double[,] Coord = LinearK.MakeCoord(falldata.fElemArray, i);
                double[,] TransCoord = MatrixUtils.Transpose(Coord);
                double[,] TransT = new double[12, 12];
                MatrixUtils.AddPartMatrix(TransCoord, TransT, 0, 0);
                MatrixUtils.AddPartMatrix(TransCoord, TransT, 3, 3);
                MatrixUtils.AddPartMatrix(TransCoord, TransT, 6, 6);
                MatrixUtils.AddPartMatrix(TransCoord, TransT, 9, 9);
                part_vcon = MatrixUtils.Multiply(TransT, part_vcon);

                int startid = (int)falldata.fElemArray[32 * i + 1] - 1;
                int endid = (int)falldata.fElemArray[32 * i + 2] - 1;
                for (int j =0; j < 6; j++)
                {
                    vcon[startid * 6 + j] = part_vcon[j];
                    vcon[endid*6+j] = part_vcon[j+6];
                }
            }
            return vcon;
        }
    }
}

