using System;
using Hagoromo.MathTools;
using Hagoromo.DataStructure;
using System.Linq;
using System.Collections.Generic;
using System.Security.Principal;

namespace Hagoromo.SolverL
{
    public static class LinearK
    {

        //Coordされた要素剛性マトリクスを構成する部分作成のための関数、部材端が剛-剛の場合用//
        public static double[,] MakeCoordElemPartType1(double[,] Coord, double d, double h, double i)
        {
            double[,] coord_elem_part1 = new double[3, 3]
            {
                { Coord[0,0]*Coord[0,0]*d+Coord[1,0]*Coord[1,0]*h+Coord[2,0]*Coord[2,0]*i, Coord[0,0]*Coord[0,1]*d+Coord[1,0]*Coord[1,1]*h+Coord[2,0]*Coord[2,1]*i, Coord[0,0]*Coord[0,2]*d+Coord[1,0]*Coord[1,2]*h+Coord[2,0]*Coord[2,2]*i },
                { Coord[0,0]*Coord[0,1]*d+Coord[1,0]*Coord[1,1]*h+Coord[2,0]*Coord[2,1]*i, Coord[0,1]*Coord[0,1]*d+Coord[1,1]*Coord[1,1]*h+Coord[2,1]*Coord[2,1]*i, Coord[0,1]*Coord[0,2]*d+Coord[1,1]*Coord[1,2]*h+Coord[2,1]*Coord[2,2]*i },
                { Coord[0,0]*Coord[0,2]*d+Coord[1,0]*Coord[1,2]*h+Coord[2,0]*Coord[2,2]*i, Coord[0,2]*Coord[0,1]*d+Coord[1,2]*Coord[1,1]*h+Coord[2,2]*Coord[2,1]*i, Coord[0,2]*Coord[0,2]*d+Coord[1,2]*Coord[1,2]*h+Coord[2,2]*Coord[2,2]*i }
            };

            return coord_elem_part1;
        }

        public static double[,] MakeCoordElemPartType2(double[,] Coord, double a, double b)
        {
            double[,] coord_elem_part2 = new double[3, 3]
            {
                { Coord[0,0]*Coord[1,0]*a+Coord[0,0]*Coord[1,0]*b, Coord[0,1]*Coord[1,0]*a+Coord[0,0]*Coord[1,1]*b, Coord[0,2]*Coord[1,0]*a+Coord[0,0]*Coord[1,2]*b},
                { Coord[0,0]*Coord[1,1]*a+Coord[0,1]*Coord[1,0]*b, Coord[0,1]*Coord[1,1]*a+Coord[0,1]*Coord[1,1]*b, Coord[0,2]*Coord[1,1]*a+Coord[0,1]*Coord[1,2]*b},
                { Coord[0,0]*Coord[1,2]*a+Coord[0,2]*Coord[1,0]*b, Coord[0,1]*Coord[1,2]*a+Coord[0,2]*Coord[1,1]*b, Coord[0,2]*Coord[1,2]*a+Coord[0,2]*Coord[1,2]*b}
            };

            return coord_elem_part2;
        }

        //i番目のelemの座標変換行列を出力する関数、有限要素法ハンドブックのp219の3*3行列T,coord angleについてはpdf「TKTについて」を参照
        public static double[,] MakeCoord(object[] fElemArray,int i)
        {
            (double theta, double l, double dx, double dy, double dz) =
            ((double)fAlldata.GetElemItem(fElemArray, i,3,0), (double)fAlldata.GetElemItem(fElemArray, i, 3,1), (double)fAlldata.GetElemItem(fElemArray, i, 3, 2), (double)fAlldata.GetElemItem(fElemArray, i, 3,3), (double)fAlldata.GetElemItem(fElemArray, i, 3, 4));
            //座標変換行列,座標変換行列とcoord angleについてはpdf「TKTについて」を参照//
            double ctheta = Math.Cos(Math.PI * theta / 180);//thetaはdegreeで入力//
            double stheta = Math.Sin(Math.PI * theta / 180);
            double nn = dz / l;//pdfのnまたはnxに対応//
            double[,] Coord;
            
            //部材が鉛直(全体のz軸と部材の方向(部材のx軸)が平行)の時
            if (nn == -1 || nn == 1)
            {
                Coord = new double[3, 3]
                {
                    { -nn*stheta, ctheta, 0  },
                    { nn*ctheta,  stheta, 0  },
                    { 0,          0,      nn },
                };
            }

            //部材が鉛直でない時//
            else
            {
                double ll = dx / l;//pdfのlまたはlxに対応//
                double mm = dy / l;//pdfのmまたはmxに対応//
                double lmroot = Math.Sqrt(ll * ll + mm * mm);
                Coord = new double[3, 3]
                {
                    { (mm*stheta-nn*ll*ctheta)/lmroot,   -(ll*stheta+nn*mm*ctheta)/lmroot, lmroot*ctheta  },
                    { -(mm*ctheta+nn*ll*stheta)/lmroot,  (ll*ctheta-nn*mm*stheta)/lmroot,  lmroot*stheta  },
                    { ll,                                mm,                               nn             },
                };
            }

            return Coord;
        }

        //部材端が剛剛のelemid番目のelemの要素剛性マトリクスのCoord後の行列を4ブロックで作成、有限要素法ハンドブックp219を参照、TkTを手計算した結果を打ち込んでいる。
        public static double[][,] MakeTKeTforFixFix(fAlldata falldata, int elemid)
        {
            object[] fElemArray = falldata.fElemArray;
            object[,] fSectArray = falldata.fSectArray;
            double[,] fPropArray = falldata.fPropArray;
            
            //要素剛性マトリクスに出てくる値をあらかじめ計算//
            int sectid = (int)fAlldata.GetElemItem(fElemArray, elemid, 0, 0) - 1;
            int propid = (int)fSectArray[sectid, 0] - 1;

            double theta = (double)fAlldata.GetElemItem(fElemArray, elemid, 3, 0);
            double l = (double)fAlldata.GetElemItem(fElemArray, elemid, 3, 1);
            double dx = (double)fAlldata.GetElemItem(fElemArray, elemid, 3, 2);
            double dy = (double)fAlldata.GetElemItem(fElemArray, elemid, 3, 3);
            double dz = (double)fAlldata.GetElemItem(fElemArray, elemid, 3, 4);

            double E = fPropArray[propid, 1];
            double A = (double)fSectArray[sectid, 1];
            double Ix = (double)fSectArray[sectid, 2];
            double Iy = (double)fSectArray[sectid, 3];
            double K = (double)fSectArray[sectid, 4];
            double G = fPropArray[propid, 1] * 0.5 / (1 + fPropArray[propid, 2]);

            double a = 2 * E * Ix / l;/*2EIx/L*/
            double b = 2 * a;/*4EIx/L*/
            double c = 3 * a / l;/*6EIx/L^2*/
            double d = 2 * c / l;/*12EIx/L^3*/
            double e = 2 * E * Iy / l;/*2EIy/L*/
            double f = 2 * e;/*4EIy/L*/
            double g = 3 * e / l;/*6EIy/L^2*/
            double h = 2 * g / l;/*12EIy/L^3*/
            double i = E * A / l;/*EA/L*/
            double j = G * K / l;/*KG/L K:サンブナンねじり定数*/

            //座標変換行列
            double[,] Coord = MakeCoord(fElemArray, elemid);

            //T*k_e*Tの部分行列ア～オを計算//
            double[,] coord_elem_type1 = MakeCoordElemPartType1(Coord, d, h, i);
            double[,] coord_elem_type2 = MakeCoordElemPartType2(Coord, -g, c);
            double[,] coord_elem_type3 = MakeCoordElemPartType2(Coord, c, -g);
            double[,] coord_elem_type4 = MakeCoordElemPartType1(Coord, f, b, j);
            double[,] coord_elem_type5 = MakeCoordElemPartType1(Coord, e, a, -j);

            //部分行列からT*k_e*T全体を作成、ただし後に全体剛性マトリクスを作るときを考えて4ブロックに分けて出力する//
            double[,] coord_elem_k_part11 = new double[6, 6];
            double[,] coord_elem_k_part12 = new double[6, 6];
            double[,] coord_elem_k_part21 = new double[6, 6];
            double[,] coord_elem_k_part22 = new double[6, 6];

            //アの部分をKeの適切な場所に配置
            MatrixUtils.CopyMatrix(coord_elem_type1, coord_elem_k_part11, 0, 0);
            MatrixUtils.CopyMatrix(coord_elem_type1, coord_elem_k_part22, 0, 0);
            MatrixUtils.CopyNegMatrix(coord_elem_type1, coord_elem_k_part12, 0, 0);
            MatrixUtils.CopyNegMatrix(coord_elem_type1, coord_elem_k_part21, 0, 0);

            //イの部分をKeの適切な場所に配置
            MatrixUtils.CopyMatrix(coord_elem_type2, coord_elem_k_part11, 0, 3);
            MatrixUtils.CopyMatrix(coord_elem_type2, coord_elem_k_part12, 0, 3);
            MatrixUtils.CopyNegMatrix(coord_elem_type2, coord_elem_k_part21, 0, 3);
            MatrixUtils.CopyNegMatrix(coord_elem_type2, coord_elem_k_part22, 0, 3);

            //ウの部分をKeの適切な場所に配置
            MatrixUtils.CopyMatrix(coord_elem_type3, coord_elem_k_part11, 3, 0);
            MatrixUtils.CopyMatrix(coord_elem_type3, coord_elem_k_part21, 3, 0);
            MatrixUtils.CopyNegMatrix(coord_elem_type3, coord_elem_k_part12, 3, 0);
            MatrixUtils.CopyNegMatrix(coord_elem_type3, coord_elem_k_part22, 3, 0);

            //エの部分をKeの適切な場所に配置
            MatrixUtils.CopyMatrix(coord_elem_type4, coord_elem_k_part11, 3, 3);
            MatrixUtils.CopyMatrix(coord_elem_type4, coord_elem_k_part22, 3, 3);

            //オの部分をKeの適切な場所に配置
            MatrixUtils.CopyMatrix(coord_elem_type5, coord_elem_k_part12, 3, 3);
            MatrixUtils.CopyMatrix(coord_elem_type5, coord_elem_k_part21, 3, 3);

            double[][,] matrices = new double[4][,];
            matrices[0] = coord_elem_k_part11;
            matrices[1] = coord_elem_k_part12;
            matrices[2] = coord_elem_k_part21;
            matrices[3] = coord_elem_k_part22;

            return matrices;
        }

        //要素剛性マトリクスKeを出力する
        public static double[,] MakeKe(fAlldata falldata,int i)
        {
            object[] fElemArray = falldata.fElemArray;
            object[,] fSectArray = falldata.fSectArray;
            double[,] fPropArray = falldata.fPropArray;

            bool[] constraint_input = new bool[12];
            for (int jj = 0; jj < 12; jj++)
            {
                //constraintはtrueが剛(固定)としている。iconも同じでtrueが固定
                constraint_input[jj] = (bool)fAlldata.GetElemItem(fElemArray, i, 2, jj);
            }
            int sectid = (int)fAlldata.GetElemItem(fElemArray, i, 0, 0) - 1;
            int propid = (int)fSectArray[sectid, 0] - 1;

            double theta = (double)fAlldata.GetElemItem(fElemArray, i, 3, 0);
            double l = (double)fAlldata.GetElemItem(fElemArray, i, 3, 1);
            double dx = (double)fAlldata.GetElemItem(fElemArray, i, 3, 2);
            double dy = (double)fAlldata.GetElemItem(fElemArray, i, 3, 3);
            double dz = (double)fAlldata.GetElemItem(fElemArray, i, 3, 4);
            double E = fPropArray[propid, 1];
            double A = (double)fSectArray[sectid, 1];
            double Ix = (double)fSectArray[sectid, 2];
            double Iy = (double)fSectArray[sectid, 3];
            double K = (double)fSectArray[sectid, 4];
            double G = fPropArray[propid, 1] * 0.5 / (1 + fPropArray[propid, 2]);

            double m00, m06, m66, m11, m17, m77, m22, m28, m88, m33, m99, m44, m1010, m55, m511, m04, m010, m46, m610, m13, m19, m37, m79, m39, m410;
            m00 = m66 = 12 * E * Ix / (l * l * l);
            m06 = -m00;
            m11 = m77 = 12 * E * Iy / (l * l * l);
            m17 = -m11;
            m22 = m88 = E * A / l;
            m28 = -m22;
            m33 = m99 = 4 * E * Iy / l;
            m44 = m1010 = 4 * E * Ix / l;
            m55 = G * K / l;
            m511 = -m55;
            m04 = m010 = 6 * E * Ix / (l * l);
            m46 = m610 = -m04;
            m37 = m79 = 6 * E * Iy / (l * l);
            m13 = m19 = -m37;
            m39 = 2 * E * Iy / l;
            m410 = 2 * E * Ix / l;

            //要素剛性マトリクス
            
            double[,] m =  { { m00, 0, 0, 0, m04, 0, m06, 0, 0, 0, m010, 0 },
                            { 0,m11, 0,m13, 0, 0, 0,m17, 0,m19, 0, 0 },
                            { 0, 0,m22, 0, 0, 0, 0, 0,m28, 0, 0, 0 },
                            { 0,m13, 0,m33, 0, 0, 0,m37, 0,m39, 0, 0 },
                            { m04, 0, 0, 0,m44, 0,m46, 0, 0, 0,m410, 0 },
                            { 0, 0, 0, 0, 0,  m55, 0, 0, 0, 0, 0, m511 },
                            { m06, 0, 0, 0,m46, 0,m66, 0, 0, 0,m610, 0 },
                            { 0,m17, 0, m37, 0, 0, 0, m77,0, m79, 0, 0 },
                            { 0, 0,m28, 0, 0, 0, 0, 0,   m88, 0, 0, 0 },
                            { 0,m19, 0,m39, 0, 0, 0, m79, 0, m99, 0, 0 },
                            { m010, 0, 0, 0,m410,0,m610, 0, 0, 0,m1010, 0 },
                            { 0, 0, 0, 0, 0,   m511, 0, 0, 0, 0, 0, m55 }};

            double[,] mmm = (double[,])m.Clone();
            //座標変換のずれ分の並び替え以下9行 25.07.24
            bool[] constraint = (bool[])constraint_input.Clone();
            constraint[0] = constraint_input[1];
            constraint[1] = constraint_input[0];
            constraint[3] = constraint_input[4];
            constraint[4] = constraint_input[3];
            constraint[6] = constraint_input[7];
            constraint[7] = constraint_input[6];
            constraint[9] = constraint_input[10];
            constraint[10] = constraint_input[9];
            for (int pp = 0; pp < 12; pp++)
            {
                if (!constraint[pp])
                {
                    for (int ss = 0; ss < 12; ss++)
                    {
                        for (int rr = 0; rr < 12; rr++)
                        {
                            m[pp, pp] = Math.Abs(m[pp, pp]) < 1e-14 ? 0.0 : m[pp, pp];
                            m[ss, rr] = mmm[ss, rr] - mmm[ss, pp] * mmm[pp, rr] / mmm[pp, pp];
                        }
                    }
                    mmm = (double[,])m.Clone();
                }
            }


            return m;
        }

        //算出したT*k_e*Tを全て足し合わせて全体の剛性マトリクスを作成//
        public static double[,] MakeGlobalK(fAlldata falldata)
        {
            
            object[] fElemArray = falldata.fElemArray;
            int elem_number  = fElemArray.Length/32;

            // 各要素を6×6配列で初期化
            double[][,] elem_k = new double[4][,];
            int node_number = falldata.fNodeXYZArray.GetLength(0);//node数
            int global_k_size = node_number * 6;
            double[,] global_k = new double[global_k_size, global_k_size];
            for (int i = 0; i < elem_number; i++)
            {
                bool[] constraint = new bool[12];
                for (int jj = 0; jj < 12; jj++)
                {
                    //constraintはtrueが剛(固定)としている。iconも同じでtrueが固定
                    constraint[jj] = (bool)fAlldata.GetElemItem(fElemArray, i, 2, jj);
                }
                
                //部材端が剛剛の場合
                if (constraint.All(f => f))
                {
                    elem_k = MakeTKeTforFixFix(falldata, i);
                }

                else
                {
                    //剛剛以外の場合
                    double[,] m = MakeKe(falldata, i);
                    double[,] Coord = MakeCoord(fElemArray, i); 
                   
                    double[,] T = new double[12, 12];
                    MatrixUtils.AddPartMatrix(Coord, T, 0, 0);
                    MatrixUtils.AddPartMatrix(Coord, T, 3, 3);
                    MatrixUtils.AddPartMatrix(Coord, T, 6, 6);
                    MatrixUtils.AddPartMatrix(Coord, T, 9, 9);
                    double[,] g_elemk = MatrixUtils.Multiply(MatrixUtils.Transpose(T), MatrixUtils.Multiply(m, T));
                    elem_k[0] = MatrixUtils.ExtractPartMatrix(g_elemk, 0, 0, 6, 6);
                    elem_k[1] = MatrixUtils.ExtractPartMatrix(g_elemk, 0, 6, 6, 6);
                    elem_k[2] = MatrixUtils.ExtractPartMatrix(g_elemk, 6, 0, 6, 6);
                    elem_k[3] = MatrixUtils.ExtractPartMatrix(g_elemk, 6, 6, 6, 6);
                }

                int node_start1 = ((int)fAlldata.GetElemItem(fElemArray, i, 1, 0) - 1) * 6;//行列内ではnode番号*6番目からスタートする
                int node_start2 = ((int)fAlldata.GetElemItem(fElemArray, i, 1, 1) - 1) * 6;

                MatrixUtils.AddPartMatrix(elem_k[0], global_k, node_start1, node_start1);
                MatrixUtils.AddPartMatrix(elem_k[1], global_k, node_start1, node_start2);
                MatrixUtils.AddPartMatrix(elem_k[2], global_k, node_start2, node_start1);
                MatrixUtils.AddPartMatrix(elem_k[3], global_k, node_start2, node_start2);
            }

            return global_k;
        }

        //計算のために全体剛性マトリクスを並び変えて4つのブロックKff,Kfs,Ksf,Kssを作成する//
        /*
            k_ff = modified_global_k[0];
            k_fs = modified_global_k[1];
            k_sf = modified_global_k[2];
            k_ss = modified_global_k[3];
        */
        public static (double[,], double[,], double[,], double[,]) MakeModifiedGlobalK(fAlldata falldata)
        {
            double[,] global_k = MakeGlobalK(falldata);
            bool[] fNodeIConArray = falldata.fNodeIConArray;
            double[][,] modified_global_k = MatrixUtils.MakeModifiedMatrix(global_k, fNodeIConArray);
            return (modified_global_k[0], modified_global_k[1], modified_global_k[2], modified_global_k[3]);
        }
    }
}

