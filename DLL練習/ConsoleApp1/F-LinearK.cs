using System;
using Hagoromo.MathTools;
using Hagoromo.DataStructure;
using System.Collections.Generic;

namespace Hagoromo.StiffnessTools
{
    public static class LinearK
    {

        //Coordされた要素剛性マトリクスを構成する部分作成のための関数//
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

        //要素剛性マトリクスのCoord後の行列を作成、有限要素法ハンドブックp219を参照、TkTを手計算した結果を打ち込んでいる。//
        public static double[][,] MakeCoordElemK(double theta, double l, double dx, double dy, double dz, double E, double A, double Ix, double Iy, double K, double G)
        {//要素剛性マトリクスに出てくる値をあらかじめ計算//
            double a = 2 * E * Ix / l;
            double b = 2 * a;
            double c = 3 * a / l;
            double d = 2 * c / l;
            double e = 2 * E * Iy / l;
            double f = 2 * e;
            double g = 3 * e / l;
            double h = 2 * g / l;
            double i = E * A / l;
            double j = G * K / l;

            //座標変換行列,座標変換行列とcoord angleについてはpdf「座標変換行列について」を参照//
            double ctheta = Math.Cos(Math.PI * theta / 180);//thetaはdegreeで入力//
            double stheta = Math.Sin(Math.PI * theta / 180);
            double nn = dz / l;//pdfのnまたはnxに対応//
            double[,] Coord;
            //部材が鉛直(全体のz軸と部材のx軸が平行の時//
            if (nn == -1 || nn == 1)
            {
                Coord = new double[3, 3]
                {
                    { 0,          0,      nn },
                    { nn*ctheta,  stheta, 0  },
                    { -nn*stheta, ctheta, 0  }
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
                    { ll,                                mm,                               nn             },
                    { -(mm*ctheta+nn*ll*stheta)/lmroot,  (ll*ctheta-nn*mm*stheta)/lmroot,  lmroot*stheta  },
                    { (mm*stheta-nn*ll*ctheta)/lmroot,   -(ll*stheta+nn*mm*ctheta)/lmroot, lmroot*ctheta  }
                };

            }

            //T*k_e*Tの部分行列を計算//
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
            MatrixUtils.CopyMatrix(coord_elem_type1, coord_elem_k_part11, 0, 0);
            MatrixUtils.CopyMatrix(coord_elem_type1, coord_elem_k_part22, 0, 0);
            MatrixUtils.CopyNegMatrix(coord_elem_type1, coord_elem_k_part12, 0, 0);
            MatrixUtils.CopyNegMatrix(coord_elem_type1, coord_elem_k_part21, 0, 0);

            MatrixUtils.CopyMatrix(coord_elem_type2, coord_elem_k_part11, 0, 3);
            MatrixUtils.CopyMatrix(coord_elem_type2, coord_elem_k_part12, 0, 3);
            MatrixUtils.CopyNegMatrix(coord_elem_type2, coord_elem_k_part21, 0, 3);
            MatrixUtils.CopyNegMatrix(coord_elem_type2, coord_elem_k_part22, 0, 3);

            MatrixUtils.CopyMatrix(coord_elem_type3, coord_elem_k_part11, 3, 0);
            MatrixUtils.CopyMatrix(coord_elem_type3, coord_elem_k_part21, 3, 0);
            MatrixUtils.CopyNegMatrix(coord_elem_type3, coord_elem_k_part12, 3, 0);
            MatrixUtils.CopyNegMatrix(coord_elem_type3, coord_elem_k_part22, 3, 0);

            MatrixUtils.CopyMatrix(coord_elem_type4, coord_elem_k_part11, 3, 3);
            MatrixUtils.CopyMatrix(coord_elem_type4, coord_elem_k_part22, 3, 3);

            MatrixUtils.CopyMatrix(coord_elem_type5, coord_elem_k_part12, 3, 3);
            MatrixUtils.CopyMatrix(coord_elem_type5, coord_elem_k_part21, 3, 3);

            double[][,] matrices = new double[4][,];
            for (int ii = 0; ii < matrices.Length; ii++)
            {
                matrices[ii] = new double[6, 6];  // 各要素を6×6配列で初期化
            }

            matrices[0] = coord_elem_k_part11;
            matrices[1] = coord_elem_k_part12;
            matrices[2] = coord_elem_k_part21;
            matrices[3] = coord_elem_k_part22;

            return matrices;
        }

        //算出したT*k_e*Tを全て足し合わせて全体の剛性マトリクスを作成//
        public static double[,] MakeGlobalK(fAlldata falldata)
        {

            object[] fElemArray = falldata.fElemArray;
            object[,] fSectArray = falldata.fSectArray;
            double[,] fPropArray = falldata.fPropArray;
            int node_number = falldata.fNodeXYZArray.GetLength(0);//node数
            int global_k_size = node_number * 6;
            double[,] global_k = new double[global_k_size, global_k_size];
            double[][,] elem_k = new double[4][,];
            for (int ii = 0; ii < 4; ii++)
            {
                elem_k[ii] = new double[6, 6];  // 各要素を6×6配列で初期化
            }
            for (int i = 0; i < node_number; i++)
            {
                int sectid = (int)fAlldata.GetElemItem(fElemArray, i, 0) - 11;
                int propid = (int)fSectArray[sectid,0] - 1;
                double G = fPropArray[propid,1] * 0.5 / (1 + fPropArray[propid, 2]);
                elem_k = MakeCoordElemK((double)fAlldata.GetElemItem(fElemArray,i,3,0), (double)fAlldata.GetElemItem(fElemArray, i, 3, 1), (double)fAlldata.GetElemItem(fElemArray, i, 3, 2),
                    (double)fAlldata.GetElemItem(fElemArray, i, 3, 3), (double)fAlldata.GetElemItem(fElemArray, i, 3, 4),
                    fPropArray[propid, 1], (double)fSectArray[sectid,1], (double)fSectArray[sectid, 2], (double)fSectArray[sectid, 3], (double)fSectArray[sectid, 4], G);
                int node_start1 = ((int)fAlldata.GetElemItem(fElemArray, i, 1, 0) - 101) * 6;//行列内ではnode番号*6番目からスタートする
                int node_start2 = ((int)fAlldata.GetElemItem(fElemArray, i, 1, 1) - 101) * 6;

                MatrixUtils.AddPartMatrix(elem_k[0], global_k, node_start1, node_start1);
                MatrixUtils.AddPartMatrix(elem_k[1], global_k, node_start1, node_start2);
                MatrixUtils.AddPartMatrix(elem_k[2], global_k, node_start2, node_start1);
                MatrixUtils.AddPartMatrix(elem_k[3], global_k, node_start2, node_start2);
            }

            return global_k;
        }

        //計算のために全体剛性マトリクスを並び変えて4つのブロックKff,Kfs,Ksf,Kssを作成する//
        /*public static void MakeModifiedGlobalK(fAlldata falldata, double[,] k_ff, double[,] k_fs, double[,] k_sf, double[,] k_ss)
        {
            double[,] global_k = MakeGlobalK(falldata);
            bool[] fNodeIConArray = falldata.fNodeIConArray;
            int global_k_size = global_k.GetLength(0);
            double[][,] modified_global_k = MatrixUtils.MakeModifiedMatrix(global_k, fNodeIConArray);
            k_ff = modified_global_k[0];
            k_fs = modified_global_k[1];
            k_sf = modified_global_k[2];
            k_ss = modified_global_k[3];
        }
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

