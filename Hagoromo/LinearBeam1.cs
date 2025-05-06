using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Hagoromo
{
    public class LinearBeam1 : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public LinearBeam1()
          : base("LinearBeam1", "Nickname",
              "軸、曲げ、サンブナンねじりのみ考慮(Hoganと同じ)の線形解析",
              "Category", "Subcategory")
        {
        }

        // 行列を別の行列にコピーする関数
        static void CopyMatrix(double[,] source, double[,] destination, int startRow, int startCol)
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

        // 行列の-1倍を別の行列にコピーする関数
        static void CopyNegMatrix(double[,] source, double[,] destination, int startRow, int startCol)
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
        static void AddPartMatrix(double[,] source, double[,] destination, int startRow, int startCol)
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

        //p=Kuのp,uの並び替えに伴ってKを並び変えて4つのブロックで出力する関数//
        static double[,,] MakeModifiedMatrix(double[,] matrix, bool[] flags)
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

            double[,,] matrices = [matrix_ff, matrix_fs, matrix_sf, matrix_ss];
            return matrices;
        }

        //連立一次方程式Ax=bをLDL分解で解く,ただしAは実対称行列の下三角を上の行から順に行ずつで並べた配列//
        static double[] SolveByLDL(double[] A, double[] b)
        {
            int n = b.Length;//nは行列のサイズ//
            double[] U = new double[n*(n+1)/2];//Uは上三角成分のみを下の行から順に行ずつで並べた配列//
            double[] L = new double[n*(n-1)/2];//Lは下三角成分(1のところも省く)を上の行から順に行ずつで並べた配列//

            //次のfor文で帰納的にU,Lを求めるが最初にその初項を設定//
            int t = n * (n - 1) / 2;
            for (int j = 1; j <= n; i++)
            {
                U[t + j - 1] = A[j - 1]; //u_1j=a_1j//
                L[j * (j - 3) / 2 + 1] = U[t + j - 1] / U[t]; //l_j1=u_1j/u_11//
            }

            //U,Lをそれぞれ求める//
            int s = 0;
            int id = 0;
            for (int i = 2; i <= n; i++)
            {
                for (int j = i; j <= n; j++)
                {
                    s = 0;
                    for (int k = 1; k <= i - 1; k++)
                    {
                        s += L[i * (i - 3) / 2 + k] * U[(n - k) * (n - k + 1) + j - k];
                    }
                    id = (n - i) * (n - i + 1) / 2 + j - i;
                    U[id] = A[j * (j - 1) + i - 1] - s;//u_ijを求める//

                    if (i != j)
                    {
                        L[j * (j - 3) / 2 + i] = U[id] / U[(n - i) * (n - i + 1) / 2]//l_ijを求める//
                    }
                }
            }

            //ガウスの前進消去法でyを求める、yが順に求まり次第、メモリ削減のためにそれをbに上書きしていく//
            //y1=b1なので初項は省略できる//
            for (int i = 2;i<= n; i++)
            {
                s = 0;
                for (int j = 1; j <= i-1; j++)
                {
                    s += L[i * (i - 3) / 2 + j] * b[j - 1];
                }
                b[i - 1] -= s;
            }

            //後退代入でxを求める、xが後ろから順に求まり次第、メモリ削減のためにそれをbに上書きしていく//
            b[n - 1] = b[n - 1] / U[0];
            for (int i = 1; i <= n-1; i++)
            {
                s = 0;
                for (int j = n-i+1; j <= n; j++)
                {
                    s += U[i * (i + 3) / 2 + j - n] * b[j - 1];
                }
                b[n - i - 1] = (b[n - i - 1] - s) / U[i * (i + 1) / 2];
            }
            return b;
        }

        //Coordされた要素剛性マトリクスを構成する部分作成のための関数//
        static double[,] MakeCoordElemPartType1(double[,] Coord, double d, double h, double i)
        {
            double[,] coord_elem_part1 = new double[3, 3]
            {
                { Coord[0,0]*Coord[0,0]*d+Coord[1,0]*Coord[1,0]*h+Coord[2,0]*Coord[2,0]*i, Coord[0,0]*Coord[0,1]*d+Coord[1,0]*Coord[1,1]*h+Coord[2,0]*Coord[2,1]*i, Coord[0,0]*Coord[0,2]*d+Coord[1,0]*Coord[1,2]*h+Coord[2,0]*Coord[2,2]*i },
                { Coord[0,0]*Coord[0,1]*d+Coord[1,0]*Coord[1,1]*h+Coord[2,0]*Coord[2,1]*i, Coord[0,1]*Coord[0,1]*d+Coord[1,1]*Coord[1,1]*h+Coord[2,1]*Coord[2,1]*i, Coord[0,1]*Coord[0,2]*d+Coord[1,1]*Coord[1,2]*h+Coord[2,1]*Coord[2,2]*i },
                { Coord[0,0]*Coord[0,2]*d+Coord[1,0]*Coord[1,2]*h+Coord[2,0]*Coord[2,2]*i, Coord[0,2]*Coord[0,1]*d+Coord[1,2]*Coord[1,1]*h+Coord[2,2]*Coord[2,1]*i, Coord[0,2]*Coord[0,2]*d+Coord[1,2]*Coord[1,2]*h+Coord[2,2]*Coord[2,2]*i }
            };

            return coord_elem_part1;
        }

        static double[,] MakeCoordElemPartType2(double[,] Coord, double a, double b)
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
        static double[,,] MakeCoordElemK(double theta, double l, double dx, double dy, double dz, double E, double A, double Ix, double Iy, double K, double G)
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
            double ctheta = Math.Cos(Math.PI * theta / 180);//thataはdegreeで入力//
            double stheta = Math.Sin(Math.PI * theta / 180);
            double nn = dz / l;//pdfのnまたはnxに対応//
            //部材が鉛直(全体のz軸と部材のx軸が平行の時//
            if (nn ==-1 || nn= 1)
            {
                double[,] Coord = new double[3, 3]
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
                double[,] Coord = new double[3, 3]
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
            CopyMatrix(coord_elem_type1, coord_elem_k_part11, 0, 0);
            CopyMatrix(coord_elem_type1, coord_elem_k_part22, 0, 0);
            CopyNegMatrix(coord_elem_type1, coord_elem_k_part12, 0, 0);
            CopyNegMatrix(coord_elem_type1, coord_elem_k_part21, 0, 0);

            CopyMatrix(coord_elem_type2, coord_elem_k_part11, 0, 3);
            CopyMatrix(coord_elem_type2, coord_elem_k_part12, 0, 3);
            CopyNegMatrix(coord_elem_type2, coord_elem_k_part21, 0, 3);
            CopyNegMatrix(coord_elem_type2, coord_elem_k_part22, 0, 3);

            CopyMatrix(coord_elem_type3, coord_elem_k_part11, 3, 0);
            CopyMatrix(coord_elem_type3, coord_elem_k_part21, 3, 0);
            CopyNegMatrix(coord_elem_type3, coord_elem_k_part12, 3, 0);
            CopyNegMatrix(coord_elem_type3, coord_elem_k_part22, 3, 0);

            CopyMatrix(coord_elem_type4, coord_elem_k_part11, 3, 3);
            CopyMatrix(coord_elem_type4, coord_elem_k_part22, 3, 3);

            CopyMatrix(coord_elem_type5, coord_elem_k_part12, 3, 3);
            CopyMatrix(coord_elem_type5, coord_elem_k_part21, 3, 3);

            double[,,] matrices = new double[4,6,6];
            matrices[0] = coord_elem_k_part11;
            matrices[1] = coord_elem_k_part12;
            matrices[2] = coord_elem_k_part21;
            matrices[3] = coord_elem_k_part22;

            return matrices;
        }

        //算出したT*k_e*Tを全て足し合わせて全体の剛性マトリクスを作成//
        static double[,] MakeGlobalK(Alldata alldata)
        {
            int node_number = alldata.NodeArray.GetLength(0);
            int global_k_size = node_number * 6;
            double[,] global_k = new double[global_k_size, global_k_size];
            double[,,] elem_k = new double[4, 6, 6];
            for (int i = 0; i < node_number; i++)
            {
                double[] element = alldata.ElementArray[i];
                double[] section = alldata.SectionArray[element[0] - 11];
                double[] property = alldata.PropertyArray[section[2] - 1];
                double G = property[3] * 0.5 / (1 + property[4])
                elem_k = MakeCoordElemK(element[3][0], element[3][1], element[3][2], element[3][3], element[3][4], property[3], section[3], section[4], section[5], section[6], G);
                int node_start1 = (element[1][0] - 101) * 6;
                int node_start2 = (element[1][1] - 101) * 6;

                AddPartMatrix(elem_k[0], global_k, node_start1, node_start1);
                AddPartMatrix(elem_k[1], global_k, node_start1, node_start2);
                AddPartMatrix(elem_k[2], global_k, node_start2, node_start1);
                AddPartMatrix(elem_k[3], global_k, node_start2, node_start2);
            }

            return global_k;
        }

        //Pfを算出する、CMQの分配、Vcon、自重の考慮//

        //計算のために全体剛性マトリクスを並び変えて4つのブロックKff,Kfs,Ksf,Kssを作成する//
        static double[,,] MakeModifiedGlobalK(Alldata alldata)
        {
            double[,] global_k = MakeGlobalK(alldata);
            int global_k_size = global_k.GetLength(0);
            bool[] icondata = new bool[global_k_size];
            for (int i = 0; i < global_k_size / 6; i++)
            {
                for (int j = 0; j < 6; j++)
                { if (global_k[i, j]
                    icondata[i * 6 + j] = alldata.NodeArray[i][1][j];
                }
            }
            double[,,] modified_global_k = MakeModifiedMatrix(global_k, icondata);
            double[,] k_ff = modified_global_k[0];
            double[,] k_fs = modified_global_k[1];
            double[,] k_sf = modified_global_k[2];
            double[,] k_ss = modified_global_k[3];
        }

        //連立一次方程式を解く//


        //iconの型をboolに直す、nodearrayなどのデータ型を[,]にする。[][]ではない、データ型作り直す//

        //C++で記述しておいて、UIだけC#で行う。奥村のやってくれたデータ構成の部分をC#からC++にデータを読み込ませるときに同時に作成させる//






            /// <summary>
            /// Registers all the input parameters for this component.
            /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {

        }

        /// <summary>
        /// Provides an Icon for the component.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                //You can add image files to your project resources and access them like this:
                // return Resources.IconForThisComponent;
                return null;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("F6B3F263-6011-418C-ABF6-A61889AE225D"); }
        }
    }
}