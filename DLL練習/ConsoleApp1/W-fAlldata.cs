using Grasshopper.Kernel.Types;
using System;
using System.Collections.Generic;

namespace Hagoromo.DataStructure
{
    public class fAlldata
    {
        public double[,] fPropArray { get; set; }
        public object[,] fSectArray { get; set; }
        public double[,] fNodeXYZArray { get; set; }
        public bool[] fNodeIConArray { get; set; }
        public double[] fNodeVConArray { get; set; }
        public object[] fElemArray { get; set; }

        public fAlldata(double[,] fproparray, object[,] fsectarray, double[,] fnodeXYZarray, bool[] fnodeIConarray, double[] fnodeVConarray, object[] felemarray)
        {
            if (fproparray == null)
                throw new ArgumentNullException(nameof(fproparray), "Property array cannot be null.");
            if (fsectarray == null)
                throw new ArgumentNullException(nameof(fsectarray), "Section array cannot be null.");
            if (fnodeXYZarray == null)
                throw new ArgumentNullException(nameof(fnodeXYZarray), "NodeXYZ array cannot be null.");
            if (fnodeIConarray == null)
                throw new ArgumentNullException(nameof(fnodeIConarray), "NodeICon array cannot be null.");
            if (fnodeVConarray == null)
                throw new ArgumentNullException(nameof(fnodeVConarray), "NodeVCon array cannot be null.");
            if (felemarray == null)
                throw new ArgumentNullException(nameof(felemarray), "Element array cannot be null.");

            fPropArray = fproparray;
            fSectArray = fsectarray;
            fNodeXYZArray = fnodeXYZarray;
            fNodeIConArray = fnodeIConarray;
            fNodeVConArray = fnodeVConarray;
            fElemArray = felemarray;
        }

        public fAlldata(Alldata alldata)
        {
            fElemArray = FlattenElem(alldata.ElementArray);
            fNodeXYZArray = FlattenNodeXYZ(alldata.NodeArray);
            fNodeIConArray = FlattenNodeICon(alldata.NodeArray);
            fNodeVConArray = FlattenNodeVCon(alldata.NodeArray);
            fPropArray = FlattenProp(alldata.PropertyArray);
            fSectArray  =FlattenSect(alldata.SectionArray);
        }

        public static object[] FlattenElem(object[,] elemarray)
        {
            object[] fElemArray = new object[elemarray.GetLength(0) * 32];
            for (int i = 0; i < elemarray.GetLength(0); i++)
            {
                fElemArray[i * 32] = elemarray[i, 0];
                int[] nodesid = (int[])elemarray[i, 1];
                fElemArray[i * 32 + 1] = nodesid[0];
                fElemArray[i * 32 + 2] = nodesid[1];
                int[] ends = (int[])elemarray[i, 2];
                for (int j = 0; j < 12; j++)
                {
                    fElemArray[i * 32 + j + 3] = ends[j];
                }
                double[] coord = (double[])elemarray[i, 3];
                for (int j = 0; j < 5; j++)
                {
                    fElemArray[i * 32 + j + 15] = coord[j];
                }
                double[] cmq = (double[])elemarray[i, 4];
                for (int j = 0; j < 12; j++)
                {
                    fElemArray[i * 32 + j + 20] = cmq[j];
                }
            }
            return fElemArray;
        }

        public static object GetElemItem(object[] fElemArray, int i, int j, int k = 0)
        {
            if (j == 0 || j == 1)
            {
                return fElemArray[i * 32 + j + k];
            }
            else if (j == 2)
            {
                return fElemArray[i * 32 + 3 + k];
            }
            else if (j == 3)
            {
                return fElemArray[i * 32 + 15 + k];
            }
            else if (j == 4)
            {
                return fElemArray[i * 32 + 20 + k];
            }
            return null;
        }

        public static double[,] FlattenNodeXYZ(object[,] nodearray)/*node座標を[,]で配列*/
        {
            double[,] fNodeXYZArray = new double[nodearray.GetLength(0), 3];
            for (int i = 0; i < nodearray.GetLength(0); i++)
            {
                double[] xyz = (double[])nodearray[i, 0];
                fNodeXYZArray[i, 0] = xyz[0];
                fNodeXYZArray[i, 1] = xyz[1];
                fNodeXYZArray[i, 2] = xyz[2];
            }
            return fNodeXYZArray;
        }

        public static bool[] FlattenNodeICon(object[,] nodearray)/*node座標羅列の配列とboolのicondataとdoubleの配列のvconを用意*/
        {
            bool[] fNodeIConArray = new bool[nodearray.GetLength(0) * 6];
            for (int i = 0; i < nodearray.GetLength(0); i++)
            {
                bool[] icon = (bool[])nodearray[i, 1];
                for (int j = 0; j < 6; j++)
                {
                    fNodeIConArray[i*6 + j] = icon[j];
                }
            }
            return fNodeIConArray;
        }

        public static bool GetNodeICon(bool[] fNodeIConArray, int i, int j)/*node座標羅列の配列とboolのicondataとdoubleの配列のvconを用意*/
        {
            return fNodeIConArray[i * 6 + j];
        }

        public static double[] FlattenNodeVCon(object[,] nodearray)/*node座標羅列の配列とboolのicondataとdoubleの配列のvconを用意*/
        {
            double[] fNodeVConArray = new double[nodearray.GetLength(0) * 6];
            for (int i = 0; i < nodearray.GetLength(0); i++)
            {
                double[] vcon = (double[])nodearray[i, 2];
                for (int j = 0; j < 6; j++)
                {
                    fNodeVConArray[i*6 + j] = vcon[j];
                }
            }
            return fNodeVConArray;
        }

        public static double GetNodeVCon(double[] fNodeVConArray, int i, int j)/*node座標羅列の配列とboolのicondataとdoubleの配列のvconを用意*/
        {
            return fNodeVConArray[i * 6 + j];
        }

        public static double[,] FlattenProp(object[,] proparray)/*nameとpropidは不要なので省く,２次元配列にしておく*/
        {
            double[,] fPropArray = new double[proparray.GetLength(0), 3];
            for (int i = 0; i < proparray.GetLength(0); i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    fPropArray[i, j] = (double)proparray[i, j + 2];
                }
            }
            return fPropArray;
        }

        public static object[,] FlattenSect(object[,] sectarray)/*nameとsectidは不要なので省く,二次元配列にしておく*/
        {
            object[,] fSectArray = new object[sectarray.GetLength(0), 5];
            for (int i = 0; i < sectarray.GetLength(0); i++)
            {
                for (int j = 0; j < 5; j++)
                {
                    fSectArray[i, j] = sectarray[i, j + 2];
                }
            }
            return fSectArray;
        }


    }


}