using Grasshopper.Kernel;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Collections;
using Rhino.Render.ChangeQueue;
using System;
using System.Collections.Generic;
using System.Linq;
using Hagoromo.MathTools;

namespace Hagoromo.DataTools
{
    public static class DataTools
    {
        // 座標を丸めて文字列化（誤差を吸収するため）
        //databaseの中にdataがあるか確認して、なかったものだけを返す。
        public static int[] GetMissingElements(List<int> database, int[] data)
        {
            List<int> missing = new List<int>();

            foreach (int d in data)
            {
                if (!database.Contains(d)) // databaseに含まれていないなら
                {
                    missing.Add(d);
                }
            }

            return missing.ToArray();
        }
    }
}
