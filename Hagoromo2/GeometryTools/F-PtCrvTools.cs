using Grasshopper.Kernel;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Collections;
using Rhino.Render.ChangeQueue;
using System;
using System.Collections.Generic;
using System.Linq;
using Hagoromo.MathTools;

namespace Hagoromo.GeometryTools
{
    public static class PtCrvTools
    {
        // 座標を丸めて文字列化（誤差を吸収するため）
        public static string KeyFromPoint(Point3d pt, double tol)
        {
            double x = Math.Round(pt.X / tol) * tol;
            double y = Math.Round(pt.Y / tol) * tol;
            double z = Math.Round(pt.Z / tol) * tol;
            return $"{x},{y},{z}";
        }

        //Polylineの各点が点群の中にあったら、その点群でのindexを返す関数
        public static int[] GetOutlineIndices(Point3d[] newTopoVertices, Polyline outline)
        {
            BoundingBox bb = new BoundingBox(newTopoVertices);
            double tol = bb.Diagonal.Length * 1e-5;

            // まず辞書を作る: key = 丸めた座標, value = インデックス
            var dict = new Dictionary<string, int>();
            for (int i = 0; i < newTopoVertices.Length; i++)
            {
                string key = KeyFromPoint(newTopoVertices[i], tol);
                if (!dict.ContainsKey(key))  // 重複防止
                    dict[key] = i;
            }

            // outline の点を辞書で検索
            List<int> indices = new List<int>();
            foreach (Point3d p in outline)
            {
                string key = KeyFromPoint(p, tol);
                if (dict.TryGetValue(key, out int idx))
                    indices.Add(idx);
                else
                    RhinoApp.WriteLine($"Warning: outline point {p} not found in newTopoVertices");
            }

            return indices.ToArray();
        }

        public static Point3d[] Convert2Dto3D(double[][] newTopoVertices2D)
        {
            Point3d[] newTopoVertices = new Point3d[newTopoVertices2D.Length];
            for (int i = 0; i < newTopoVertices2D.Length; i++)
            {
                newTopoVertices[i].X = newTopoVertices2D[i][0];
                newTopoVertices[i].Y = newTopoVertices2D[i][1];
                newTopoVertices[i].Z = 0;
            }
            return newTopoVertices;
        }

        public static bool IsSameLine(Line edgeLine, Line cutLine, double tol = 1e-6)
        {
            // 始点終点の両方の距離が十分小さい or 逆順でも十分小さいなら一致とみなす
            bool sameDir = edgeLine.From.DistanceTo(cutLine.From) < tol &&
                            edgeLine.To.DistanceTo(cutLine.To) < tol;

            bool oppDir = edgeLine.From.DistanceTo(cutLine.To) < tol &&
                            edgeLine.To.DistanceTo(cutLine.From) < tol;

            return sameDir || oppDir;
        }
    }
}
