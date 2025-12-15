using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Hagoromo.GeometryTools;
using Hagoromo.MathTools;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Collections;
using Rhino.Render.ChangeQueue;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Hagoromo.DevelopableMesh
{
    public class CheckConstraintsValue : GH_Component
    {
        public CheckConstraintsValue()
          : base("CheckConstraintsValue", "ConsValue",
              "CheckConstraintsValue",
              "Hagoromo", "Optimization")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated CutMesh", "(C)M", "mesh to develop", GH_ParamAccess.item);
            pManager.AddIntegerParameter("iterations", "i", "NRiteration", GH_ParamAccess.item);
            pManager.AddBooleanParameter("xyMirror", "xy", "xyMirror", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("yzMirror", "yz", "yzMirror", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("zxMirror", "zx", "zxMirror", GH_ParamAccess.item, false);
            pManager.AddCurveParameter("outerBoundaryCurves", "outerB", "outerBoundaryCurves", GH_ParamAccess.list);
            int fixCrvIndex = pManager.AddCurveParameter("fixPointsCrv", "fixCrv", "fixPointsCrv", GH_ParamAccess.list);
            pManager[6].Optional = true;
            pManager.AddNumberParameter("w", "weight", "weight", GH_ParamAccess.list);
        }


        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            //pManager.AddMeshParameter("Developed Mesh", "M", "developed planer mesh", GH_ParamAccess.item);
            pManager.AddIntegerParameter("vertToConstraints", "vertToConstraints", "vertToConstraints", GH_ParamAccess.list);
            pManager.AddIntegerParameter("constraintsOrder", "constraintsOrder", "constraintsOrder", GH_ParamAccess.list);
            pManager.AddIntegerParameter("vertOrderInDup", "vertOrderInDup", "vertOrderInDup", GH_ParamAccess.list);
            pManager.AddIntegerParameter("dupVertIndices", "dupVertIndices", "dupVertIndices", GH_ParamAccess.tree);
            pManager.AddIntegerParameter("devConsCount", "devConsCount", "devConstCount", GH_ParamAccess.list);
            //pManager.AddIntegerParameter("a", "a", "a", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            object input = null;
            if (!DA.GetData(0, ref input)) return;

            CutMesh cutMesh = null;

            if (input is IGH_Goo goo)
            {
                // Mesh へのキャストを試す
                if (goo.CastTo(out Rhino.Geometry.Mesh mm))
                {
                    cutMesh = new CutMesh(mm);
                }
                // CutMesh へのキャストを試す
                else if (goo.CastTo(out CutMesh cm))
                {
                    cutMesh = cm.Clone();
                }
            }

            if (cutMesh == null)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Mesh または CutMesh を入力してください");
                return;
            }
            cutMesh = cutMesh.Sort();
            int iterations = 0;
            DA.GetData(1, ref iterations);
            bool xyMirror = false;
            DA.GetData(2, ref xyMirror);
            bool yzMirror = false;
            DA.GetData(3, ref yzMirror);
            bool zxMirror = false;
            DA.GetData(4, ref zxMirror);

            List<Curve> outerCrvs = new List<Curve>();
            DA.GetDataList(5, outerCrvs);
            List<Point3d> outerPoints = new List<Point3d>();
            foreach (Curve crv in outerCrvs)
            {
                if (crv.TryGetPolyline(out Polyline polyline))
                {
                    outerPoints.AddRange(polyline);
                }
            }

            List<Curve> fixCrvs = new List<Curve>();
            bool hasInput = DA.GetDataList(6, fixCrvs);
            if (!hasInput) { fixCrvs = new List<Curve>(); }
            List<Point3d> fixPoints = new List<Point3d>();
            foreach (Curve crv in fixCrvs)
            {
                if (crv.TryGetPolyline(out Polyline polyline))
                {
                    fixPoints.AddRange(polyline);
                }
            }

            List<Point3d> vertices = cutMesh.Vertices;

            HashSet<int> outerIndices = new HashSet<int>();
            foreach (Point3d fixPt in outerPoints)
            {
                double minDist = double.MaxValue;
                int closestIndex = -1;

                for (int i = 0; i < vertices.Count; i++)
                {
                    double dist = fixPt.DistanceToSquared(vertices[i]);
                    if (dist < minDist)
                    {
                        minDist = dist;
                        closestIndex = i;
                    }
                }

                if (closestIndex != -1)
                    outerIndices.Add(closestIndex);
            }
            List<int> outerVertIndices = outerIndices.ToList();

            HashSet<int> fixIndices = new HashSet<int>();
            foreach (Point3d fixPt in fixPoints)
            {
                double minDist = double.MaxValue;
                int closestIndex = -1;

                for (int i = 0; i < vertices.Count; i++)
                {
                    double dist = fixPt.DistanceToSquared(vertices[i]);
                    if (dist < minDist)
                    {
                        minDist = dist;
                        closestIndex = i;
                    }
                }

                if (closestIndex != -1)
                    fixIndices.Add(closestIndex);
            }
            List<int> fixVertIndices = fixIndices.ToList();
            List<List<int>> dup = cutMesh.DuplicatedVertIndices;

            HashSet<int> expandedSet = new HashSet<int>(fixVertIndices);
            foreach (int idx in fixVertIndices)
            {
                foreach (List<int> group in dup)
                {
                    if (group.Contains(idx))
                    {
                        foreach (int other in group)
                        {
                            expandedSet.Add(other);
                        }
                    }
                }
            }
            List<int> sortedFixVertIndices = expandedSet.ToList();
            sortedFixVertIndices.Sort();


            HashSet<int> expandedSet2 = new HashSet<int>(outerVertIndices);
            foreach (int idx in outerVertIndices)
            {
                foreach (List<int> group in dup)
                {
                    if (group.Contains(idx))
                    {
                        foreach (int other in group)
                        {
                            expandedSet2.Add(other);
                        }
                    }
                }
            }
            List<int> sortedOuterVertIndices = expandedSet2.ToList();
            sortedOuterVertIndices.Sort();

            List<double> wList = new List<double>();
            DA.GetDataList(7, wList);
            double[] w = wList.ToArray();
            // 以降 cutMesh が確実に利用可能




            double w0 = w[0];
            double w1 = w[1];
            double w2 = w[2];
            double w3 = w[3];
            double w4 = w[4];
            double w5 = w[5];
            double w6 = w[6];
            /*----------------------------------------------------A～Fの分類-----------------------------------------------------------------------*/
            List<List<List<int>>> category = CurvatureTools.CategolizeCutMesh(cutMesh, xyMirror, yzMirror, zxMirror, sortedOuterVertIndices, sortedFixVertIndices);
            List<int> B = category[0][1];
            List<int> C = category[0][2];
            List<int> D = category[0][3];
            List<int> E = category[0][4];
            List<int> F = category[0][5];
            List<int> Axy = category[0][6];
            List<int> Ayz = category[0][7];
            List<int> Azx = category[0][8];
            List<int> Afree = category[0][9];
            List<int> Bx = category[0][10];
            List<int> By = category[0][11];
            List<int> Bz = category[0][12];
            List<int> Cxy = category[0][13];
            List<int> Cyz = category[0][14];
            List<int> Czx = category[0][15];
            List<List<int>> DoneMirror = category[1];
            List<List<int>> xyMirrorInnerCut = category[2];
            List<List<int>> yzMirrorInnerCut = category[3];
            List<List<int>> zxMirrorInnerCut = category[4];
            List<List<int>> DtwoMirror = category[5];
            List<List<int>> xyYZinnerCut = category[6];
            List<List<int>> yzZXinnerCut = category[7];
            List<List<int>> zxXYinnerCut = category[8];
            List<List<int>> Fblocks = category[9];


            /*--------トポロジカルに変わらなければ上記の分類は変わらない--------------------------------------------------------*/
            List<List<int>> duplicatedVertIndices = cutMesh.DuplicatedVertIndices;
            int dupVertCount = duplicatedVertIndices.Count;
            int n = dupVertCount * 3;
            //ｍは制約条件+エネルギー関数の個数。m行の順番はdupVertの上から順、ループの時もループ内で一番最初に出てくるdupVertの位置を基準とする。
            //ループの時は、角度の和の条件、sinの条件、cosの条件の順とする
            int devConstraintCount = B.Count + C.Count + E.Count + DoneMirror.Count * 2 + DtwoMirror.Count + Fblocks.Count * 3;
            int edgeCount = cutMesh.Edges.Count;

            //エネルギー関数の数
            int energyCount = edgeCount;
            //developable以外の制約条件の数
            int constraintMoreCount = 0;
            int m = devConstraintCount + constraintMoreCount + energyCount;


            //あやしい
            int[] constraintsOrder = new int[dupVertCount];
            for (int i = 0; i < constraintsOrder.Length; i++)
            {
                constraintsOrder[i] = -1;
            }
            int guide = 0;
            List<int> skipVert = new List<int>();
            int iii = 0;
            foreach (List<int> vertGroup in duplicatedVertIndices)
            {
                if (skipVert.Contains(vertGroup[0])) { iii += 1; continue; }
                if (B.Contains(vertGroup[0])) { constraintsOrder[iii] = guide; guide += 1; iii += 1; continue; }
                if (C.Contains(vertGroup[0])) { constraintsOrder[iii] = guide; guide += 1; iii += 1; continue; }
                if (E.Contains(vertGroup[0])) { constraintsOrder[iii] = guide; guide += 1; iii += 1; continue; }

                List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(vertGroup[0]));
                if (found != null)
                {
                    foreach (int f in found)
                    {
                        for (int i = 0; i < dupVertCount; i++)
                        {
                            if (duplicatedVertIndices[i].Contains(f))
                            {
                                constraintsOrder[i] = guide; // f は i 番目のサブリストにある
                                break;
                            }
                        }
                    }

                    guide += 2;
                    skipVert.AddRange(found);
                    iii += 1;
                    continue;
                }

                found = DtwoMirror.FirstOrDefault(sub => sub.Contains(vertGroup[0]));
                if (found != null)
                {
                    foreach (int f in found)
                    {
                        for (int i = 0; i < dupVertCount; i++)
                        {
                            if (duplicatedVertIndices[i].Contains(f))
                            {
                                constraintsOrder[i] = guide; // f は i 番目のサブリストにある
                                break;
                            }
                        }
                    }
                    guide += 1;
                    skipVert.AddRange(found);
                    iii += 1;
                    continue;
                }

                found = Fblocks.FirstOrDefault(sub => sub.Contains(vertGroup[0]));
                if (found != null)
                {
                    foreach (int f in found)
                    {
                        for (int i = 0; i < dupVertCount; i++)
                        {
                            if (duplicatedVertIndices[i].Contains(f))
                            {
                                constraintsOrder[i] = guide; // f は i 番目のサブリストにある
                                break;
                            }
                        }
                    }
                    guide += 3;
                    skipVert.AddRange(found);
                    iii += 1;
                    continue;
                }
                iii += 1;
            }
            //あやしい終わり


            //verticesのi番目の点がdupVertの何番目のグループの点なのかの対応のリスト
            int[] vertOrderInDup = new int[cutMesh.Vertices.Count];
            for (int i = 0; i < vertOrderInDup.Length; i++)
            {
                vertOrderInDup[i] = -1;
            }
            for (int i = 0; i < duplicatedVertIndices.Count; i++)
            {
                foreach (int vertIndex in duplicatedVertIndices[i])
                {
                    vertOrderInDup[vertIndex] = i;
                }
            }

            //verticesのi番目の点がconstraintsの何行目(～何行目)なのかの対応のリスト
            int[] vertToConstraints = new int[cutMesh.Vertices.Count];
            for (int i = 0; i < vertToConstraints.Length; i++)
            {
                vertToConstraints[i] = -1;
            }
            for (int i = 0; i < cutMesh.Vertices.Count; i++)
            {
                vertToConstraints[i] = constraintsOrder[vertOrderInDup[i]];
            }
            List<int> list = vertToConstraints.ToList();
            List<int> list2 = constraintsOrder.ToList();
            List<int> list3 = vertOrderInDup.ToList();
            List<int> flatList = duplicatedVertIndices.SelectMany(x => x).ToList();
            DataTree<int> tree = ListOfListsToTree(duplicatedVertIndices);
            //CutMesh newMesh = CurvatureTools.CGNRConsiderOther(cutMesh, iterations, xyMirror, yzMirror, zxMirror, sortedOuterVertIndices, sortedFixVertIndices, w);
            DA.SetDataList(0, list);
            DA.SetDataList(1, list2);
            DA.SetDataList(2, list3);
            DA.SetDataTree(3, tree);
            DA.SetData(4, devConstraintCount);
            //DA.SetDataList(0, sortedOuterVertIndices);
        }



        public override Guid ComponentGuid
        {
            get { return new Guid("9F2424BB-049A-4770-A86E-84FB45C4FB01"); }
        }

        public DataTree<int> ListOfListsToTree(List<List<int>> listOfLists)
        {
            DataTree<int> tree = new DataTree<int>();

            for (int i = 0; i < listOfLists.Count; i++)
            {
                // GH_Pathはツリーのパス（枝）を表します: {0}, {1}, {2}...
                GH_Path path = new GH_Path(i);

                // そのパスにリストの中身をまとめて追加
                tree.AddRange(listOfLists[i], path);
            }

            return tree;
        }
    }
}