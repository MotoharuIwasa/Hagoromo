using Grasshopper.Kernel;
using Grasshopper.Kernel.Geometry.Delaunay;
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
    public class MyComponent1 : GH_Component
    {
        public MyComponent1()
          : base("CheckRuling", "CRuling",
              "ルーリングをチェックします",
              "Hagoromo", "Optimization")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Developable CutMesh", "(C)M", "mesh to rule", GH_ParamAccess.item);
            pManager.AddNumberParameter("width", "D", "rectangleWidth", GH_ParamAccess.item);
            pManager.AddCurveParameter("Lines", "L", "Boundaries and Orisen. 線織面の境界線", GH_ParamAccess.list);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            //pManager.AddIntegerParameter("Rulings", "R", "possible first rulings", GH_ParamAccess.list);
            pManager.AddCurveParameter("Rulings", "R", "possible first rulings", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            //--------------------------------------入力したライン上の節点のインデックスのリストアップ-----------------
            List<Curve> curves = new List<Curve>();
            if (!DA.GetDataList(2, curves)) return;

            List<Polyline> cutLines = new List<Polyline>();

            foreach (var crv in curves)
            {
                if (crv.TryGetPolyline(out Polyline pl))
                {
                    cutLines.Add(pl);
                }
                else
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "One of the input curves is not a polyline.");
                }
            }
            CutMesh cutMesh = null;
            object input = null;
            List<int> edgeIndices = new List<int>();
            if (!DA.GetData(0, ref input)) return;
            if (input is IGH_Goo goo)
            {
                // CutMesh へのキャストを試す
                if (goo.CastTo(out cutMesh))
                {
                    cutMesh = cutMesh.Sort();
                    RTree rtree = new RTree();
                    for (int i = 0; i < cutMesh.Vertices.Count; i++)
                    {
                        rtree.Insert(cutMesh.Vertices[i], i);
                    }

                    edgeIndices = new List<int>();
                    foreach (Polyline polyline in cutLines)
                    {
                        List<int> topoVertsOnPolyline = new List<int>();
                        foreach (Point3d pt in polyline)
                        {
                            int closestTv = -1;
                            double minDist = double.MaxValue;

                            double tol = 0.001; // 許容誤差
                            BoundingBox box = new BoundingBox(
                                new Point3d(pt.X - tol, pt.Y - tol, pt.Z - tol),
                                new Point3d(pt.X + tol, pt.Y + tol, pt.Z + tol)
                            );

                            rtree.Search(box, (sender, args) =>
                            {
                                double d = pt.DistanceTo(cutMesh.Vertices[args.Id]);
                                if (d < minDist)
                                {
                                    minDist = d;
                                    closestTv = args.Id;
                                }
                            });

                            topoVertsOnPolyline.Add(closestTv);
                        }

                        for (int i = 0; i < topoVertsOnPolyline.Count - 1; i++)
                        {
                            List<int> vAList = (cutMesh.DuplicatedVertIndices.Where(inner => inner.Contains(topoVertsOnPolyline[i])).ToList())[0];
                            List<int> vBList = (cutMesh.DuplicatedVertIndices.Where(inner => inner.Contains(topoVertsOnPolyline[i + 1])).ToList())[0];

                            // vA に接続するエッジの中から vB を探す
                            foreach (int vA in vAList)
                            {
                                List<int> connectedEdges = cutMesh.GetEdgesForVertex(vA);
                                foreach (int ei in connectedEdges)
                                {
                                    if ((cutMesh.Edges[ei][0] == vA && vBList.Contains(cutMesh.Edges[ei][1])) || (vBList.Contains(cutMesh.Edges[ei][0]) && cutMesh.Edges[ei][1] == vA))
                                    {
                                        if (!edgeIndices.Contains(ei))
                                            edgeIndices.Add(ei);
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            if (cutMesh == null)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Mesh または CutMesh を入力してください");
                return;
            }

            HashSet<int> vertIndices2 = new HashSet<int>();
            foreach (int edgeIndex in edgeIndices)
            {
                vertIndices2.Add(cutMesh.Edges[edgeIndex][0]);
                vertIndices2.Add(cutMesh.Edges[edgeIndex][1]);
            }
            List<int> vertIndices = vertIndices2.ToList();
            //--------------------------------------入力したライン上の節点のインデックスのリストアップ終了-----------------


            /*
            //---------------------------------------可展面近似-------------------------------------------------------
            List<int> boundaryVertIndices = cutMesh.BoundaryVertIndices();
            List<int> fullSet = Enumerable.Range(0, cutMesh.Vertices.Count).ToList();
            List<int> internalVertIndices = fullSet.Except(boundaryVertIndices).ToList();
            int[][][] TriInterFaceIndices = CutMeshCalcTools.MakeInternalFaceID(cutMesh);
            Point3d[] newVertices = cutMesh.Vertices.ToArray();

            double F = 0;
            double[] gaussian = CutMeshCalcTools.GaussianCurvature(cutMesh);
            for (int i = 0; i < gaussian.Length; i++)
            {
                F += gaussian[i];
            }

            int iterations = 0;
            int count = internalVertIndices.Count;
            if (F > 1e-4)
            {
                Vector3d[] Jacobi = new Vector3d[count];
                Jacobi[0] = new Vector3d(1, 0, 0);
                Vector3d[] p = new Vector3d[count];

                while (iterations < c)
                {
                    //newTopoVerticesとjacobiとpを更新していく
                    CurvatureTools.CGCrvNextMesh(newVertices, Jacobi, p, internalVertIndices, TriInterFaceIndices);
                    iterations += 1;
                }
            }
            cutMesh.Vertices = newVertices.ToList();
            //-----------------------------------------------------可展近似終了--------------------------------------------
            */

            //展開図の作成
            CutMesh mesh2d = NetTools.NetBFFandCGLength(cutMesh);


            //-----------------------------------------------------本題のルーリング----------------------------------------
            double width = 0;
            if (!DA.GetData(1, ref width)) return;
            int[] qMax = RulingTools.qMaxVertIndex(mesh2d, cutMesh, vertIndices);
            (List<int[]> OnEdge, List<double[]> Ratio, List<Vector3d[]> Normal) manyRulings = RulingTools.PossibleRulings(mesh2d, cutMesh, qMax);
            
            (List<int[]> OnEdge, List<double[]> Ratio) possibleRuling = RulingTools.RulingChoice(cutMesh, manyRulings.OnEdge, manyRulings.Ratio, manyRulings.Normal, width);
            List<int[]> onEdge = possibleRuling.OnEdge;
            List<double[]> ratio = possibleRuling.Ratio;
            /*
            List<int[]> onEdge = manyRulings.OnEdge;
            List<double[]> ratio = manyRulings.Ratio;
            List<int> test = onEdge.SelectMany(arr => arr).ToList();
            */
            List<Line> lines = new List<Line>();
            for (int i = 0; i < onEdge.Count; i++)
            {
                Point3d a = (1 - ratio[i][0]) * cutMesh.Vertices[cutMesh.Edges[onEdge[i][0]][0]] + ratio[i][0] * cutMesh.Vertices[cutMesh.Edges[onEdge[i][0]][1]];
                Point3d b = (1 - ratio[i][1]) * cutMesh.Vertices[cutMesh.Edges[onEdge[i][1]][0]] + ratio[i][1] * cutMesh.Vertices[cutMesh.Edges[onEdge[i][1]][1]];
                lines.Add(new Line(a, b));
            }
            
            DA.SetDataList(0, lines);
            //DA.SetDataList(0, test);
            //DA.SetDataList(0, qMax.ToList());
        }

        protected override System.Drawing.Bitmap Icon
        {
            get
            {

                return null;
            }
        }

        public override Guid ComponentGuid
        {
            get { return new Guid("9A975ABE-60E3-4FDE-AAF1-277F0B46871A"); }
        }
    }
}