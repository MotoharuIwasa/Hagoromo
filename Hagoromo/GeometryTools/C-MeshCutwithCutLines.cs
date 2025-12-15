using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Hagoromo.GeometryTools
{
    public class MeshCutWithCutLines : GH_Component
    {
        public MeshCutWithCutLines()
          : base("MeshCut with CutLines", "MeshCutbyLines",
              "Cut (Cut)Mesh with CutLines",
              "Hagoromo", "Geometry")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "CutMesh or Mesh", GH_ParamAccess.item);
            pManager.AddCurveParameter("Open Cut Lines (Polylines)", "L", "CutLines", GH_ParamAccess.list);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("TriangulatedCutMesh", "CM", "CutMesh", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Curve> curves = new List<Curve>();
            if (!DA.GetDataList(1, curves)) return;

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

            object input = null;
            if (!DA.GetData(0, ref input)) return;

            if (input is IGH_Goo goo)
            {
                // Mesh へのキャストを試す
                if (goo.CastTo(out Mesh mesh))
                {
                    // RTree 構築
                    RTree rtree = new RTree();
                    for (int i = 0; i < mesh.TopologyVertices.Count; i++)
                    {
                        rtree.Insert(mesh.TopologyVertices[i], i);
                    }

                    List<int> edgeIndices = new List<int>();
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
                                double d = pt.DistanceTo(mesh.TopologyVertices[args.Id]);
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
                            int vA = topoVertsOnPolyline[i];
                            int vB = topoVertsOnPolyline[i + 1];

                            // vA に接続するエッジの中から vB を探す
                            int[] connectedEdges = mesh.TopologyVertices.ConnectedEdges(vA);
                            foreach (int ei in connectedEdges)
                            {
                                IndexPair tv = mesh.TopologyEdges.GetTopologyVertices(ei);
                                if ((tv.I == vA && tv.J == vB) || (tv.I == vB && tv.J == vA))
                                {
                                    if (!edgeIndices.Contains(ei))
                                        edgeIndices.Add(ei);
                                    break;
                                }
                            }
                        }
                    }

                    CutMesh newMesh = MeshCutTools.CutMeshWithEdgeIndices(mesh, edgeIndices);
                    DA.SetData(0, new GH_CutMesh(newMesh));
                }

                // CutMesh へのキャストを試す
                else if (goo.CastTo(out CutMesh cutMesh2))
                {
                    CutMesh cutMesh = cutMesh2.Clone();
                    // RTree 構築
                    RTree rtree = new RTree();
                    for (int i = 0; i < cutMesh.Vertices.Count; i++)
                    {
                        rtree.Insert(cutMesh.Vertices[i], i);
                    }

                    List<int> edgeIndices = new List<int>();
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

                    CutMesh newMesh = MeshCutTools.CutMeshWithEdgeIndices(cutMesh, edgeIndices);
                    DA.SetData(0, new GH_CutMesh(newMesh));
                }
            }
        }

        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                //You can add image files to your project resources and access them like this:
                // return Resources.IconForThisComponent;
                return null;
            }
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("F53ADF39-2D5B-466F-97A2-C7564EFB361E"); }
        }
    }
}