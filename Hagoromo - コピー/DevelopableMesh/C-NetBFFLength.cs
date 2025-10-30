using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Hagoromo.GeometryTools;
using Rhino;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Hagoromo.DevelopableMesh
{
    public class NetBFFLength : GH_Component
    {
        public NetBFFLength()
          : base("NetBFFLength", "NetBFFLength",
              "BFF and Length minimization",
              "Hagoromo", "Optimization")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "Mesh or CutMesh", GH_ParamAccess.item);
            pManager.AddCurveParameter("Feature Lines (Polylines)", "L", "Lines", GH_ParamAccess.list);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("cutMesh", "CM", "CM", GH_ParamAccess.item);
            pManager.AddCurveParameter("Feature Lines (Polylines)", "L", "Lines", GH_ParamAccess.list);
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
                CutMesh cutMesh = new CutMesh();
                // Mesh へのキャストを試す
                if (goo.CastTo(out Rhino.Geometry.Mesh m))
                {
                    cutMesh = new CutMesh(m);
                    cutMesh = cutMesh.Sort();
                }
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
                CutMesh newMesh = NetTools.NetBFFandCGLength(cutMesh);
                GH_CutMesh ghCutMesh = new GH_CutMesh(newMesh);
                DA.SetData(0, ghCutMesh);
                List<Line> lines = new List<Line>();
                foreach (int edgeIndex in edgeIndices)
                {
                    lines.Add(newMesh.GetEdgeLine(edgeIndex));
                }
                DA.SetData(0, new GH_CutMesh(newMesh));
                DA.SetDataList(1, lines);
            }
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
        public override Guid ComponentGuid
        {
            get { return new Guid("BA43CBDA-76CF-48DE-AED1-D4AE397AB62D"); }
        }
    }
}