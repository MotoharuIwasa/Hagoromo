using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Hagoromo.GeometryTools;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Hagoromo.DevelopableMesh
{
    public class NetCGNR : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public NetCGNR()
          : base("NetCGNR", "NetCGNR",
              "NetCGNR",
              "Hagoromo", "Optimization")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "Mesh or CutMesh", GH_ParamAccess.item);
            pManager.AddCurveParameter("Feature Lines (Polylines)", "L", "Lines", GH_ParamAccess.list);
            pManager[1].Optional = true;
            pManager.AddIntegerParameter("iterations", "iter", "iterations", GH_ParamAccess.item);
            pManager.AddNumberParameter("w0", "w0", "length preservation weight", GH_ParamAccess.item);
            pManager.AddNumberParameter("w1", "w1", "flatten 3Dmesh to 2D weight", GH_ParamAccess.item);
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
            int iteration = 0;
            DA.GetData(2, ref iteration);
            double[] w = new double[2];
            DA.GetData(3, ref w[0]);
            DA.GetData(4, ref w[1]);

            object input = null;
            CutMesh cutMesh = new CutMesh();
            if (!DA.GetData(0, ref input)) return;
            if (!(input is IGH_Goo goo2)) return;
            if (input is IGH_Goo goo)
            {
                // Mesh へのキャストを試す
                if (goo.CastTo(out Rhino.Geometry.Mesh m))
                {
                    cutMesh = new CutMesh(m);
                    cutMesh = cutMesh.Sort();
                }
                else if (goo.CastTo(out CutMesh mesh))
                {
                    cutMesh = mesh.Sort();
                }
                else { return; }
            }

            List<Curve> curves = new List<Curve>();
            DA.GetDataList(1, curves);


            if (curves.Count == 0)
            {
                CutMesh newMesh2 = NetTools.NetCGNR(cutMesh, iteration, w);
                DA.SetData(0, new GH_CutMesh(newMesh2));
                DA.SetDataList(1, new List<Line>()); // 出力 1 も空リスト
                return;
            }

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
            CutMesh newMesh = NetTools.NetCGNR(cutMesh,iteration,w);
            GH_CutMesh ghCutMesh = new GH_CutMesh(newMesh);
            List<Line> lines = new List<Line>();
            foreach (int edgeIndex in edgeIndices)
            {
                lines.Add(newMesh.GetEdgeLine(edgeIndex));
            }
            DA.SetData(0, new GH_CutMesh(newMesh));
            DA.SetDataList(1, lines);
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
            get { return new Guid("B19525BE-5F3F-4526-A249-DE649BBAE8EF"); }
        }
    }
}