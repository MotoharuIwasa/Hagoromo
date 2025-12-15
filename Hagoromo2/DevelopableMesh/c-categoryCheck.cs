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
    public class categoryCheck : GH_Component
    {
        public categoryCheck()
          : base("categoryCheck", "categoryCheck",
              "categoryCheck",
              "Hagoromo", "Optimization")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated CutMesh", "(C)M", "mesh to develop", GH_ParamAccess.item);
            pManager.AddBooleanParameter("xyMirror", "xy", "xyMirror", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("yzMirror", "yz", "yzMirror", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("zxMirror", "zx", "zxMirror", GH_ParamAccess.item, false);
            pManager.AddCurveParameter("outerBoundaryCurves", "outerB", "outerBoundaryCurves", GH_ParamAccess.list);
            int fixCrvIndex = pManager.AddCurveParameter("fixPointsCrv", "fixCrv", "fixPointsCrv", GH_ParamAccess.list);
            pManager[5].Optional = true;
        }


        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            //pManager.AddMeshParameter("Developed Mesh", "M", "developed planer mesh", GH_ParamAccess.item);
            pManager.AddIntegerParameter("category", "category", "category", GH_ParamAccess.tree);
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
                if (goo.CastTo(out Rhino.Geometry.Mesh m))
                {
                    cutMesh = new CutMesh(m);
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
            bool xyMirror = false;
            DA.GetData(1, ref xyMirror);
            bool yzMirror = false;
            DA.GetData(2, ref yzMirror);
            bool zxMirror = false;
            DA.GetData(3, ref zxMirror);

            List<Curve> outerCrvs = new List<Curve>();
            DA.GetDataList(4, outerCrvs);
            List<Point3d> outerPoints = new List<Point3d>();
            foreach (Curve crv in outerCrvs)
            {
                if (crv.TryGetPolyline(out Polyline polyline))
                {
                    outerPoints.AddRange(polyline);
                }
            }

            List<Curve> fixCrvs = new List<Curve>();
            bool hasInput = DA.GetDataList(5, fixCrvs);
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

            // 以降 cutMesh が確実に利用可能
            List<List<List<int>>> category = CurvatureTools.CategolizeCutMesh(cutMesh, xyMirror, yzMirror, zxMirror, sortedOuterVertIndices, sortedFixVertIndices);
            DataTree<int> tree = new DataTree<int>();

            for (int i = 0; i < category.Count; i++)
            {
                for (int j = 0; j < category[i].Count; j++)
                {
                    GH_Path path = new GH_Path(i, j);
                    foreach (int val in category[i][j])
                    {
                        tree.Add(val, path);
                    }
                }
            }
            DA.SetDataTree(0, tree);
            //DA.SetDataList(0, sortedOuterVertIndices);
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

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("E4AE0FA2-B255-4023-8949-C4B25C43AC6F"); }
        }
    }
}