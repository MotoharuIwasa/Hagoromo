using Grasshopper.Kernel;
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
    public class DevelopCutMesh3 : GH_Component
    {
        public DevelopCutMesh3()
          : base("Develop CutMesh3", "DevelopCM3",
              "Develop CutMesh3",
              "Hagoromo", "Optimization")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated CutMesh", "(C)M", "mesh to develop", GH_ParamAccess.item);
            pManager.AddIntegerParameter("iterations", "i", "iteration", GH_ParamAccess.item);
            pManager.AddBooleanParameter("xyMirror", "xy", "xyMirror", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("yzMirror", "yz", "yzMirror", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("zxMirror", "zx", "zxMirror", GH_ParamAccess.item, false);
            pManager.AddCurveParameter("outerBoundaryCurves", "outerB", "outerBoundaryCurves", GH_ParamAccess.list);
            int fixCrvIndex = pManager.AddCurveParameter("fixPointsCrv", "fixCrv", "fixPointsCrv", GH_ParamAccess.list);
            pManager[6].Optional = true;
            pManager.AddNumberParameter("pace", "alpha", "pace", GH_ParamAccess.item, 1);
        }


        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            //pManager.AddMeshParameter("Developed Mesh", "M", "developed planer mesh", GH_ParamAccess.item);
            pManager.AddGenericParameter("Modified CutMesh", "(C)M", "modified CutMesh", GH_ParamAccess.item);
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
            int iterations = 0;
            DA.GetData(1, ref iterations);
            bool xyMirror = false;
            DA.GetData(2, ref xyMirror);
            bool yzMirror = false;
            DA.GetData(3, ref yzMirror);
            bool zxMirror = false;
            DA.GetData(4, ref zxMirror);
            double alpha = 0;
            DA.GetData(7, ref alpha);

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

            // 以降 cutMesh が確実に利用可能
            CutMesh newMesh = CurvatureTools.CGDevCutMeshConsiderJustSelf(cutMesh, iterations, alpha, xyMirror, yzMirror, zxMirror, sortedOuterVertIndices, sortedFixVertIndices);
            DA.SetData(0, new GH_CutMesh(newMesh));
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
            get { return new Guid("D9DFC020-0CAC-4BD8-B32A-3C3F72A25815"); }
        }
    }
}