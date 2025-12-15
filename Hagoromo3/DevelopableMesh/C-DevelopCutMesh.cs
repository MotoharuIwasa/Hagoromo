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
using static Hagoromo.DevelopableMesh.DevelopableTools;
using static Hagoromo.GeometryTools.MeshCalcTools;

namespace Hagoromo.DevelopableMesh
{
    public class DevelopCutMesh : GH_Component
    {
        public DevelopCutMesh()
          : base("Develop CutMesh", "DevelopCM",
              "切込み非対応。CGによる可展面近似",
              "Hagoromo", "Optimization")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated CutMesh", "(C)M", "mesh to develop", GH_ParamAccess.item);
            pManager.AddIntegerParameter("count", "C", "max iteration", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            //pManager.AddMeshParameter("Developed Mesh", "M", "developed planer mesh", GH_ParamAccess.item);
            pManager.AddGenericParameter("Modified CutMesh", "(C)M", "modified CutMesh", GH_ParamAccess.item);
            //pManager.AddPointParameter("a", "a", "a", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            int c = 0;
            if (!DA.GetData(1, ref c)) return;
            object input = null;
            if (!DA.GetData(0, ref input)) return;

            if (input is IGH_Goo goo)
            {
                // Mesh へのキャストを試す
                if (goo.CastTo(out Rhino.Geometry.Mesh mesh))
                {
                    Point3d[] newTopoVertices = MeshDataTools.DoubleTopoVertices(mesh);
                    List<int> internalVertexIndices = MeshDataTools.TopoInternalVertIndices(mesh);
                    int[][][] TriFaceIndices = MeshDataTools.TriFaceIndices(mesh);
                    int[][][] TriInterFaceIndices = new int[internalVertexIndices.Count][][];
                    for (int i = 0; i < internalVertexIndices.Count; i++)
                    {
                        TriInterFaceIndices[i] = TriFaceIndices[internalVertexIndices[i]];
                    }

                    double F = 0;
                    for (int i = 0; i < internalVertexIndices.Count; i++)
                    {
                        F += CurvatureTwo(mesh, internalVertexIndices[i]);
                    }

                    int iterations = 0;
                    int count = internalVertexIndices.Count;
                    if (F > 1e-4)
                    {
                        Vector3d[] Jacobi = new Vector3d[count];
                        Jacobi[0] = new Vector3d(1, 0, 0);
                        Vector3d[] p = new Vector3d[count];

                        while (iterations < c)
                        {
                            //newTopoVerticesとjacobiとpを更新していく
                            CGCrvNextMesh(newTopoVertices, Jacobi, p, internalVertexIndices, TriInterFaceIndices);
                            iterations += 1;
                        }
                    }

                    //更新後のメッシュを作成
                    Rhino.Geometry.Mesh newMesh = MeshDataTools.MakeMesh(mesh, newTopoVertices);
                    DA.SetData(0, newMesh);
                }

                // CutMesh へのキャストを試す
                else if (goo.CastTo(out CutMesh cutMesh))
                {
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
                            CGCrvNextMesh(newVertices, Jacobi, p, internalVertIndices, TriInterFaceIndices);
                            iterations += 1;
                        }
                    }
                    cutMesh.Vertices = newVertices.ToList();
                    DA.SetData(0, new GH_CutMesh(cutMesh));


                }
            }

            /*
            if (cutMesh == null)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Mesh または CutMesh を入力してください");
                return;
            }
            */

            //DA.SetDataList(1, newTopoVertices);
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
            get { return new Guid("CE617720-6BCB-4A96-B3C4-93DFCEC83C62"); }
        }
    }
}