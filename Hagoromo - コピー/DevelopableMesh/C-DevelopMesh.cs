using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino.Geometry.Collections;
using Rhino.Render.ChangeQueue;
using System;
using System.Collections.Generic;
using Rhino;
using Hagoromo.MathTools;
using Hagoromo.GeometryTools;
using System.Linq;

namespace Hagoromo.DevelopableMesh
{
    //共役勾配法によるDevelopMesh
    public class DevelopMesh : GH_Component
    {
        public DevelopMesh()
          : base("Develop Mesh", "Develop",
              "Develop Mesh",
              "Hagoromo", "Optimization")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Triangulated Mesh", "M", "mesh to develop", GH_ParamAccess.item);
            pManager.AddIntegerParameter("count", "C", "max iteration", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            //pManager.AddMeshParameter("Developed Mesh", "M", "developed planer mesh", GH_ParamAccess.item);
            pManager.AddMeshParameter("Modified Mesh", "M", "modified mesh", GH_ParamAccess.item);
            pManager.AddPointParameter("a", "a", "a", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {

            Rhino.Geometry.Mesh mesh = null;
            if (!DA.GetData(0, ref mesh)) return;
            int c = 0;
            if (!DA.GetData(1, ref c)) return;

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
                F += CurvatureTools.CurvatureTwo(mesh, internalVertexIndices[i]);
            }

            int iterations = 0;
            int count = internalVertexIndices.Count;
            if (F > 1e-4)
            {
                Vector3d[] Jacobi = new Vector3d[count];
                Jacobi[0] = new Vector3d(1,0,0);
                Vector3d[] p = new Vector3d[count];

                while (iterations < c)
                {
                    //newTopoVerticesとjacobiとpを更新していく
                    CurvatureTools.CGCrvNextMesh(newTopoVertices, Jacobi, p, internalVertexIndices, TriInterFaceIndices);
                    iterations += 1;
                }
            }

            //更新後のメッシュを作成
            Rhino.Geometry.Mesh newMesh = MeshDataTools.MakeMesh(mesh, newTopoVertices);

            DA.SetData(0, newMesh);
            DA.SetDataList(1, newTopoVertices);
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
            get { return new Guid("DCE322E7-9656-4B28-96BF-2E96C85F5A78"); }
        }
    }
}