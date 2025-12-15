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
    //DevelopMeshよりも性能がいい
    public class DevelopMesh2 : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public DevelopMesh2()
          : base("Develop Mesh2", "Develop2",
              "Develop Mesh2",
              "Hagoromo", "Optimization")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
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
            for (int i = 0;  i < internalVertexIndices.Count; i++)
            {
                TriInterFaceIndices[i] = TriFaceIndices[internalVertexIndices[i]];
            }

            double F = 0;
            for (int i = 0; i < internalVertexIndices.Count; i++)
            {
                F += CurvatureTools.CurvatureTwo(mesh, internalVertexIndices[i]);
            }

            int iterations = 0;
            if (F > 1e-4)
            {
                while (iterations < c)
                {
                    newTopoVertices = CurvatureTools.SDnewTopoV(newTopoVertices, internalVertexIndices, TriInterFaceIndices);
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
            get { return new Guid("F7DFF70B-A684-4BA2-9206-3C9C2139BC0A"); }
        }
    }
}