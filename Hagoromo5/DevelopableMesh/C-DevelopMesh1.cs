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
    public class DevelopMesh1 : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public DevelopMesh1()
          : base("Develop Mesh1", "Develop1",
              "Develop Mesh1",
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
            pManager.AddIntegerParameter("a", "a", "a", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {

            Rhino.Geometry.Mesh mesh = null;
            if (!DA.GetData(0, ref mesh)) return;
            int c = 0;
            if (!DA.GetData(1, ref c)) return;

            List<int> internalVertexIndices = MeshDataTools.TopoInternalVertIndices(mesh);
            int count = 0;
            double F = 0;
            Rhino.Geometry.Mesh optMesh = mesh.DuplicateMesh();

            double f = 0;
            for (int i = 0; i < internalVertexIndices.Count; i++)
            {
                double curvatureTwo = CurvatureTools.CurvatureTwo(optMesh, internalVertexIndices[i]);
                f += curvatureTwo;
            }
            F = f;

            if (F > 1e-4)
            {
                while (count < c && F > 1e-4)
                {
                    optMesh = CurvatureTools.SDCrvNextMesh2(optMesh);
                    //ガウス曲率の2乗の和を計算
                    f = 0;
                    for (int i = 0; i < internalVertexIndices.Count; i++)
                    {
                        double curvatureTwo = CurvatureTools.CurvatureTwo(optMesh, internalVertexIndices[i]);
                        f += curvatureTwo;
                    }
                    F = f;
                    count += 1;
                }
            }
            DA.SetData(0, optMesh);
            DA.SetData(1, count);
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
            get { return new Guid("822F3EF4-7A21-4B2D-A1B8-8995E982A3D2"); }
        }
    }
}
