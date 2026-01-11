using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Hagoromo.GeometryTools;
using Hagoromo.MathTools;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Collections;
using System;
using System.Collections.Generic;
using System.Linq;
using static Hagoromo.GeometryTools.CutMeshCalcTools;
using static Hagoromo.DevelopableMesh.DevelopableTools;
using static Hagoromo.DevelopableMesh.CutChoiceTools;
using static Hagoromo.DevelopableMesh.CGNR3D;


namespace Hagoromo.DevelopableMesh
{
    public class EdgePushPull : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public EdgePushPull()
          : base("EdgePushPull", "EdgePushPull",
              "EdgePushPull",
              "Hagoromo", "Optimization")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated 3d CutMesh", "(C)M", "mesh to develop", GH_ParamAccess.item);
            pManager.AddGenericParameter("Triangulated 2d CutMesh", "(C)M", "mesh to develop", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("vector start points", "P", "vector start points", GH_ParamAccess.list);
            pManager.AddBooleanParameter("push pull colour flag", "C", "if edges are pulled, retrun true", GH_ParamAccess.list);
            pManager.AddGenericParameter("push pull vector", "V", "push pull vector", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            object input = null;
            if (!DA.GetData(0, ref input)) return;

            CutMesh mesh3d = null;

            if (input is IGH_Goo goo)
            {
                // Mesh へのキャストを試す
                if (goo.CastTo(out Mesh m))
                {
                    mesh3d = new CutMesh(m);
                }
                // CutMesh へのキャストを試す
                else if (goo.CastTo(out CutMesh cm))
                {
                    mesh3d = cm.Clone();
                }
            }

            if (mesh3d == null)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Mesh または CutMesh を入力してください");
                return;
            }


            object input2 = null;
            if (!DA.GetData(1, ref input2)) return;

            CutMesh mesh2d = null;

            if (input2 is IGH_Goo goo2)
            {
                // Mesh へのキャストを試す
                if (goo2.CastTo(out Mesh m2))
                {
                    mesh2d = new CutMesh(m2);
                }
                // CutMesh へのキャストを試す
                else if (goo2.CastTo(out CutMesh cm2))
                {
                    mesh2d = cm2.Clone();
                }
            }

            if (mesh2d == null)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Mesh または CutMesh を入力してください");
                return;
            }

            List<List<double>> initialHeight = GetInitialHeight(mesh3d);
            var edgeVecData = EdgeVecList(initialHeight, mesh2d);
            List<List<bool>> positive = edgeVecData.positive;
            List<List<Vector3d>> edgeVec = edgeVecData.edgeVec;
            List<bool> positiveFlat = positive.SelectMany(x => x).ToList();
            List<Vector3d> edgeVecFlat = edgeVec.SelectMany(x => x).ToList();
            DA.SetDataList(0, edgeVecData.vecStart);
            DA.SetDataList(1, positiveFlat);
            DA.SetDataList(2, edgeVecFlat);
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

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("C10C49BC-E392-4945-B1D5-96BFBD07A2F2"); }
        }
    }
}