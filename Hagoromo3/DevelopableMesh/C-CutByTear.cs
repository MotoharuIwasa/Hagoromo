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
using static Hagoromo.GeometryTools.MeshCutTools;
using static Hagoromo.DevelopableMesh.DevelopableTools;
using static Hagoromo.DevelopableMesh.CutChoiceTools;
using static Hagoromo.DevelopableMesh.NetTools;
using static Hagoromo.DevelopableMesh.CGNR3D;


namespace Hagoromo.DevelopableMesh
{
    public class CutByTear : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public CutByTear()
          : base("CutByTear", "CutByTear",
              "CutByTear",
              "Hagoromo", "Optimization")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated 3d CutMesh", "(C)M", "mesh to develop", GH_ParamAccess.item);
            pManager.AddIntegerParameter("iterations", "iter", "iterations", GH_ParamAccess.item);
            pManager.AddNumberParameter("w0", "w0", "length preserve weight in each iteration", GH_ParamAccess.list);
            pManager.AddNumberParameter("w1", "w1", "face flip preserve weight in each iteration", GH_ParamAccess.list);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated 3d CutMesh", "(C)M3D", "3次元メッシュにおける切込みの履歴", GH_ParamAccess.list);
            pManager.AddGenericParameter("Triangulated 2d CutMesh", "(C)M2D", "展開図における切込みの履歴", GH_ParamAccess.list);
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

            int iter = 0;
            DA.GetData(1, ref iter);
            List<double> w0List = new List<double>();
            DA.GetDataList(2, w0List);
            List<double> w1List = new List<double>();
            DA.GetDataList(3, w1List);

            double w0 = w0List[0];
            double w1 = w1List[0];
            mesh3d = mesh3d.Sort();
            List<int> sortedOuterVertIndices = mesh3d.BoundaryVertIndices();
            List<GH_CutMesh> mesh3dHistory = new List<GH_CutMesh> { new GH_CutMesh(mesh3d.Clone()) };
            List<GH_CutMesh> mesh2dHistory = new List<GH_CutMesh>();
            for (int i = 0; i < iter; i++)
            {
                if (w0List.Count > i) { w0 = w0List[i]; }
                if (w1List.Count > i) { w1 = w1List[i]; }
                //DevelopCGLS(mesh3d, sortedOuterVertIndices,10,100);
                CutMesh mesh2d = CutMeshNetLBFGS2(mesh3d, w0, w1);
                
                mesh2dHistory.Add(new GH_CutMesh(mesh2d.Clone()));
                List<int> cutEdges = FindNextCutEdge(mesh3d, mesh2d);
                mesh3d = CutMeshWithEdgeIndices(mesh3d, cutEdges).Sort();
                mesh3dHistory.Add(new GH_CutMesh(mesh3d.Clone()));
            }
            //DevelopCGLS(mesh3d, sortedOuterVertIndices, 200, 100);
            //mesh3dHistory.Add(new GH_CutMesh(mesh3d.Clone()));

            DA.SetDataList(0, mesh3dHistory);
            DA.SetDataList(1, mesh2dHistory);
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
            get { return new Guid("00672A22-08B9-4071-B1FA-644FE3B1A059"); }
        }
    }
}