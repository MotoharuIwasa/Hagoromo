using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino.Input.Custom;
using Rhino.Render.ChangeQueue;
using System;
using System.Collections.Generic;
using System.Security.Cryptography;
using Hagoromo.GeometryTools;

namespace Hagoromo.DevelopableMesh
{
    public class TutteMesh : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public TutteMesh()
          : base("TutteMesh", "TutteMesh",
              "Description",
              "Hagoromo", "Optimization")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Triangulated Mesh", "M", "mesh to develop", GH_ParamAccess.item);
            pManager.AddCurveParameter("Outline", "L", "Boundary edge as polyline", GH_ParamAccess.item);
            pManager[1].Optional = true;

        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Tutte Mesh", "M", "Tutte mesh", GH_ParamAccess.item);
            pManager.AddPointParameter("point", "p", "tuttepoints", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Rhino.Geometry.Mesh mesh = null;
            if (!DA.GetData(0, ref mesh)) return;
            Polyline outline = null;
            Curve nakedEdge = null;

            bool hasInput = DA.GetData(1, ref nakedEdge);

            if (hasInput)
            {
                if (!nakedEdge.TryGetPolyline(out outline))
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Input must be a valid polyline.");
                    return;
                }
            }
            else
            {
                Polyline[] nakedEdges = mesh.GetNakedEdges();
                if (nakedEdges == null || nakedEdges.Length == 0)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Mesh has no naked edges.");
                    return;
                }
                outline = nakedEdges[0];
            }

            double[][] newTopoVertices2D = NetTools.TutteTopoVertices(mesh, outline);
            Point3d[] newTopoVertices = PtCrvTools.Convert2Dto3D(newTopoVertices2D);

            //更新後のメッシュを作成
            Rhino.Geometry.Mesh newMesh = MeshDataTools.MakeMesh(mesh, newTopoVertices);

            DA.SetData(0, newMesh);
            DA.SetDataList(1, newTopoVertices);
        }

        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return new System.Drawing.Bitmap(@"C:\MasterThesis\figures\result01.png");
            }
        }

        public override Guid ComponentGuid
        {
            get { return new Guid("A3104764-3643-4839-96AC-9E06B0E5A212"); }
        }
    }
}