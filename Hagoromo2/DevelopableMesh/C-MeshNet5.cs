using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using Hagoromo.GeometryTools;
using System.Linq;

namespace Hagoromo.DevelopableMesh
{
    public class MeshNet5 : GH_Component
    {
        public MeshNet5()
          : base("MeshNet5", "MeshNet5",
              "NeshNet5",
              "Hagoromo", "Optimization")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Triangulated Mesh or CutMesh", "(C)M", "Mesh or CutMesh", GH_ParamAccess.item);
            pManager.AddCurveParameter("Outline", "L", "Boundary edge as polyline", GH_ParamAccess.item);
            pManager[1].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("cutMesh", "CM", "CM", GH_ParamAccess.item);
            pManager.AddGenericParameter("cutMesh", "CM", "CM", GH_ParamAccess.item);
            //pManager.AddPointParameter("boundary", "b", "b", GH_ParamAccess.list);
            //pManager.AddIntegerParameter("boundary", "be", "b", GH_ParamAccess.list);
            //pManager.AddNumberParameter("boundary", "be", "b", GH_ParamAccess.list);
            //pManager.AddNumberParameter("boundary", "be", "b", GH_ParamAccess.list);
            //pManager.AddTextParameter("MatrixText", "MT", "Matrix as string", GH_ParamAccess.item);
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
            int[][] newConnectedFaces = MeshCutTools.MeshToOpenTree(newMesh);
            List<int> cutEdgeIndices = MeshCutTools.CutEdgeIndices(newMesh, newConnectedFaces);

            CutMesh cutMesh = MeshCutTools.CutMeshWithEdgeIndices(newMesh, cutEdgeIndices);
            Point3d[] vertices = new Point3d[cutMesh.Vertices.Count];
            for (int i = 0; i < cutMesh.DuplicatedVertIndices.Count; i++)
            {
                for (int j = 0; j < cutMesh.DuplicatedVertIndices[i].Count; j++)
                {
                    vertices[cutMesh.DuplicatedVertIndices[i][j]] = mesh.Vertices[i];
                }
            }
            List<Point3d> verticesList = vertices.ToList();
            cutMesh.Vertices = verticesList;


            GH_CutMesh ghCutMesh = new GH_CutMesh(cutMesh);
            //DA.SetData(0, ghCutMesh);

            CutMesh devMesh = NetTools.NetBFF(cutMesh);
            GH_CutMesh ghDevMesh = new GH_CutMesh(devMesh);
            DA.SetData(1, ghDevMesh);
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
            get { return new Guid("E64ADB99-FEFE-4452-9238-1C4B23387DDA"); }
        }
    }
}