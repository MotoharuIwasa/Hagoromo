using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using Hagoromo.GeometryTools;

namespace Hagoromo.DevelopableMesh
{
    public class MeshNet3 : GH_Component
    {
        //角度ベースの最適化による展開図。うまくいっていない
        public MeshNet3()
          : base("MeshNet3", "MeshNet3",
              "Description",
              "Hagoromo", "Optimization")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Triangulated Developable Mesh", "M", "mesh to develop", GH_ParamAccess.item);
            pManager.AddCurveParameter("Outline", "L", "Boundary edge as polyline", GH_ParamAccess.item);
            pManager[1].Optional = true;
            pManager.AddIntegerParameter("count", "C", "max iteration", GH_ParamAccess.item);
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
            Curve inputCurve = null;

            bool hasInput = DA.GetData(1, ref inputCurve);

            if (hasInput)
            {
                if (!inputCurve.TryGetPolyline(out outline))
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


            int c = 0;
            if (!DA.GetData(2, ref c)) return;

            double[][][] initialAngle = MeshCalcTools.GetMeshSignAngle(mesh);
            int[][] faceTopoVertIndices = MeshDataTools.FaceTopoVertIndices(mesh);
            double[][] newTopoVertices2D = NetTools.TutteTopoVertices(mesh, outline);

            int count = newTopoVertices2D.Length;
            double[][] Jacobi = new double[count][];
            for (int i = 0; i < count; i++)
            {
                Jacobi[i] = new double[2];
            }
            Jacobi[0][0] = 1;
            double[][] p = new double[count][];
            for (int i = 0; i < count; i++)
            {
                p[i] = new double[2];
            }

            int iterations = 0;
            while (iterations < c)
            {
                //newTopoVerticesとjacobiとpを更新していく
                NetTools.NetCGAngle(Jacobi, p, faceTopoVertIndices, initialAngle, newTopoVertices2D);
                iterations += 1;
            }

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
                //You can add image files to your project resources and access them like this:
                // return Resources.IconForThisComponent;
                return null;
            }
        }

        public override Guid ComponentGuid
        {
            get { return new Guid("33A5B0BA-172D-4B30-AFB2-F165440FEDE4"); }
        }
    }
}