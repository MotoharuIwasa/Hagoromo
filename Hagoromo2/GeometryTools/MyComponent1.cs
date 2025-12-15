using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using Rhino.Render.ChangeQueue;
using System;
using System.Collections.Generic;
using System.Linq;
using Hagoromo.DevelopableMesh;
using Hagoromo.MathTools;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra.Double.Solvers;
using MathNet.Numerics.LinearAlgebra.Factorization;


namespace Hagoromo.GeometryTools
{
    public class Test : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public Test()
          : base("MyComponent1", "Nickname",
              "Description",
              "Hagoromo", "Optimization")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "Mesh or CutMesh", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("cutMesh", "CM", "CM", GH_ParamAccess.item);
            //pManager.AddPointParameter("boundary", "b", "b", GH_ParamAccess.list);
            //pManager.AddIntegerParameter("boundary", "be", "b", GH_ParamAccess.list);
            pManager.AddNumberParameter("gaussian", "g", "g", GH_ParamAccess.list);
            pManager.AddNumberParameter("k", "k", "k", GH_ParamAccess.list);
            pManager.AddNumberParameter("b", "b", "b", GH_ParamAccess.list);
            pManager.AddNumberParameter("a", "a", "a", GH_ParamAccess.list);
            pManager.AddTextParameter("MatrixText", "MT", "Matrix as string", GH_ParamAccess.item);
            //pManager.AddTextParameter("MatrixText", "MT", "Matrix as string", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            object input = null;
            if (!DA.GetData(0, ref input)) return;

            CutMesh mesh = null;

            if (input is IGH_Goo goo)
            {
                // Mesh へのキャストを試す
                if (goo.CastTo(out Rhino.Geometry.Mesh m))
                {
                    mesh = new CutMesh(m);
                }
                // CutMesh へのキャストを試す
                else if (goo.CastTo(out CutMesh cm))
                {
                    mesh = cm;
                }
            }

            
            CutMesh cutMesh = mesh.Sort();
            GH_CutMesh gH_CutMesh = new GH_CutMesh(cutMesh);
            DA.SetData(0, gH_CutMesh);
            int boundaryVertCount = cutMesh.BoundaryVertIndices().Count;
            double[,] A = NetTools.BuildLaplace(cutMesh);

            string matrixStr = "";
            int rows = A.GetLength(0);
            int cols = A.GetLength(1);

            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < cols; j++)
                {
                    matrixStr += A[i, j].ToString("F8").PadLeft(15); // 右寄せで10桁分
                }
                matrixStr += "\n";
            }

            DA.SetData(5, matrixStr);


            double[] kdiff = new double[boundaryVertCount];
            double[] gaussian = CutMeshCalcTools.GaussianCurvature(cutMesh);
            double[] gaussian2 = new double[cutMesh.Vertices.Count];
            DA.SetDataList(1, gaussian);
            double[] k = CutMeshCalcTools.GeodesicCurvature(cutMesh);
            DA.SetDataList(2, k);



            //double[] u = NetTools.NeumannToDirichlet(A, MatrixUtils.Multiply(-1, gaussian), kdiff, boundaryVertCount);

            double[] fai = MatrixUtils.Multiply(-1, gaussian);
            double[] h = kdiff;
            double[] h2 = new double[fai.Length];
            for (int i = 0; i < boundaryVertCount; i++)
            {
                h2[i] = h[i];
            }
            double[] b = MatrixUtils.Subtract(fai, h2);
            double[,] Apart = MatrixUtils.ExtractPartMatrix(A, 1, 1, A.GetLength(0) - 1, A.GetLength(1) - 1);
            double[] bpart = new double[b.Length - 1];
            for (int i = 0; i < b.Length - 1; i++)
            {
                bpart[i] = b[i + 1];
            }
            
            var Amat = DenseMatrix.OfArray(Apart);
            var bmat = Vector.Build.DenseOfArray(bpart);
            double[] apart = Amat.Solve(bmat).ToArray();
            double[] a = new double[apart.Length+1];
            for (int i = 1; i < a.Length; i++)
            {
                a[i] = apart[i - 1];
            }
            //double[] a = SLEsSolvers.SolveByLDL(Atri, b);
            //a[0] = 0;
            double[] u = a.Take(boundaryVertCount).ToArray();
            DA.SetDataList(3, b);
            DA.SetDataList(4, a);


            int boundaryEdgeCount = boundaryVertCount;
            double[] l = new double[boundaryEdgeCount];
            double[] lnew = new double[boundaryEdgeCount];

            for (int i = 0; i < boundaryEdgeCount; i++)
            {
                Vector3d vector = cutMesh.Vertices[i] - cutMesh.Vertices[(i + 1) % boundaryEdgeCount];
                double length = vector.Length;
                l[i] = length;
                lnew[i] = l[i] * Math.Exp((u[i] + u[(i + 1) % boundaryVertCount]) * 0.5);
            }
            //DA.SetDataList(4, lnew);

            //double[][] gamma = NetTools.NetBFFCheck(mesh);
            //Point3d[] p = PtCrvTools.Convert2Dto3D(gamma);
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
            get { return new Guid("C3B5B5AD-366F-428A-B529-7685477CB280"); }
        }
    }
}