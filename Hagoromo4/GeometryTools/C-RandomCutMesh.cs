using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;

using Rhino.Geometry;
using System;
using System.Collections.Generic;

using Grasshopper.Kernel.Types;

namespace Hagoromo.GeometryTools
{
    public class RondomCutMesh : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent2 class.
        /// </summary>
        public RondomCutMesh()
          : base("RondomCutMesh", "RondomCutMesh",
              "RondomCutMesh",
              "Hagoromo", "Geometry")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "Mesh or CutMesh", GH_ParamAccess.item);
            pManager.AddNumberParameter("Difference", "D", "difference", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Count", "C", "generating count", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "Mesh or CutMesh", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            object input = null;
            if (!DA.GetData(0, ref input)) return;

            CutMesh cutMesh = null;

            if (input is IGH_Goo goo)
            {
                // Mesh へのキャストを試す
                if (goo.CastTo(out Mesh m))
                {
                    cutMesh = new CutMesh(m);
                }
                // CutMesh へのキャストを試す
                else if (goo.CastTo(out CutMesh cm))
                {
                    cutMesh = cm.Clone();
                }
            }

            if (cutMesh == null)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Mesh または CutMesh を入力してください");
                return;
            }

            // 以降 cutMesh が確実に利用可能

            double strength = 0;
            DA.GetData(1, ref strength);
            int count = 0;
            DA.GetData(2, ref count);

            List<GH_CutMesh> cutMeshList = new List<GH_CutMesh>();
            cutMeshList.Add(new GH_CutMesh(cutMesh));
            List<Point3d> vertices = cutMesh.Vertices;
            Random rnd = new Random();

            for (int i = 0; i < count - 1; i++)
            {
                List<Point3d> newVertices = new List<Point3d> (cutMesh.Vertices);
                for(int j =0; j <  cutMesh.DuplicatedVertIndices.Count; j++)
                {
                    // 0.0 ～ 1.0 の乱数を生成し、それを -strength ～ +strength の範囲に変換する
                    // 式: (乱数 - 0.5) * 2 * 強さ

                    double xNoise = (rnd.NextDouble() - 0.5) * 2 * strength;
                    double yNoise = (rnd.NextDouble() - 0.5) * 2 * strength;
                    double zNoise = (rnd.NextDouble() - 0.5) * 2 * strength;

                    foreach (int v in cutMesh.DuplicatedVertIndices[j])
                    {
                        newVertices[v] += new Point3d(xNoise, yNoise, zNoise);
                    }
                }
                CutMesh newCutMesh = cutMesh.Clone();
                newCutMesh.Vertices = newVertices;
                cutMeshList.Add(new GH_CutMesh(newCutMesh));
            }
            DA.SetDataList(0, cutMeshList);
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
            get { return new Guid("C234CD7F-17FB-49A9-99CC-AA165FD51EAD"); }
        }
    }
}