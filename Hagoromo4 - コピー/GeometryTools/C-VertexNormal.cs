using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Hagoromo.MathTools;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Collections;
using System;
using System.Collections.Generic;
using System.Linq;
using static Hagoromo.GeometryTools.CutMeshCalcTools;

namespace Hagoromo.GeometryTools
{
    public class VertexNormal : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public VertexNormal()
          : base("VertexNormal", "VertexNormal",
              "Vertex Normal",
              "Hagoromo", "geometry")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "CutMesh or Mesh", GH_ParamAccess.item);
            pManager.AddNumberParameter("scale", "S", "scale", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Normals", "n", "vertex normals", GH_ParamAccess.list);
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
            double scale = 0;
            if (!DA.GetData(1, ref scale)) return;
            List<Line> lines = new List<Line>();
            Vector3d[] normal = VertexNormal(cutMesh);
            for (int i = 0; i < normal.Length; i++)
            {
                Point3d center = cutMesh.Vertices[i];
                Line line = new Line(center, center + normal[i] * scale);
                lines.Add(line);
            }
            DA.SetDataList(0, lines);
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

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("4478C672-C8F9-44DB-806B-323D200FFB97"); }
        }
    }
}