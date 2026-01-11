using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Hagoromo.GeometryTools;
using Hagoromo.MathTools;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using static Hagoromo.GeometryTools.CutMeshCalcTools;
namespace Hagoromo.DevelopableMesh
{
    public class SignedArea : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public SignedArea()
          : base("SignedArea", "SignedArea",
              "SignedArea",
              "Hagoromo", "Optimization")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "CutMesh or Mesh", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("area", "A", "signed area", GH_ParamAccess.list);
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


            int[,] facesWithOrder = FacesWithOrder(cutMesh);
            List<double> area = new List<double>();
            for (int i = 0; i < facesWithOrder.GetLength(0);i++)
            {
                int v0 = facesWithOrder[i, 0];
                int v1 = facesWithOrder[i, 1];
                int v2 = facesWithOrder[i, 2];
                double x0 = cutMesh.Vertices[v0].X, y0 = cutMesh.Vertices[v0].Y;
                double x1 = cutMesh.Vertices[v1].X, y1 = cutMesh.Vertices[v1].Y;
                double x2 = cutMesh.Vertices[v2].X, y2 = cutMesh.Vertices[v2].Y;
                area.Add(0.5 * ((x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)));
            }
            DA.SetDataList(0, area);

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
            get { return new Guid("548639FB-8C1F-42A6-A14F-6CDDAA7000A0"); }
        }
    }
}