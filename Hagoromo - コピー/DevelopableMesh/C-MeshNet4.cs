using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using Hagoromo.GeometryTools;

namespace Hagoromo.DevelopableMesh
{
    public class MeshNet4 : GH_Component
    {
        public MeshNet4()
          : base("MeshNet4", "MeshNet4",
              "NeshNet4",
              "Hagoromo", "Optimization")
        {
        }

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
            //pManager.AddNumberParameter("boundary", "be", "b", GH_ParamAccess.list);
            //pManager.AddNumberParameter("boundary", "be", "b", GH_ParamAccess.list);
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

            CutMesh newMesh = NetTools.NetBFF(mesh);
            GH_CutMesh ghCutMesh = new GH_CutMesh(newMesh);
            DA.SetData(0, ghCutMesh);
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
            get { return new Guid("5B335F69-A190-45D5-875B-BD4DEA66A9EA"); }
        }
    }
}