using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Hagoromo
{
    public class Vcon : GH_Component
    {
        public Vcon()
          : base("Vcon Info", "Vcon",
              "Extract Vcondata from Points and Forces",
              "Hagoromo", "Linear")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            var defaultForces = new List<double> { 0, 0, 0, 0, 0, 0 };
            pManager.AddPointParameter("Points", "P", "The points for Vcondata", GH_ParamAccess.list);
            pManager.AddNumberParameter("Forces", "F", "6 double values representing forces (Fx, Fy, Fz, Mx, My, Mz)", GH_ParamAccess.list,defaultForces);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Vcondata", "v", "Exported Vcondata", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Point3d> points = new List<Point3d>();
            List<double> forces = new List<double>();

            // Retrieve inputs
            if (!DA.GetDataList(0, points)) return;
            if (!DA.GetDataList(1, forces)) return;

            // Ensure the forces list contains exactly 6 doubles
            if (forces.Count != 6)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Forces input must contain exactly 6 double values.");
                return;
            }

            // Create Vcondata
            Vcondata vcondata = new Vcondata(points, new List<double>(forces));

            // Wrap Vcondata in GH_Vcondata
            DA.SetData(0, new GH_Vcondata(vcondata));
        }

        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return null;
            }
        }

        public override Guid ComponentGuid
        {
            get { return new Guid("178E5712-C34C-4FE8-887E-F8EC9E7D059C"); }
        }
    }
}