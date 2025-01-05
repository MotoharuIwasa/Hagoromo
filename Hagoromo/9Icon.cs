using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using static Rhino.DocObjects.HatchPattern;

namespace Hagoromo
{
    public class ICon : GH_Component
    {
        public ICon()
          : base("Icon Info", "Icon",
              "Gather Icon information",
              "Hagoromo", "Linear")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            var defaultConstraints = new List<int> { 0, 0, 0, 0, 0, 0 };
            pManager.AddPointParameter("Points", "P", "Points defining the Icon", GH_ParamAccess.list);
            pManager.AddIntegerParameter("Constraints", "Con", "Constraint of the points", GH_ParamAccess.list, defaultConstraints);

        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Icondata", "i", "Exported Icon data", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Point3d> points = new List<Point3d>();
            List<int> constraints = new List<int>();

            // Retrieve inputs
            if (!DA.GetDataList(0, points)) return;
            if (!DA.GetDataList(1, constraints)) return;

            // Validate constraints
            if (constraints.Count != 6)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Constraints must have exactly 6 integers.");
                return;
            }

            // Create Icondata instance
            Icondata icondata = new Icondata(points, constraints);

            // Wrap Icondata in GH_Icondata
            DA.SetData(0, new GH_Icondata(icondata));
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
            get { return new Guid("03F83A50-6518-4A12-A810-2B8A695E503B"); }
        }
    }
}