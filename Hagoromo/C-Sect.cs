using System;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Hagoromo.DataStructure
{
    public class Sect : GH_Component
    {
        public Sect()
          : base("Section Info", "Sect",
              "Gather section information",
              "Hagoromo", "Linear")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("SectName", "N", "Define Section Name", GH_ParamAccess.item, "SName");
            pManager.AddIntegerParameter("SectId", "SId", "Define Section Id", GH_ParamAccess.item, 1);
            pManager.AddIntegerParameter("PropId", "PId", "Property Id of the section", GH_ParamAccess.item, 1);
            pManager.AddNumberParameter("Area", "A", "Section Area", GH_ParamAccess.item, 0.0);
            pManager.AddNumberParameter("IXX", "Ix", "Moment of Inertia about X-axis", GH_ParamAccess.item, 0.0);
            pManager.AddNumberParameter("IYY", "Iy", "Moment of Inertia about Y-axis", GH_ParamAccess.item, 0.0);
            pManager.AddNumberParameter("VEN", "V", "Shear Area Coefficient", GH_ParamAccess.item, 0.0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Sectdata", "s", "Exported section data", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            string sectName = string.Empty;
            int sectId = 0;
            int propId = 0;
            double area = 0.0;
            double ixx = 0.0;
            double iyy = 0.0;
            double ven = 0.0;

            if (!DA.GetData(0, ref sectName)) return;
            if (!DA.GetData(1, ref sectId)) return;
            if (!DA.GetData(2, ref propId)) return;
            if (!DA.GetData(3, ref area)) return;
            if (!DA.GetData(4, ref ixx)) return;
            if (!DA.GetData(5, ref iyy)) return;
            if (!DA.GetData(6, ref ven)) return;

            var sectdata = new Sectdata(sectName, sectId, propId, area, ixx, iyy, ven);
            DA.SetData(0, new GH_Sectdata(sectdata));
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
            get { return new Guid("C3500BE4-0951-4FE9-BF7C-22DDE7B581C1"); }
        }
    }
}