using System;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Hagoromo
{
    public class Prop : GH_Component
    {
        public Prop()
          : base("Property Info", "Prop",
              "Gather property information",
              "Hagoromo", "Linear")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("PropName", "N", "Define Property Name", GH_ParamAccess.item, "PName");
            pManager.AddIntegerParameter("PropId", "PId", "Define Property Id", GH_ParamAccess.item, 1);
            pManager.AddNumberParameter("Density", "Den", "Property Density", GH_ParamAccess.item, 0);
            pManager.AddNumberParameter("Young's Modulus", "E", "Property Young's Modulus", GH_ParamAccess.item, 0.0);
            pManager.AddNumberParameter("Poisson's Ratio", "POI", "Property Poisson's Ratio", GH_ParamAccess.item, 0.0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Propdata", "p", "Exported property data ", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            string name = string.Empty;
            int id = 0;
            double density = 0;
            double youngsModulus = 0;
            double poissonsRatio = 0;

            if (!DA.GetData(0, ref name)) return;
            if (!DA.GetData(1, ref id)) return;
            if (!DA.GetData(2, ref density)) return;
            if (!DA.GetData(3, ref youngsModulus)) return;
            if (!DA.GetData(4, ref poissonsRatio)) return;

            var propdata = new Propdata(name, id, density, youngsModulus, poissonsRatio);
            DA.SetData(0, new GH_Propdata(propdata));
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
            get { return new Guid("3681EBF2-22AD-416A-9029-975E6AB4EE38"); }
        }
    }
}
