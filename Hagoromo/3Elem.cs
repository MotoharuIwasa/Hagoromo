using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino.DocObjects;
using Rhino.Geometry;

namespace Hagoromo
{
    public class Elem : GH_Component
    {
        public Elem()
          : base("Element Info", "Elem",
              "Gather element information",
              "Hagoromo", "Linear")
        {
        }

        public class Elemdata
        {
            public int SectId { get; set; }
            public List<int> Constraint { get; set; }
            public List<double> CMQ { get; set; }
            public double CordAngle { get; set; }
            public List<Line> Lines { get; set; }

            public Elemdata(int sectId, List<int> constraint, List<double> cmq, double cordAngle, List<Line> lines)
            {
                SectId = sectId;
                Constraint = constraint;
                CMQ = cmq;
                CordAngle = cordAngle;
                Lines = lines;
            }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            var defaultConstraints = new List<int>(System.Linq.Enumerable.Repeat(0, 12));
            var defaultCMQ = new List<double>(System.Linq.Enumerable.Repeat(0.0, 12));

            pManager.AddLineParameter("Lines", "L", "The lines to calculate nodes", GH_ParamAccess.list);
            pManager.AddIntegerParameter("SectId", "SId", "Section Id of the element", GH_ParamAccess.item,11);
            pManager.AddIntegerParameter("Constraint", "Con", "Constraint of the element ends", GH_ParamAccess.list, defaultConstraints);
            pManager.AddNumberParameter("CMQ", "CMQ", "CMQ values", GH_ParamAccess.list, defaultCMQ);
            pManager.AddNumberParameter("CordAngle", "Ang", "Cord angle Value", GH_ParamAccess.item, 0.0);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Elemdata", "e", "Exported element data", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Line> lines = new List<Line>();
            int sectId = 0;
            List<int> constraint = new List<int>();
            List<double> cmq = new List<double>();
            double cordAngle = 0.0;

            if (!DA.GetDataList(0, lines)) return;
            if (!DA.GetData(1, ref sectId)) return;
            if (!DA.GetDataList(2, constraint)) return;
            if (!DA.GetDataList(3, cmq)) return;
            if (!DA.GetData(4, ref cordAngle)) return;

            Elemdata elemdata = new Elemdata(sectId, constraint, cmq, cordAngle, lines);

            // Wrap Elemdata in GH_Elemdata
            DA.SetData(0, new GH_Elemdata(elemdata));
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
            get { return new Guid("853011DC-2918-4A37-95C3-75731D8FFB9D"); }
        }
    }
}