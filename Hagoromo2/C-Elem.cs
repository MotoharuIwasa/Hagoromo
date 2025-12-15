using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino.DocObjects;
using Rhino.Geometry;

namespace Hagoromo.DataStructure
{
    public class Elem : GH_Component
    {
        public Elem()
          : base("Element Info", "Elem",
              "Gather element information,部材端の自由度は固定がTrue(=1)",
              "Hagoromo", "Linear")
        {
        }

        public class Elemdata
        {
            public int SectId { get; set; }
            public List<bool> Constraint { get; set; }
            public List<double> CMQ { get; set; }
            public List<object> RotInfo { get; set; }
            public List<Line> Lines { get; set; }

            public Elemdata(int sectId, List<bool> constraint, List<double> cmq, List<object> rotInfo, List<Line> lines)
            {
                SectId = sectId;
                Constraint = constraint;
                CMQ = cmq;
                RotInfo = rotInfo;
                Lines = lines;
            }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            var defaultConstraints = new List<bool>(System.Linq.Enumerable.Repeat(true, 12));
            var defaultCMQ = new List<double>(System.Linq.Enumerable.Repeat(0.0, 6));

            pManager.AddLineParameter("Lines", "L", "The lines to calculate nodes", GH_ParamAccess.list);
            pManager.AddIntegerParameter("SectId", "SId", "Section Id of the element", GH_ParamAccess.item,1);
            pManager.AddBooleanParameter("Constraint", "Con", "Constraint of the element ends", GH_ParamAccess.list, defaultConstraints);
            pManager.AddNumberParameter("CoordAngle", "Ang", "Coord angle Value(degree)", GH_ParamAccess.item, 0.0);
            pManager.AddNumberParameter("CMQ(St)", "CMQ(St)", "CMQ values of start node", GH_ParamAccess.list, defaultCMQ);
            pManager.AddNumberParameter("CMQ(Ed)", "CMQ(Ed)", "CMQ values of end node", GH_ParamAccess.list, defaultCMQ);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Elemdata", "e", "Exported element data", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Line> lines = new List<Line>();
            int sectId = 0;
            List<bool> constraint = new List<bool>();
            double coordAngle = 0.0;
            List<double> cmq1 = new List<double>();
            List<double> cmq2 = new List<double>();

            if (!DA.GetDataList(0, lines)) return;
            if (!DA.GetData(1, ref sectId)) return;
            if (!DA.GetDataList(2, constraint)) return;
            if (!DA.GetData(3, ref coordAngle)) return;
            if (!DA.GetDataList(4, cmq1)) return;
            if (!DA.GetDataList(5, cmq2)) return;
            List<double> cmq = new List<double>();
            cmq.AddRange(cmq1);
            cmq.AddRange(cmq2);

            // Use a list of lists to store rotInfo for each line
            List<object> rotInfo = new List<object>();

            for (int i = 0; i < lines.Count; i++)
            {
                Line line = lines[i];
                double dx = line.To.X - line.From.X;
                double dy = line.To.Y - line.From.Y;
                double dz = line.To.Z - line.From.Z;
                double length = line.Length;
                double angle = coordAngle;

                List<double> lineRotInfo = new List<double> { angle, length, dx, dy, dz };

                rotInfo.Add(lineRotInfo);
            }

            Elemdata elemdata = new Elemdata(sectId, constraint, cmq, rotInfo, lines);

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