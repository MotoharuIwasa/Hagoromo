using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino.Collections;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using static Rhino.DocObjects.HatchPattern;

namespace Hagoromo.DataStructure
{
    public class ICon : GH_Component
    {
        public ICon()
          : base("Icon Info", "Icon",
              "Gather Icon information,True(=1)が固定、同じ固定条件のpointはまとめてpointsに入れてOK",
              "Hagoromo", "Linear")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            var defaultConstraints = new List<bool> { true, true, true, true, true, true };
            pManager.AddPointParameter("Points", "P", "Points defining the Icon", GH_ParamAccess.list);
            pManager.AddBooleanParameter("Constraints", "Con", "Constraint of the points", GH_ParamAccess.list, defaultConstraints);

        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Icondata", "i", "Exported Icon data", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Point3d> points = new List<Point3d>();
            List<bool> constraints = new List<bool>();

            // Retrieve inputs
            if (!DA.GetDataList(0, points)) return;
            if (!DA.GetDataList(1, constraints)) return;

            // Validate constraints
            if (constraints.Count != 6)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Constraints must have exactly 6 booleans.");
                return;
            }

            // Icondata 配列を作成
            Icondata[] icondataArray = new Icondata[points.Count];
            for (int i = 0; i < points.Count; i++)
            {
                List<Point3d> singlePointList = new List<Point3d> { points[i] };
                icondataArray[i] = new Icondata(singlePointList, constraints);
            }

            List<GH_Icondata> ghIcondataList = new List<GH_Icondata>();
            for (int i = 0; i < icondataArray.Length; i++)
            {
                ghIcondataList.Add(new GH_Icondata(icondataArray[i]));
            }

            DA.SetDataList(0, ghIcondataList);
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