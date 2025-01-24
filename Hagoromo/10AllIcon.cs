using System;
using System.Collections.Generic;
using System.Linq;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;

namespace Hagoromo
{
    public class AllIcon : GH_Component
    {
        public AllIcon()
          : base("All Icon Info", "AllIcon",
              "Gather all Icon information and combine",
              "Hagoromo", "Linear")
        {
        }

        private List<Point3d> _nodes = new List<Point3d>();

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Icondata", "i", "List of Icondata", GH_ParamAccess.list);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("AllIcondata", "I", "Combined Icon data", GH_ParamAccess.item);
            pManager.AddTextParameter("Contents", "C", "Contents of AllIcondata", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var icondataList = new List<GH_Icondata>();

            if (!DA.GetDataList(0, icondataList) || icondataList.Count == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "No icon data provided.");
                return;
            }

            _nodes.Clear();

            foreach (var ghIcondata in icondataList)
            {
                if (!ghIcondata.IsValid || ghIcondata.Value == null)
                    continue;

                var icondata = ghIcondata.Value;
                foreach (var point in icondata.Points)
                {
                    _nodes.Add(point);
                }
            }

            double[,] nodeArray = new double[_nodes.Count, 3];
            object[,] iconArray = new object[icondataList.Count, 2];

            for (int i = 0; i < _nodes.Count; i++)
            {
                nodeArray[i, 0] = _nodes[i].X;
                nodeArray[i, 1] = _nodes[i].Y;
                nodeArray[i, 2] = _nodes[i].Z;
            }

            for (int i = 0; i < icondataList.Count; i++)
            {
                var icondata = icondataList[i].Value;
                if (icondata == null) continue;

                iconArray[i, 0] = icondata.Constraints.ToArray();
            }

            var combinedData = new AllIcondata(nodeArray, iconArray);

            var nodeText = string.Join("\n", Enumerable.Range(0, _nodes.Count).Select(i =>
                $"I{i} X:{nodeArray[i, 0]:F2},Y: {nodeArray[i, 1]:F2}, Z:{nodeArray[i, 2]:F2} " +
                $"Con:[{string.Join(",", (int[])iconArray[i, 0])}] "
            ));

            DA.SetData(0, new GH_AllIcondata(combinedData));
            DA.SetData(1, $"{nodeText}");
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
            get { return new Guid("36CCC910-1C9D-4C55-81CB-D38773AFC9A3"); }
        }
    }
}