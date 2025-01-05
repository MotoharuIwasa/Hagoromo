using System;
using System.Collections.Generic;
using System.Linq;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Hagoromo
{
    public class AllVcon : GH_Component
    {
        public AllVcon()
          : base("All Vcon Info", "AllVcon",
              "Gather all Vcon information and combine",
              "Hagoromo", "Linear")
        {
        }

        private List<Point3d> _nodes = new List<Point3d>();


        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Vcondata", "v", "List of Vcondata to combine", GH_ParamAccess.list);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("AllVcondata", "V", "Combined Vcon data", GH_ParamAccess.item);
            pManager.AddTextParameter("Contents", "C", "Contents of AllVcondata", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var vcondataList = new List<GH_Vcondata>();

            // Retrieve inputs
            if (!DA.GetDataList(0, vcondataList) || vcondataList.Count == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "No Vcondata provided.");
                return;
            }

            _nodes.Clear();

            foreach (var ghVcondata in vcondataList)
            {
                if (!ghVcondata.IsValid || ghVcondata.Value == null)
                    continue;

                var vcondata = ghVcondata.Value;
                foreach (var point in vcondata.Points)
                {
                    _nodes.Add(point);
                }
            }

            var nodeArray = new double[_nodes.Count, 3];
            object[,] vconArray = new object[vcondataList.Count, 2];

            for (int i = 0; i < _nodes.Count; i++)
            {
                nodeArray[i, 0] = _nodes[i].X;
                nodeArray[i, 1] = _nodes[i].Y;
                nodeArray[i, 2] = _nodes[i].Z;
            }

            for (int i = 0; i < vcondataList.Count; i++)
            {
                var vcondata = vcondataList[i].Value;
                if (vcondata == null) continue;

                vconArray[i, 0] = vcondata.Forces.ToArray();
            }

            var combinedData = new AllVcondata(nodeArray, vconArray);

            // Generate output strings for visualization
            var nodeText = string.Join("\n", Enumerable.Range(0, _nodes.Count).Select(i =>
                $"V{i} X: {nodeArray[i, 0]}, Y: {nodeArray[i, 1]}, Z: {nodeArray[i, 2]} | " +
                $"Forces: [{string.Join(",", (double[])vconArray[i,0])}]"
            ));

            // Set outputs
            DA.SetData(0, new GH_AllVcondata(combinedData));
            DA.SetData(1, nodeText);
        }

        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return null; // Add your custom icon here
            }
        }

        public override Guid ComponentGuid
        {
            get { return new Guid("9430C482-7082-4E94-B511-69FB5985A89A"); }
        }
    }
}
