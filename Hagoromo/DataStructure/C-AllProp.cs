using System;
using System.Collections.Generic;

using Grasshopper.Kernel;


namespace Hagoromo.DataStructure
{
    public class AllProp : GH_Component
    {
        public AllProp()
          : base("All Property Info", "AllProp",
              "Gather all property information and combine",
              "Hagoromo", "Input")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Propdata", "p", "List of Propdata", GH_ParamAccess.list);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("AllPropdata", "P", "Combined property data", GH_ParamAccess.item);
            pManager.AddTextParameter("Contents", "C", "Contents of AllPropdata", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var propdataList = new List<GH_Propdata>();

            if (!DA.GetDataList(0, propdataList) || propdataList.Count == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "No property data provided.");
                return;
            }

            var propertyArray = new object[propdataList.Count, 5];
            var contentStrings = new List<string>();

            for (int i = 0; i < propdataList.Count; i++)
            {
                if (!propdataList[i].IsValid || propdataList[i].Value == null)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, $"Invalid Propdata at index {i}.");
                    continue;
                }

                var propData = propdataList[i].Value;
                propertyArray[i, 0] = propData.Name;
                propertyArray[i, 1] = propData.Id;
                propertyArray[i, 2] = propData.Density;
                propertyArray[i, 3] = propData.YoungsModulus;
                propertyArray[i, 4] = propData.PoissonsRatio;

                contentStrings.Add($"P{i + 1}:{propData.Name}, PId:{propData.Id}, Den:{propData.Density}, " +
                    $"E:{propData.YoungsModulus}, POI:{propData.PoissonsRatio}");
            }

            var allPropdata = new AllPropdata(propertyArray);

            DA.SetData(0, new GH_AllPropdata(allPropdata));
            DA.SetData(1, string.Join("\n", contentStrings));
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
            get { return new Guid("71E90AA3-2D46-4225-BA78-C25B6FC1BE83"); }
        }
    }
}