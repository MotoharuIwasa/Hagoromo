using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Hagoromo
{
    public class AllSect : GH_Component
    {
        public AllSect()
          : base("All Section Info", "AllSect",
              "Gather all section information and combine",
              "Hagoromo", "Linear")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Sectdata", "s", "List of Sectdata", GH_ParamAccess.list);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("AllSectdata", "S", "Combined section data", GH_ParamAccess.item);
            pManager.AddTextParameter("Contents", "C", "Contents of AllSectdata", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var sectdataList = new List<GH_Sectdata>();

            if (!DA.GetDataList(0, sectdataList) || sectdataList.Count == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "No section data provided.");
                return;
            }

            var sectionArray = new object[sectdataList.Count, 7];
            var contentStrings = new List<string>();

            for (int i = 0; i < sectdataList.Count; i++)
            {
                if (!sectdataList[i].IsValid || sectdataList[i].Value == null)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, $"Invalid Sectdata at index {i}.");
                    continue;
                }

                var sectdata = sectdataList[i].Value;

                sectionArray[i, 0] = sectdata.SectName; // Section Name
                sectionArray[i, 1] = sectdata.SectId;   // Section ID
                sectionArray[i, 2] = sectdata.PropId;   // Property ID
                sectionArray[i, 3] = sectdata.Area;     // Section Area
                sectionArray[i, 4] = sectdata.IXX;      // Moment of Inertia (X-axis)
                sectionArray[i, 5] = sectdata.IYY;      // Moment of Inertia (Y-axis)
                sectionArray[i, 6] = sectdata.VEN;      // Shear Area Coefficient

                contentStrings.Add($"S{i + 1}:{sectdata.SectName}, " +
                    $"SId:{sectdata.SectId}, PId:{sectdata.PropId}, A:{sectdata.Area}, " +
                    $"IX:{sectdata.IXX}, IY:{sectdata.IYY}, VEN:{sectdata.VEN}");
            }

            var allSectdata = new AllSectdata(sectionArray);

            DA.SetData(0, new GH_AllSectdata(allSectdata));
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
            get { return new Guid("A785FA2E-13CA-4956-BB4C-4A512ACD8796"); }
        }
    }
}
