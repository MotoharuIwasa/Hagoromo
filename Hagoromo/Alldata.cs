using System;
using System.Collections.Generic;
using System.Linq;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Rhino.Geometry;

namespace Hagoromo
{
    public class CombineAlldata : GH_Component
    {
        public CombineAlldata()
          : base("Combine Alldata", "Alldata",
              "Combine all structural data into one package",
              "Hagoromo", "Linear")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("AllPropdata", "P", "Property data", GH_ParamAccess.item);
            pManager.AddGenericParameter("AllSectdata", "S", "Section data", GH_ParamAccess.item);
            pManager.AddGenericParameter("AllElemdata", "E", "Element data", GH_ParamAccess.item);
            pManager.AddGenericParameter("AllIcondata", "I", "Icon data", GH_ParamAccess.item);
            pManager.AddGenericParameter("AllVcondata", "V", "Vcon data", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Alldata", "A", "Combined data package", GH_ParamAccess.item);
            pManager.AddTextParameter("Contents", "C", "Contents of combined data", GH_ParamAccess.tree);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_AllPropdata ghPropdata = null;
            GH_AllSectdata ghSectdata = null;
            GH_AllElemdata ghElemdata = null;
            GH_AllIcondata ghIcondata = null;
            GH_AllVcondata ghVcondata = null;

            if (!DA.GetData(0, ref ghPropdata) || ghPropdata == null || !ghPropdata.IsValid)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Invalid Property data");
                return;
            }
            if (!DA.GetData(1, ref ghSectdata) || ghSectdata == null || !ghSectdata.IsValid)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Invalid Section data");
                return;
            }
            if (!DA.GetData(2, ref ghElemdata) || ghElemdata == null || !ghElemdata.IsValid)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Invalid Element data");
                return;
            }
            if (!DA.GetData(3, ref ghIcondata) || ghIcondata == null || !ghIcondata.IsValid)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Invalid Icon data");
                return;
            }
            if (!DA.GetData(4, ref ghVcondata) || ghVcondata == null || !ghVcondata.IsValid)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Invalid Vcon data");
                return;
            }

            var propdata = ghPropdata.Value;
            var sectdata = ghSectdata.Value;
            var elemdata = ghElemdata.Value;
            var icondata = ghIcondata.Value;
            var vcondata = ghVcondata.Value;

            int nodeCount = elemdata.NodeArray.GetLength(0);
            var combinedNodeArray = new object[nodeCount, 3];

            // Create dictionaries for icon and vcon lookup
            var iconLookup = new Dictionary<Point3d, int[]>();
            for (int i = 0; i < icondata.NodeArray.GetLength(0); i++)
            {
                Point3d point = new Point3d(
                    icondata.NodeArray[i, 0],
                    icondata.NodeArray[i, 1],
                    icondata.NodeArray[i, 2]
                );
                // Icon Lookup
                var iconRow = new int[6];

                for (int j = 0; j < 6; j++)
                {
                    var iconData = (int[])icondata.IconArray[i, 0];
                    if (iconData[j] is int value)
                    {
                        iconRow[j] = value;
                    }
                    else
                    {
                        throw new InvalidCastException("IconArray element is not of type int.");
                    }
                }
                iconLookup[point] = iconRow;
            }

            var vconLookup = new Dictionary<Point3d, double[]>();
            for (int i = 0; i < vcondata.NodeArray.GetLength(0); i++)
            {
                Point3d point = new Point3d(
                    vcondata.NodeArray[i, 0],
                    vcondata.NodeArray[i, 1],
                    vcondata.NodeArray[i, 2]
                );
                // Vcon Lookup
                var vconRow = new double[6];
                for (int j = 0; j < 6; j++)
                {
                    var vconData = (double[])vcondata.VconArray[i, 0];
                    if (vconData[j] is double value)
                    {
                        vconRow[j] = value;
                    }
                    else
                    {
                        throw new InvalidCastException("VconArray element is not of type double.");
                    }
                }
                vconLookup[point] = vconRow;
            }

            // Process nodes
            for (int i = 0; i < nodeCount; i++)
            {
                var position = new double[3];
                position[0] = elemdata.NodeArray[i, 0];
                position[1] = elemdata.NodeArray[i, 1];
                position[2] = elemdata.NodeArray[i, 2];

                var iconRow = new int[6];
                if (iconLookup.TryGetValue(new Point3d(position[0], position[1], position[2]), out int[] iconValues))
                {
                    for (int j = 0; j < Math.Min(6, iconValues.Length); j++)
                        iconRow[j] = iconValues[j];
                }

                var vconRow = new double[6];
                if (vconLookup.TryGetValue(new Point3d(position[0], position[1], position[2]), out double[] vconValues))
                {
                    for (int j = 0; j < Math.Min(6, vconValues.Length); j++)
                        vconRow[j] = vconValues[j];
                }

                combinedNodeArray[i,0] = position;
                combinedNodeArray[i,1] = iconRow;
                combinedNodeArray[i,2] = vconRow;
            }

            var alldata = new Alldata(
                propdata.PropertyArray,
                sectdata.SectionArray,
                combinedNodeArray,
                elemdata.ElementArray
            );

            // Generate the preview as a tree
            var contentsTree = new Grasshopper.DataTree<string>();

            // Properties
            for (int i = 0; i < propdata.PropertyArray.GetLength(0); i++)
            {
                var row = new List<string>();
                for (int j = 0; j < propdata.PropertyArray.GetLength(1); j++)
                {
                    row.Add(propdata.PropertyArray[i, j]?.ToString() ?? "null");
                }
                contentsTree.Add(string.Join(", ", row), new GH_Path(1));
            }

            // Sections
            for (int i = 0; i < sectdata.SectionArray.GetLength(0); i++)
            {
                var row = new List<string>();
                for (int j = 0; j < sectdata.SectionArray.GetLength(1); j++)
                {
                    row.Add(sectdata.SectionArray[i, j]?.ToString() ?? "null");
                }
                contentsTree.Add(string.Join(", ", row), new GH_Path(2));
            }

            // Nodes
            for (int i = 0; i < nodeCount; i++)
            {
                var node0 = combinedNodeArray[i, 0];
                var node1 = combinedNodeArray[i, 1];
                var node2 = combinedNodeArray[i, 2];

                var position = string.Join(",", ((double[])node0).Select(d => d.ToString("F2")));
                var icon = string.Join(",", (int[])node1);
                var vcon = string.Join(",", ((double[])node2).Select(d => d.ToString("F2")));
                contentsTree.Add($"[{position}],[{icon}],[{vcon}]", new GH_Path(3));
            }

            // Elements
            for (int i = 0; i < elemdata.ElementArray.GetLength(0); i++)
            {
                var row = new List<string>
                    {
                        elemdata.ElementArray[i, 0]?.ToString() ?? "null",
                        $"[{string.Join(",", (int[])elemdata.ElementArray[i, 1])}]",
                        $"[{string.Join(",", (int[])elemdata.ElementArray[i, 2])}]",
                        $"[{string.Join(",", ((double[])elemdata.ElementArray[i, 3]).Select(d => d.ToString("F2")))}]",
                        $"[{string.Join(",", ((double[])elemdata.ElementArray[i, 4]).Select(d => d.ToString("F2")))}]"
                    };
                contentsTree.Add(string.Join(", ", row), new GH_Path(4));
            }

            DA.SetData(0, new GH_Alldata(alldata));
            DA.SetDataTree(1, contentsTree);
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
            get { return new Guid("12345678-90AB-CDEF-1234-567890ABCDEF"); }
        }
    }
}
