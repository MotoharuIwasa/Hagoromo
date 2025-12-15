using Grasshopper.Kernel.Types;
using System;

namespace Hagoromo.DataStructure
{
    public class AllVcondata
    {
        public double[,] NodeArray { get; set; }
        public object[,] VconArray { get; set; }

        public AllVcondata(double[,] nodeArray, object[,] vconArray)
        {
            NodeArray = nodeArray;
            VconArray = vconArray;
        }
    }

    public class GH_AllVcondata : GH_Goo<AllVcondata>
    {
        public GH_AllVcondata() : base() { }

        public GH_AllVcondata(AllVcondata allVcondata) : base(allVcondata) { }

        public override IGH_Goo Duplicate()
        {
            return new GH_AllVcondata(new AllVcondata(
                (double[,])Value.NodeArray.Clone(),
                (object[,])Value.VconArray.Clone()
            ));
        }

        public override string ToString()
        {
            return $"AllVcondata with {Value.NodeArray.GetLength(0)} nodes";
        }

        public override bool IsValid => Value != null;

        public override string IsValidWhyNot => IsValid ? string.Empty : "Invalid AllVcondata";

        public override string TypeName => "AllVcondata";

        public override string TypeDescription => "Wrapper for combined node and Vcon array data";
    }
}