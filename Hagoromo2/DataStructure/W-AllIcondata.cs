using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using System.Collections.Generic;

namespace Hagoromo.DataStructure
{
    public class AllIcondata
    {
        public double[,] NodeArray { get; set; }
        public object[,] IconArray { get; set; }

        public AllIcondata(double[,] nodeArray, object[,] iconArray)
        {
            NodeArray = nodeArray;
            IconArray = iconArray;
        }
    }

    public class GH_AllIcondata : GH_Goo<AllIcondata>
    {
        public GH_AllIcondata() : base() { }

        public GH_AllIcondata(AllIcondata allIcondata) : base(allIcondata) { }

        public override IGH_Goo Duplicate()
        {
            return new GH_AllIcondata(new AllIcondata(
                (double[,])Value.NodeArray.Clone(),
                (object[,])Value.IconArray.Clone()
            ));
        }

        public override string ToString()
        {
            return $"AllIcondata with {Value.NodeArray.GetLength(0)} nodes";
        }

        public override bool IsValid => Value != null;

        public override string IsValidWhyNot => IsValid ? string.Empty : "Invalid AllIcondata";

        public override string TypeName => "AllIcondata";

        public override string TypeDescription => "Wrapper for combined node and icon array data";
    }
}