using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using System.Collections.Generic;

namespace Hagoromo.DataStructure
{
    public class AllElemdata
    {
        public double[,] NodeArray { get; set; }
        public object[,] ElementArray { get; set; }

        public AllElemdata(double[,] nodeArray, object[,] elementArray)
        {
            NodeArray = nodeArray;
            ElementArray = elementArray;
        }
    }

    public class GH_AllElemdata : GH_Goo<AllElemdata>
    {
        public GH_AllElemdata() : base() { }

        public GH_AllElemdata(AllElemdata allElemdata) : base(allElemdata) { }

        public override IGH_Goo Duplicate()
        {
            return new GH_AllElemdata(new AllElemdata(
                (double[,])Value.NodeArray.Clone(),
                (object[,])Value.ElementArray.Clone()
            ));
        }

        public override string ToString()
        {
            return $"AllElemdata with {Value.NodeArray.GetLength(0)} nodes and {Value.ElementArray.GetLength(0)} elements";
        }

        public override bool IsValid => Value != null;

        public override string IsValidWhyNot => IsValid ? string.Empty : "Invalid AllElemdata";

        public override string TypeName => "AllElemdata";

        public override string TypeDescription => "Wrapper for combined node and element array data";
    }
}
