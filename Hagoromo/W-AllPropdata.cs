using Grasshopper.Kernel.Types;
using System;
using System.Collections.Generic;

namespace Hagoromo.DataStructure
{
    public class AllPropdata
    {
        public object[,] PropertyArray { get; set; }

        public AllPropdata(object[,] propertyArray)
        {
            if (propertyArray == null)
                throw new ArgumentNullException(nameof(propertyArray), "Property array cannot be null.");
            PropertyArray = propertyArray;
        }
    }

    public class GH_AllPropdata : GH_Goo<AllPropdata>
    {
        public GH_AllPropdata() : base() { }

        public GH_AllPropdata(AllPropdata allPropdata) : base(allPropdata) { }

        public override IGH_Goo Duplicate()
        {
            if (Value == null)
                throw new InvalidOperationException("Cannot duplicate a null AllPropdata value.");

            return new GH_AllPropdata(new AllPropdata(
                (object[,])Value.PropertyArray.Clone()
            ));
        }

        public override string ToString()
        {
            return $"AllPropdata with {Value.PropertyArray.GetLength(0)} properties";
        }

        public override bool IsValid => Value != null;

        public override string IsValidWhyNot => IsValid ? string.Empty : "Invalid AllPropdata";

        public override string TypeName => "AllPropdata";

        public override string TypeDescription => "Wrapper for combined property array data";
    }
}