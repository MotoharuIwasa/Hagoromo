using Grasshopper.Kernel.Types;
using System;
using System.Collections.Generic;

namespace Hagoromo.DataStructure
{
    public class Alldata
    {
        public object[,] PropertyArray { get; set; }
        public object[,] SectionArray { get; set; }
        public object[,] NodeArray { get; set; }
        public object[,] ElementArray { get; set; }
        public double[] GravityArray { get; set; }

        // **デフォルトコンストラクタを追加**
        public Alldata()
        {
            PropertyArray = new object[,] {};
            SectionArray = new object[,] {};
            NodeArray = new object[,] {};
            ElementArray = new object[,] {};
            GravityArray = new double[3];

        }
        public Alldata(object[,] propertyArray, object[,] sectionArray, object[,] nodeArray, object[,] elementArray, double[] gravityArray)
        {
            if (propertyArray == null)
                throw new ArgumentNullException(nameof(propertyArray), "Property array cannot be null.");
            if (sectionArray == null)
                throw new ArgumentNullException(nameof(sectionArray), "Section array cannot be null.");
            if (nodeArray == null)
                throw new ArgumentNullException(nameof(nodeArray), "Node array cannot be null.");
            if (elementArray == null)
                throw new ArgumentNullException(nameof(elementArray), "Element array cannot be null.");
            if (gravityArray == null)
                throw new ArgumentNullException(nameof(gravityArray), "Gravity array cannot be null.");

            PropertyArray = propertyArray;
            SectionArray = sectionArray;
            NodeArray = nodeArray;
            ElementArray = elementArray;
            GravityArray = gravityArray;
        }
    }

    public class GH_Alldata : GH_Goo<Alldata>
    {
        public GH_Alldata() : base() { }

        public GH_Alldata(Alldata alldata) : base(alldata) { }

        public override IGH_Goo Duplicate()
        {
            if (Value == null)
                throw new InvalidOperationException("Cannot duplicate a null Alldata value.");

            return new GH_Alldata(new Alldata(
                (object[,])Value.PropertyArray.Clone(),
                (object[,])Value.SectionArray.Clone(),
                (object[,])Value.NodeArray.Clone(),
                (object[,])Value.ElementArray.Clone(),
                (double[])Value.GravityArray.Clone()
            ));
        }

        public override string ToString()
        {
            return $"Alldata with {Value.PropertyArray.GetLength(0)} properties, {Value.SectionArray.GetLength(0)} sections, {Value.NodeArray.GetLength(0)} nodes, and {Value.ElementArray.GetLength(0)} elements.";
        }

        public override bool IsValid => Value != null;

        public override string IsValidWhyNot => IsValid ? string.Empty : "Invalid Alldata";

        public override string TypeName => "Alldata";

        public override string TypeDescription => "Wrapper for combined property, section, node, and element array data";
    }
}