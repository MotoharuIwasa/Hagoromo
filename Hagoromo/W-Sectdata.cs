using System;
using Grasshopper.Kernel.Types;

namespace Hagoromo.DataStructure
{
    public class Sectdata
    {
        public string SectName { get; set; }
        public int SectId { get; set; }
        public int PropId { get; set; }
        public double Area { get; set; }
        public double IXX { get; set; }
        public double IYY { get; set; }
        public double VEN { get; set; }

        public Sectdata(string sectName, int sectId, int propId, double area, double ixx, double iyy, double ven)
        {
            SectName = sectName;
            SectId = sectId;
            PropId = propId;
            Area = area;
            IXX = ixx;
            IYY = iyy;
            VEN = ven;
        }
    }

    public class GH_Sectdata : GH_Goo<Sectdata>
    {
        public GH_Sectdata() : base() { }
        public GH_Sectdata(Sectdata sectdata) : base(sectdata) { }

        public override IGH_Goo Duplicate()
        {
            return new GH_Sectdata(new Sectdata(
                Value.SectName,
                Value.SectId,
                Value.PropId,
                Value.Area,
                Value.IXX,
                Value.IYY,
                Value.VEN
            ));
        }

        public override string ToString()
        {
            return $"Sectdata {Value.SectName}, {Value.SectId},{Value.PropId}";
        }

        public override bool IsValid => Value != null;

        public override string IsValidWhyNot => IsValid ? string.Empty : "Invalid Sectdata";

        public override string TypeName => "Sectdata";

        public override string TypeDescription => "Custom data type for Sectdata";

        public override bool CastFrom(object source)
        {
            if (source is Sectdata sectdata)
            {
                Value = sectdata;
                return true;
            }
            return false;
        }

        public override bool CastTo<Q>(ref Q target)
        {
            if (typeof(Q).IsAssignableFrom(typeof(Sectdata)))
            {
                target = (Q)(object)Value;
                return true;
            }

            target = default;
            return false;
        }
    }
}
