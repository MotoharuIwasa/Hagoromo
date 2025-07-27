using System;
using Grasshopper.Kernel.Types;

namespace Hagoromo.DataStructure
{
    public class Propdata
    {
        public string Name { get; set; }
        public int Id { get; set; }
        public double Density { get; set; }
        public double YoungsModulus { get; set; }
        public double PoissonsRatio { get; set; }

        public Propdata(string name, int id, double density, double youngsModulus, double poissonsRatio)
        {
            Name = name;
            Id = id;
            Density = density;
            YoungsModulus = youngsModulus;
            PoissonsRatio = poissonsRatio;
        }
    }

    public class GH_Propdata : GH_Goo<Propdata>
    {
        public GH_Propdata() : base() { }
        public GH_Propdata(Propdata propdata) : base(propdata) { }

        public override IGH_Goo Duplicate()
        {
            return new GH_Propdata(new Propdata(
                Value.Name,
                Value.Id,
                Value.Density,
                Value.YoungsModulus,
                Value.PoissonsRatio
            ));
        }

        public override string ToString()
        {
            return $"Propdata {Value.Name},{Value.Id}";
        }

        public override bool IsValid => Value != null;

        public override string IsValidWhyNot => IsValid ? string.Empty : "Invalid Propdata";

        public override string TypeName => "Propdata";

        public override string TypeDescription => "Custom data type for Propdata";

        public override bool CastFrom(object source)
        {
            if (source is Propdata propdata)
            {
                Value = propdata;
                return true;
            }
            return false;
        }

        public override bool CastTo<Q>(ref Q target)
        {
            if (typeof(Q).IsAssignableFrom(typeof(Propdata)))
            {
                target = (Q)(object)Value;
                return true;
            }

            target = default;
            return false;
        }
    }
}
