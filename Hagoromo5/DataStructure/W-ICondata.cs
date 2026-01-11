using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using System.Collections.Generic;

namespace Hagoromo.DataStructure
{
    public class Icondata
    {
        public List<Point3d> Points { get; set; }
        public List<bool> Constraints { get; set; }

        public Icondata(List<Point3d> points, List<bool> constraints)
        {
            Points = points;
            Constraints = constraints;
        }
    }

    public class GH_Icondata : GH_Goo<Icondata>
    {
        public GH_Icondata() : base() { }

        public GH_Icondata(Icondata icondata) : base(icondata) { }

        public override IGH_Goo Duplicate()
        {
            return new GH_Icondata(new Icondata(
                new List<Point3d>(Value.Points),
                new List<bool>(Value.Constraints)
            ));
        }

        public override string ToString()
        {
            return $"Icondata {Value.Points.Count} points of {Value.Constraints}";
        }

        public override bool IsValid => Value != null;

        public override string IsValidWhyNot => IsValid ? string.Empty : "Invalid Icondata";

        public override string TypeName => "Icondata";

        public override string TypeDescription => "Custom data type for Icondata";

        public override bool CastFrom(object source)
        {
            if (source is Icondata icondata)
            {
                Value = icondata;
                return true;
            }
            return false;
        }

        public override bool CastTo<Q>(ref Q target)
        {
            if (typeof(Q).IsAssignableFrom(typeof(Icondata)))
            {
                target = (Q)(object)Value;
                return true;
            }

            target = default;
            return false;
        }
    }
}