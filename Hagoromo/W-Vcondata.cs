using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using System.Collections.Generic;

namespace Hagoromo.DataStructure
{
    public class Vcondata
    {
        public List<Point3d> Points { get; set; }
        public List<double> Forces { get; set; }

        public Vcondata(List<Point3d> points, List<double> forces)
        {
            Points = points;
            Forces = forces;
        }
    }

    public class GH_Vcondata : GH_Goo<Vcondata>
    {
        public GH_Vcondata() : base() { }

        public GH_Vcondata(Vcondata vcondata) : base(vcondata) { }

        public override IGH_Goo Duplicate()
        {
            return new GH_Vcondata(new Vcondata(
                new List<Point3d>(Value.Points),
                new List<double>(Value.Forces)
            ));
        }

        public override string ToString()
        {
            return $"Vcondata with {Value.Points.Count} points";
        }

        public override bool IsValid => Value != null;

        public override string IsValidWhyNot => IsValid ? string.Empty : "Invalid Vcondata";

        public override string TypeName => "Vcondata";

        public override string TypeDescription => "Custom data type for Vcondata";

        public override bool CastFrom(object source)
        {
            if (source is Vcondata vcondata)
            {
                Value = vcondata;
                return true;
            }
            return false;
        }

        public override bool CastTo<Q>(ref Q target)
        {
            if (typeof(Q).IsAssignableFrom(typeof(Vcondata)))
            {
                target = (Q)(object)Value;
                return true;
            }

            target = default;
            return false;
        }
    }
}