﻿using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using System.Collections.Generic;

namespace Hagoromo
{
    public class GH_Elemdata : GH_Goo<Elem.Elemdata>
    {
        public GH_Elemdata() : base() { }
        public GH_Elemdata(Elem.Elemdata elemdata) : base(elemdata) { }

        public override IGH_Goo Duplicate()
        {
            return new GH_Elemdata(new Elem.Elemdata(
                Value.SectId,
                new List<int>(Value.Constraint),
                new List<double>(Value.CMQ),
                Value.CordAngle,
                new List<Line>(Value.Lines)
            ));
        }

        public override string ToString()
        {
            return $"Elemdata Sect{Value.SectId}";
            
            //return $"Elemdata: SectId={Value.SectId}, Constraints=[{string.Join(",", Value.Constraint)}], " +
            //       $"CMQ=[{string.Join(",", Value.CMQ)}], CordAngle={Value.CordAngle}";
        }

        public override bool IsValid => Value != null;

        public override string IsValidWhyNot => IsValid ? string.Empty : "Invalid Elemdata";

        public override string TypeName => "Elemdata";

        public override string TypeDescription => "Custom data type for Elemdata";

        public override bool CastFrom(object source)
        {
            if (source is Elem.Elemdata elemdata)
            {
                Value = elemdata;
                return true;
            }
            return false;
        }

        public override bool CastTo<Q>(ref Q target)
        {
            if (typeof(Q).IsAssignableFrom(typeof(Elem.Elemdata)))
            {
                target = (Q)(object)Value;
                return true;
            }

            target = default;
            return false;
        }
    }
}