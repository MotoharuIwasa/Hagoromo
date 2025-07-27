using Grasshopper.Kernel.Types;
using System;
using System.Collections.Generic;

namespace Hagoromo.DataStructure
{
    public class AllSectdata
    {
        public object[,] SectionArray { get; set; }

        public AllSectdata(object[,] sectionArray)
        {
            if (sectionArray == null)
                throw new ArgumentNullException(nameof(sectionArray), "Section array cannot be null.");
            SectionArray = sectionArray;
        }
    }

    public class GH_AllSectdata : GH_Goo<AllSectdata>
    {
        public GH_AllSectdata() : base() { }

        public GH_AllSectdata(AllSectdata allSectdata) : base(allSectdata) { }

        public override IGH_Goo Duplicate()
        {
            if (Value == null)
                throw new InvalidOperationException("Cannot duplicate a null AllSectdata value.");

            return new GH_AllSectdata(new AllSectdata(
                (object[,])Value.SectionArray.Clone()
            ));
        }

        public override string ToString()
        {
            return $"AllSectdata with {Value.SectionArray.GetLength(0)} sections";
        }

        public override bool IsValid => Value != null;

        public override string IsValidWhyNot => IsValid ? string.Empty : "Invalid AllSectdata";

        public override string TypeName => "AllSectdata";

        public override string TypeDescription => "Wrapper for combined section array data";
    }
}
