using System;
using Grasshopper.Kernel;
using Rhino.Geometry;


public class SectionProperty : GH_Component
{
    public SectionProperty()
      : base("Section Property", "SectProps",
          "Calculate area and second moments of area from a closed planar curve.入力した単位と同じ単位で出力(rhinoの単位は無関係）rhinoの座標系周りで計算されることに注意",
          "Hagoromo", "Linear")
    {
    }

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        pManager.AddCurveParameter("Curve", "C", "Closed planar curve", GH_ParamAccess.item);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        pManager.AddNumberParameter("Area", "A", "Area of the curve", GH_ParamAccess.item);
        pManager.AddPointParameter("Centroid", "G", "Centroid of the area", GH_ParamAccess.item);
        pManager.AddNumberParameter("Ixx", "Ixx", "Moment of inertia about X axis", GH_ParamAccess.item);
        pManager.AddNumberParameter("Iyy", "Iyy", "Moment of inertia about Y axis", GH_ParamAccess.item);
    }

    protected override void SolveInstance(IGH_DataAccess DA)
    {
        Curve curve = null;
        if (!DA.GetData(0, ref curve)) return;

        if (curve == null || !curve.IsClosed || !curve.IsPlanar())
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Input must be a closed planar curve.");
            return;
        }

        var amp = AreaMassProperties.Compute(curve);
        if (amp == null)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Failed to compute area properties.");
            return;
        }

        DA.SetData(0, amp.Area);
        DA.SetData(1, amp.Centroid);
        var moi = amp.WorldCoordinatesMomentsOfInertia;
        DA.SetData(2, moi.X);
        DA.SetData(3, moi.Y);

    }
    public override Guid ComponentGuid
        {
            get { return new Guid("B3C94C33-D6D2-482B-838B-7AECED361C59"); }
        }
    
}