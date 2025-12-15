
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino.Display;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Data;
using System.Drawing;

namespace Hagoromo.DataStructure
{
    public class Vcon : GH_Component, IGH_PreviewObject

    {
        public Vcon()
          : base("Vcon Info", "Vcon",
              "Extract Vcondata from Points and Forces,iconが0のところでのvconはただの外力だが、iconが1のところでvconが0以外ならその値が強制変位とみなされる。",
              "Hagoromo", "Input")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            var defaultForces = new List<double> { 0, 0, 0, 0, 0, 0 };
            pManager.AddPointParameter("Points", "P", "The points for Vcondata", GH_ParamAccess.list);
            pManager.AddNumberParameter("Forces", "F", "6 double values representing forces (Fx, Fy, Fz, Mx, My, Mz)", GH_ParamAccess.list, defaultForces);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Vcondata", "v", "Exported Vcondata", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Point3d> points = new List<Point3d>();
            List<double> forces = new List<double>();

            // Retrieve inputs
            if (!DA.GetDataList(0, points)) return;
            if (!DA.GetDataList(1, forces)) return;

            // Ensure the forces list contains exactly 6 doubles
            if (forces.Count != 6)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Forces input must contain exactly 6 double values.");
                return;
            }

            // Icondata 配列を作成
            Vcondata[] vcondataArray = new Vcondata[points.Count];
            for (int i = 0; i < points.Count; i++)
            {
                List<Point3d> singlePointList = new List<Point3d> { points[i] };
                vcondataArray[i] = new Vcondata(singlePointList, forces);
            }

            List<GH_Vcondata> ghVcondataList = new List<GH_Vcondata>();
            for (int i = 0; i < vcondataArray.Length; i++)
            {
                ghVcondataList.Add(new GH_Vcondata(vcondataArray[i]));
            }

            DA.SetDataList(0, ghVcondataList);
            
        }

        /*
        public void DrawViewportWires(GH_PreviewWireArgs args)
        {
            foreach (IGH_Param param in this.Params.Input)
            {
                if (param.Name == "Points")
                {
                    foreach (IGH_Goo goo in param.VolatileData.AllData(true))
                    {
                        if (goo is GH_Point ghPoint)
                        {
                            args.Pipeline.DrawPoint(ghPoint.Value, PointStyle.X, 10, Color.LimeGreen);
                            //args.Pipeline.DrawPoint(ghPoint.Value, 10, Color.LimeGreen);
                        }
                    }
                }
            }
        }

        public override bool IsPreviewCapable => true;

        public override BoundingBox ClippingBox
        {
            get
            {
                BoundingBox box = BoundingBox.Empty;
                foreach (IGH_Param param in this.Params.Input)
                {
                    if (param.Name == "Points")
                    {
                        foreach (IGH_Goo goo in param.VolatileData.AllData(true))
                        {
                            if (goo is GH_Point ghPoint)
                                box = BoundingBox.Union(box, new BoundingBox(ghPoint.Value, ghPoint.Value));
                        }
                    }
                }
                return box.IsValid ? box : BoundingBox.Unset;
            }
        }
        */
        
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return null;
            }
        }

        public override Guid ComponentGuid
        {
            get { return new Guid("178E5712-C34C-4FE8-887E-F8EC9E7D059C"); }
        }


    }
}


