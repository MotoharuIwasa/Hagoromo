using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using MathNet.Numerics.Distributions;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using static Rhino.Render.TextureGraphInfo;

namespace Hagoromo.GeometryTools
{
    public class RotateCutMesh : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent2 class.
        /// </summary>
        public RotateCutMesh()
          : base("RotateCutMesh", "RotateCutMesh",
              "RotateCutMesh",
              "Hagoromo", "Geometry")
        {
        }
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "Mesh or CutMesh", GH_ParamAccess.item);
            pManager.AddPointParameter("rotation center", "C", "rotation center", GH_ParamAccess.item);
            pManager.AddVectorParameter("rotation axis", "v", "rotation axis", GH_ParamAccess.item);
            pManager.AddAngleParameter("rotation angle", "angle(°)", "rotation angle (degree)", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "Mesh or CutMesh", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            object input = null;
            if (!DA.GetData(0, ref input)) return;

            CutMesh cutMesh = null;

            if (input is IGH_Goo goo)
            {
                // Mesh へのキャストを試す
                if (goo.CastTo(out Mesh m))
                {
                    cutMesh = new CutMesh(m);
                }
                // CutMesh へのキャストを試す
                else if (goo.CastTo(out CutMesh cm))
                {
                    cutMesh = cm.Clone();
                }
            }

            if (cutMesh == null)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Mesh または CutMesh を入力してください");
                return;
            }

            // 以降 cutMesh が確実に利用可能

            Vector3d axis = new Vector3d();
            DA.GetData(2, ref axis);

            Point3d center = new Point3d();
            DA.GetData(1, ref center);

            double theta = 0;
            DA.GetData(3, ref theta);
            theta = Math.PI * theta / 180;

            Transform rotationMatrix = Transform.Rotation(theta, axis, center);
            for (int i = 0; i < cutMesh.Vertices.Count; i++)
            {
                Point3d pt = cutMesh.Vertices[i]; // コピーを取得
                pt.Transform(rotationMatrix);     // コピーを変形
                cutMesh.Vertices[i] = pt;
            }

            DA.SetData(0, new GH_CutMesh(cutMesh));

        }

        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                //You can add image files to your project resources and access them like this:
                // return Resources.IconForThisComponent;
                return null;
            }
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("6480AB62-F839-41A8-86F5-3B7B5E7FF615"); }
        }
    }
}