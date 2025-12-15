using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;

using Rhino.Geometry;
using System;
using System.Collections.Generic;

using Grasshopper.Kernel.Types;

namespace Hagoromo.GeometryTools
{
    public class DeconstructCutMesh : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public DeconstructCutMesh()
          : base("DeconstructCutMesh", "DecCutMesh",
              "DeconstructCutMesh",
              "Hagoromo", "Geometry")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "Mesh or CutMesh", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("Vertices", "V", "vertices", GH_ParamAccess.list);
            pManager.AddIntegerParameter("Faces", "F", "faces", GH_ParamAccess.tree);
            pManager.AddIntegerParameter("Edges", "E", "edges", GH_ParamAccess.tree);
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



            DA.SetDataList(0, cutMesh.Vertices);
            int[,] faces = cutMesh.Faces;
            var tree = new GH_Structure<GH_Integer>();

            for (int i = 0; i < faces.GetLength(0); i++)
            {
                var path = new GH_Path(i); // 行ごとにPathを作る
                for (int j = 0; j < faces.GetLength(1); j++)
                {
                    tree.Append(new GH_Integer(faces[i, j]), path);
                }
            }
            DA.SetDataTree(1, tree);

            List<int[]> edges = cutMesh.Edges;
            var tree2 = new GH_Structure<GH_Integer>();

            for (int i = 0; i < edges.Count; i++)
            {
                GH_Path path2 = new GH_Path(i); // edgeごとにPathを作成
                foreach (int v in edges[i])
                {
                    tree2.Append(new GH_Integer(v), path2);
                }
            }
            DA.SetDataTree(2, tree2);

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
            get { return new Guid("7EC32330-72BA-4C91-B9C0-3DD3741A3119"); }
        }
    }
}