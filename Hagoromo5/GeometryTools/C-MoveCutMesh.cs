using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using Grasshopper.Kernel.Types;

namespace Hagoromo.GeometryTools
{
    public class MoveCutMesh : GH_Component
    {
        public MoveCutMesh()
          : base("MoveCutMesh", "MoveCM",
              "複数の Mesh または CutMesh を移動させます",
              "Hagoromo", "Geometry")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            // アクセスを .list に変更
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "Mesh or CutMesh のリスト", GH_ParamAccess.list);
            pManager.AddVectorParameter("move direction", "D", "移動ベクトル", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            // 出力もリスト形式にする
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "移動後の Mesh or CutMesh リスト", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // リストとして入力を受け取る
            List<object> inputList = new List<object>();
            Vector3d vector = new Vector3d();

            if (!DA.GetDataList(0, inputList)) return;
            if (!DA.GetData(1, ref vector)) return;

            // 結果を格納するリスト
            List<GH_CutMesh> outputList = new List<GH_CutMesh>();

            foreach (object input in inputList)
            {
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
                        // 元のデータを書き換えないようクローンを作成
                        cutMesh = cm.Clone();
                    }
                }

                if (cutMesh != null)
                {
                    // 頂点の移動処理
                    for (int i = 0; i < cutMesh.Vertices.Count; i++)
                    {
                        cutMesh.Vertices[i] += vector;
                    }

                    // 結果リストに追加
                    outputList.Add(new GH_CutMesh(cutMesh));
                }
                else
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Mesh または CutMesh ではないオブジェクトが含まれていました。");
                }
            }

            // まとめて出力
            DA.SetDataList(0, outputList);
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
            get { return new Guid("58B1C16D-C7D8-4C5C-8000-E76B5AC479A0"); }
        }
    }
}