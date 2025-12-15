using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Hagoromo.GeometryTools;
using Hagoromo.MathTools;
using MathNet.Numerics.Random;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Collections;
using Rhino.Render.ChangeQueue;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Hagoromo.DevelopableMesh
{
    public class CutChoicePrincipleNoLoop : GH_Component
    {
        public CutChoicePrincipleNoLoop()
          : base("CutChoicePrincipleNoLoop", "PCNoLoop",
              "CutChoicePrincipleNoLoop",
              "Hagoromo", "Optimization")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated CutMesh", "(C)M", "mesh to develop", GH_ParamAccess.item);
            pManager.AddNumberParameter("maxLength", "L", "max length", GH_ParamAccess.list);
            pManager.AddIntegerParameter("seedCount", "SN", "seed count", GH_ParamAccess.item);
            pManager.AddNumberParameter("distance", "D", "distance", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Branch", "Branch", "branch", GH_ParamAccess.item, false);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated CutMesh", "(C)M", "mesh to develop", GH_ParamAccess.item);
            pManager.AddCurveParameter("Cut Choices", "CL", "cut edges", GH_ParamAccess.tree);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            object input = null;
            if (!DA.GetData(0, ref input)) return;

            CutMesh cutMesh = null;

            if (input is IGH_Goo goo)
            {
                // Mesh へのキャストを試す
                if (goo.CastTo(out Rhino.Geometry.Mesh m))
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
            cutMesh = cutMesh.Sort();

            int seedCount = 0;
            if (!DA.GetData(2, ref seedCount)) return;
            List<double> maxLength2 = new List<double>();
            if (!DA.GetDataList(1, maxLength2)) return;
            double[] maxLength = maxLength2.ToArray();
            double distance = 0;
            if (!DA.GetData(3, ref distance)) return;
            bool branch = false;
            if (!DA.GetData(4, ref branch)) return;

            List<int>[] cutChoices = new List<int>[seedCount];
            if (branch)
            {
                cutChoices = CutChoiceTools.AvoidLoopMultiBranchesMultiSeedsNoMerge(cutMesh, maxLength, seedCount, distance);
            }
            else
            {
                cutChoices = CutChoiceTools.AvoidLoopOneBranchMultiSeeds(cutMesh, maxLength, seedCount, distance);
            }

            List<int> flattened = cutChoices.SelectMany(list => list).ToList();
            CutMesh newMesh = MeshCutTools.CutMeshWithEdgeIndices(cutMesh, flattened);
            DA.SetData(0, new GH_CutMesh(newMesh));

            GH_Structure<GH_Line> edgeTree = new GH_Structure<GH_Line>();

            // 2. 配列の要素数（seedCount）分だけループ
            for (int i = 0; i < cutChoices.Length; i++)
            {
                // パス（枝）を作成： {0}, {1}, {2}... となります
                GH_Path path = new GH_Path(i);

                // そのシードに対応するエッジインデックスのリストを取得
                List<int> currentBranchIndices = cutChoices[i];

                // リストの中身を回してLineを取得・追加
                if (currentBranchIndices != null)
                {
                    foreach (int edgeIndex in currentBranchIndices)
                    {
                        // インデックスからLine形状を取得
                        Line line = cutMesh.GetEdgeLine(edgeIndex);

                        // GH_Lineに変換して、指定したパス(path)に追加
                        edgeTree.Append(new GH_Line(line), path);
                    }
                }
            }

            // 3. ツリーとして出力
            DA.SetDataTree(1, edgeTree);
            //DA.SetDataList(1, cutEdges);
        }

        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return null;
            }
        }

        public override Guid ComponentGuid
        {
            get { return new Guid("3BF2CF9F-35A0-452A-A39E-E73606360E0C"); }
        }
    }
}