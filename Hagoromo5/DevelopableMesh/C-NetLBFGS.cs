using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Hagoromo.GeometryTools;
using Hagoromo.MathTools;
using Rhino.Geometry;
using Rhino.Input.Custom;
using System;
using System.Collections.Generic;
using System.Linq;
using static Hagoromo.GeometryTools.CutMeshCalcTools;

namespace Hagoromo.DevelopableMesh
{
    public class NetLBFGS : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public NetLBFGS()
          : base("NetLBFGS", "NetLBFGS",
              "TutteMeshのあと長さ保存、符号付面積>0制約最適化。最初falseでやってw調整しておおまかOKになったらそのwのままtrueにするといい展開図得られる。",
              "Hagoromo", "Optimization")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "Mesh or CutMesh", GH_ParamAccess.item);
            pManager.AddCurveParameter("Feature Lines (Polylines)", "L", "Lines", GH_ParamAccess.list);
            pManager[1].Optional = true;
            pManager.AddBooleanParameter("has hole", "Hole", "having hole -> true", GH_ParamAccess.item, false);
            pManager[2].Optional = true;
            pManager.AddNumberParameter("w0", "w0", "length preservation weight", GH_ParamAccess.item, 1.0);
            pManager.AddNumberParameter("w1", "w1", "avoid face flip weight", GH_ParamAccess.item, 1.0);
            pManager.AddBooleanParameter("switch", "s", "最後にもう一度長さ保存最適化しますか？", GH_ParamAccess.item, false);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("cutMesh", "CM", "CM", GH_ParamAccess.item);
            pManager.AddCurveParameter("Feature Lines (Polylines)", "L", "Lines", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool hole = false;
            DA.GetData(2, ref hole);
            double[] w = new double[] { 1.0, 1.0 };
            DA.GetData(3, ref w[0]);
            DA.GetData(4, ref w[1]);

            object input = null;
            CutMesh cutMesh = new CutMesh();
            if (!DA.GetData(0, ref input)) return;
            if (!(input is IGH_Goo goo2)) return;
            if (input is IGH_Goo goo)
            {
                // Mesh へのキャストを試す
                if (goo.CastTo(out Rhino.Geometry.Mesh m))
                {
                    cutMesh = new CutMesh(m);
                    cutMesh = cutMesh.Sort();
                }
                else if (goo.CastTo(out CutMesh mesh2))
                {
                    cutMesh = mesh2.Sort();
                }
                else { return; }
            }

            List<Curve> curves = new List<Curve>();
            DA.GetDataList(1, curves);
            List<int> edgeIndices = CrvToEdgeIndices(cutMesh, curves);
            CutMesh newMesh = cutMesh.Clone();
            List<Line> lines = new List<Line>();
            double[] initialLength = EdgeLength(cutMesh);
            Vector3d[] normals = FaceNormal(cutMesh);
            double[] initialAbsArea = new double[normals.Length];
            for (int i = 0; i < normals.Length; i++)
            {
                initialAbsArea[i] = normals[i].Length;
            }


            if (!hole)
            {
                cutMesh = cutMesh.Sort();
                newMesh.Vertices = NetTools.NetBFF(cutMesh);
                newMesh.DuplicatedVertIndices = Enumerable.Range(0, cutMesh.Vertices.Count)
                              .Select(i => new List<int> { i })
                              .ToList();
                foreach (int edgeIndex in edgeIndices)
                {
                    lines.Add(newMesh.GetEdgeLine(edgeIndex));
                }
                DA.SetData(0, new GH_CutMesh(newMesh));
                DA.SetDataList(1, lines);
                return;
            }

            Mesh mesh = cutMesh.ConvertToMesh2();
            Polyline[] nakedEdges = mesh.GetNakedEdges();
            if (nakedEdges == null || nakedEdges.Length == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Mesh has no naked edges.");
                return;
            }
            Polyline outline = nakedEdges[0];
            double[][] newTopoVertices2D = NetTools.TutteTopoVertices(mesh, outline);
            Point3d[] newTopoVertices = PtCrvTools.Convert2Dto3D(newTopoVertices2D);

            //更新後のメッシュを作成
            Rhino.Geometry.Mesh tutteMesh = MeshDataTools.MakeMesh(mesh, newTopoVertices);
            for (int i = 0; i < newMesh.DuplicatedVertIndices.Count; i++)
            {
                List<int> vertGroup = newMesh.DuplicatedVertIndices[i];
                foreach (int vert in vertGroup)
                {
                    newMesh.Vertices[vert] = tutteMesh.Vertices[i];
                }
            }

            //newMesh.Vertices = NetTools.NetBFF2(cutMesh,outerBoundaryVertCount);
            newMesh.DuplicatedVertIndices = Enumerable.Range(0, cutMesh.Vertices.Count)
                              .Select(i => new List<int> { i })
                              .ToList();

            //----------------------------------------------以下newMeshに対して長さ最適化かつ裏返り防止LBFGS-----------------------------
            int vertCount = newMesh.Vertices.Count;
            List<Point3d> points = newMesh.Vertices;
            int variableCount = vertCount * 2;
            var system = new SimulationSystem(variableCount);

            // 初期座標のセット
            for (int i =0; i < vertCount; i++)
            {
                system.X[2 * i] = points[i].X;
                system.X[2 * i + 1] = points[i].Y;
            }

            // データの準備
            List<int[]> edges = newMesh.Edges;
            int[,] facesWithOrder = FacesWithOrder(newMesh);

            // --- 項目の追加 ---

            //----------------------------------等式の制約条件はこのように加える-----------------------------------------
            // 1. 長さ保存 (重み: 1.0)
            system.AddEnergy(new EdgeLengthEnergy2D(w[0], edges, initialLength));

            //system.AddEnergy(new AreaEnergy2D(w[1],facesWithOrder,initialAbsArea));


            //----------------------------不等式の制約条件はこのように加える-----------------------------------------------

            var areaConstraint = new AreaBarrier(w[1], facesWithOrder);

            // 同じものを「エネルギー」としても「制約」としても登録する
            system.AddEnergy(areaConstraint);
            system.AddConstraint(areaConstraint);
            //----------------------------不等式の制約条件はこの二つセットで加える 終-----------------------------------------------

            system.Step(10000);

            bool flag = false;
            DA.GetData(5, ref flag);

            if (!flag)
            {
                //求めたxをnewMeshのverticesに適用する
                for (int i = 0; i < vertCount; i++)
                {
                    newMesh.Vertices[i] = new Point3d(system.X[2 * i], system.X[2 * i + 1], 0);
                }

                foreach (int edgeIndex in edgeIndices)
                {
                    lines.Add(newMesh.GetEdgeLine(edgeIndex));
                }
                DA.SetData(0, new GH_CutMesh(newMesh));
                DA.SetDataList(1, lines);
                return;
            }

            //ほとんど長さ保存ができているのでこの状態でさらに長さ最適化を行って微調整を行う
            var system2 = new SimulationSystem(variableCount);
            for (int i = 0; i < variableCount; i++)
            {
                system2.X[i] = system.X[i];
            }
            system2.AddEnergy(new EdgeLengthEnergy2D(1, edges, initialLength));
            system2.Step();


            //求めたxをnewMeshのverticesに適用する
            for (int i = 0; i < vertCount; i++)
            {
                newMesh.Vertices[i] = new Point3d(system2.X[2 * i], system2.X[2 * i + 1], 0);
            }

            foreach (int edgeIndex in edgeIndices)
            {
                lines.Add(newMesh.GetEdgeLine(edgeIndex));
            }
            DA.SetData(0, new GH_CutMesh(newMesh));
            DA.SetDataList(1, lines);
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
            get { return new Guid("B19525BE-5F3F-4526-A249-DE649BBAE8EF"); }
        }
    }
}