using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Hagoromo.GeometryTools;
using Hagoromo.MathTools;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Collections;
using Rhino.Render.ChangeQueue;
using System;
using System.Collections.Generic;
using System.Linq;
using static Hagoromo.GeometryTools.CutMeshCalcTools;
using static Hagoromo.DevelopableMesh.DevelopableTools;
using static Hagoromo.DevelopableMesh.CGNR3D;

namespace Hagoromo.DevelopableMesh
{
    public class DevelopCGLS : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public DevelopCGLS()
          : base("DevelopCGLS", "DevelopCGLS",
              "Develop by CGLS",
              "Hagoromo", "Optimization")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated CutMesh", "(C)M", "mesh to develop", GH_ParamAccess.item);
            pManager.AddIntegerParameter("innerIter", "inIter", "CG iterations", GH_ParamAccess.item, 100);
            pManager.AddIntegerParameter("outerIter", "outIter", "LS iterations", GH_ParamAccess.item, 30);
            pManager.AddBooleanParameter("xyMirror", "xy", "xyMirror", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("yzMirror", "yz", "yzMirror", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("zxMirror", "zx", "zxMirror", GH_ParamAccess.item, false);
            pManager.AddCurveParameter("outerBoundaryCurves", "outerB", "outerBoundaryCurves", GH_ParamAccess.list);
            int fixCrvIndex = pManager.AddCurveParameter("fixPointsCrv", "fixCrv", "fixPointsCrv", GH_ParamAccess.list);
            pManager[7].Optional = true;
            int unSmoothIndex = pManager.AddCurveParameter("unSmoothCrv", "unSmoothCrv", "unSmoothCrv", GH_ParamAccess.list);
            pManager[8].Optional = true;
            pManager.AddNumberParameter("edgeW", "edgeW", "edge energy weight", GH_ParamAccess.item, 1);
            pManager.AddNumberParameter("smoothW", "smoothW", "smoothing weight", GH_ParamAccess.item, 1);
            pManager.AddNumberParameter("devW", "devW", "developable energy weight", GH_ParamAccess.item, 1);
            pManager.AddNumberParameter("positionW", "posW", "position weight", GH_ParamAccess.item, 1);
            pManager.AddNumberParameter("lenRangeW", "lenRangeW", "length range weight", GH_ParamAccess.item, 1);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            //pManager.AddMeshParameter("Developed Mesh", "M", "developed planer mesh", GH_ParamAccess.item);
            pManager.AddGenericParameter("Modified CutMesh", "(C)M", "modified CutMesh", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
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
            int iterations = 0;
            DA.GetData(1, ref iterations);
            int outerIter = 0;
            DA.GetData(2, ref outerIter);
            bool xyMirror = false;
            DA.GetData(3, ref xyMirror);
            bool yzMirror = false;
            DA.GetData(4, ref yzMirror);
            bool zxMirror = false;
            DA.GetData(5, ref zxMirror);
            double edgeW = 1;
            DA.GetData(9, ref edgeW);
            double smoothW = 1;
            DA.GetData(10, ref smoothW);
            double devW = 1;
            DA.GetData(11, ref devW);
            double posW = 1;
            DA.GetData(12, ref posW);
            double lenRangeW = 1;
            DA.GetData(13, ref lenRangeW);

            List<Curve> outerCrvs = new List<Curve>();
            DA.GetDataList(6, outerCrvs);
            List<int> sortedOuterVertIndices = CrvToVertIndices(cutMesh, outerCrvs);

            List<Curve> fixCrvs = new List<Curve>();
            bool hasInput = DA.GetDataList(7, fixCrvs);
            List<int> sortedFixVertIndices = CrvToVertIndices(cutMesh, fixCrvs);

            List<Curve> unSmoothCrvs = new List<Curve>();
            bool hasInput2 = DA.GetDataList(8, fixCrvs);
            List<int> sortedUnSmoothVertIndices = CrvToVertIndices(cutMesh, unSmoothCrvs);

            // 以降 cutMesh が確実に利用可能




            //----------------------------------------------以下cutMeshに対してDevelopable最適化-----------------------------

            //--------------------------------------------------データの準備-------------------------------------------------
            List<List<int>> duplicatedVertIndices = cutMesh.DuplicatedVertIndices;
            int dupVertCount = duplicatedVertIndices.Count;
            int variableCount = dupVertCount * 3;

            int[] vertOrderInDup = cutMesh.VertOrderInDup();
            List<List<int>> dupConnectedVertIndices = cutMesh.DupConnectedVertIndices();
            List<List<List<int>>> category = CategolizeCutMesh(cutMesh, xyMirror, yzMirror, zxMirror, sortedOuterVertIndices, sortedFixVertIndices);
            List<bool[]> moveList = CategoryToMoveList(cutMesh.Vertices.Count, category, sortedFixVertIndices);
            HashSet<int> unMoveSet = MoveListToUnMoveSet(moveList, vertOrderInDup);

            HashSet<int> unMoveSetSmooth = new HashSet<int>(unMoveSet);
            foreach (int i in sortedUnSmoothVertIndices)
            {
                int id = vertOrderInDup[i] * 3;
                unMoveSetSmooth.Add(id);
                unMoveSetSmooth.Add(id + 1);
                unMoveSetSmooth.Add(id + 2);
            }
            List<int> boundaryVerts = cutMesh.BoundaryVertIndices();
            foreach (int i in boundaryVerts)
            {
                int id = vertOrderInDup[i] * 3;
                unMoveSetSmooth.Add(id);
                unMoveSetSmooth.Add(id + 1);
                unMoveSetSmooth.Add(id + 2);
            }



            var loopLength = CalcLoopLength(cutMesh, category[5], category[9]);
            List<double> aveLengthDone = loopLength.DoneAveLength;
            List<double> aveLengthF = loopLength.FblocksAveLength;

            List<Dictionary<int, List<int>>> DoneMaps = MakeMaps(cutMesh, category[1], vertOrderInDup);
            List<Dictionary<int, List<int>>> FMaps = MakeMaps(cutMesh, category[9], vertOrderInDup);

            List<int[]> cullDupEdges = CullDupEdges(cutMesh, vertOrderInDup);
            double[] initialLength = new double[cullDupEdges.Count];
            for (int i = 0; i < cullDupEdges.Count; i++)
            {
                int[] edge = cullDupEdges[i];
                Point3d p0 = cutMesh.Vertices[edge[0]];
                Point3d p1 = cutMesh.Vertices[edge[1]];
                initialLength[i] = p0.DistanceTo(p1);
            }

            //w1～w6の設定
            double[] w = new double[] { 100.0, 100.0, 100, 100.0, 100.0, 100.0 };
            //double[] w = new double[] { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
            //--------------------------------------------------データの準備 終-------------------------------------------------


            //最適化するものの箱を準備
            var optimizer = new GaussNewtonOptimizer(variableCount);

            List<Point3d> dupInitialPosition = new List<Point3d>();
            // 初期座標のセット
            for (int i = 0; i < dupVertCount; i++)
            {
                Point3d point = cutMesh.Vertices[duplicatedVertIndices[i][0]];
                dupInitialPosition.Add(point);
                optimizer.X[3 * i] = point.X;
                optimizer.X[3 * i + 1] = point.Y;
                optimizer.X[3 * i + 2] = point.Z;
            }

            // ------------------------等式制約の追加-------------------------------------------

            optimizer.AddTerm(new MinMaxLengthCGLS3D(lenRangeW, cullDupEdges, vertOrderInDup, initialLength, unMoveSet));

            optimizer.AddTerm(new EdgeLengthCGLS3D(edgeW, cullDupEdges, vertOrderInDup, initialLength, unMoveSet));

            var devTerm = new DevelopableCGLS(devW, cutMesh, category, aveLengthDone, aveLengthF, w, moveList, DoneMaps, FMaps);
            optimizer.AddTerm(devTerm);

            optimizer.AddTerm(new SmoothingCGLS(smoothW, dupConnectedVertIndices, unMoveSetSmooth));

            optimizer.AddTerm(new VertMoveCGLS(posW, dupInitialPosition, unMoveSet));
            // ----不等式制約の追加（別に等式と特段変わりはない、その制約のクラス内で条件満たすかどうかでの処理を書くかどうかの違い）---

            //optimizer.AddTerm(new MinLengthTerm(100.0, cullDupEdges, 15, vertOrderInDup));


            //計算実行
            optimizer.Step(outerIter: outerIter, innerIter: iterations);
            //optimizer.Step(outerIter: 1, innerIter: 1);

            //求めたxをcutMeshのverticesに適用する
            for (int i = 0; i < dupVertCount; i++)
            {
                List<int> vertGroup = duplicatedVertIndices[i];
                Point3d point = new Point3d(optimizer.X[3 * i], optimizer.X[3 * i + 1], optimizer.X[3 * i + 2]);
                foreach (int vertIndex in vertGroup)
                {
                    cutMesh.Vertices[vertIndex] = point;
                }
            }

            DA.SetData(0, new GH_CutMesh(cutMesh));
        }

        /// <summary>
        /// Provides an Icon for the component.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                //You can add image files to your project resources and access them like this:
                // return Resources.IconForThisComponent;
                return null;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("5ADCF8A8-6258-4086-959A-15DDFE07F4D8"); }
        }
    }
}