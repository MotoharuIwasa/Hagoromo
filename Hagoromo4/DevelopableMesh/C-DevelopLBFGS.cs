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
    public class DevelopLBFGS : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public DevelopLBFGS()
          : base("DevelopLBFGS", "DevelopLBFGS",
              "Develop by LBFGS",
              "Hagoromo", "Optimization")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated CutMesh", "(C)M", "mesh to develop", GH_ParamAccess.item);
            pManager.AddIntegerParameter("iterations", "i", "iteration", GH_ParamAccess.item);
            pManager.AddBooleanParameter("xyMirror", "xy", "xyMirror", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("yzMirror", "yz", "yzMirror", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("zxMirror", "zx", "zxMirror", GH_ParamAccess.item, false);
            pManager.AddCurveParameter("outerBoundaryCurves", "outerB", "outerBoundaryCurves", GH_ParamAccess.list);
            int fixCrvIndex = pManager.AddCurveParameter("fixPointsCrv", "fixCrv", "fixPointsCrv", GH_ParamAccess.list);
            pManager[6].Optional = true;
            int unSmoothIndex = pManager.AddCurveParameter("unSmoothCrv", "unSmoothCrv", "unSmoothCrv", GH_ParamAccess.list);
            pManager[7].Optional = true;
            pManager.AddNumberParameter("edgeW", "edgeW", "edge energy weight", GH_ParamAccess.item, 1);
            pManager.AddNumberParameter("smoothW", "smoothW", "smoothing weight", GH_ParamAccess.item, 1);
            pManager.AddNumberParameter("devW", "devW", "developable energy weight", GH_ParamAccess.item, 1);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            //pManager.AddMeshParameter("Developed Mesh", "M", "developed planer mesh", GH_ParamAccess.item);
            pManager.AddGenericParameter("Modified CutMesh", "(C)M", "modified CutMesh", GH_ParamAccess.item);
            pManager.AddNumberParameter("DevEnergy", "DevEnergy", "DevEnergy", GH_ParamAccess.item);
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
            bool xyMirror = false;
            DA.GetData(2, ref xyMirror);
            bool yzMirror = false;
            DA.GetData(3, ref yzMirror);
            bool zxMirror = false;
            DA.GetData(4, ref zxMirror);
            double edgeW = 1;
            DA.GetData(8, ref edgeW);
            double smoothW = 1;
            DA.GetData(9, ref smoothW);
            double devW = 1;
            DA.GetData(10, ref devW);

            List<Curve> outerCrvs = new List<Curve>();
            DA.GetDataList(5, outerCrvs);
            List<int> sortedOuterVertIndices = CrvToVertIndices(cutMesh, outerCrvs);

            List<Curve> fixCrvs = new List<Curve>();
            bool hasInput = DA.GetDataList(6, fixCrvs);
            List<int> sortedFixVertIndices = CrvToVertIndices(cutMesh, fixCrvs);

            List<Curve> unSmoothCrvs = new List<Curve>();
            bool hasInput2 = DA.GetDataList(7, fixCrvs);
            List<int> sortedUnSmoothVertIndices = CrvToVertIndices(cutMesh, unSmoothCrvs);

            // 以降 cutMesh が確実に利用可能

            //----------------------------------------------以下cutMeshに対してDevelopable最適化-----------------------------
            List<List<int>> duplicatedVertIndices = cutMesh.DuplicatedVertIndices;
            int dupVertCount = duplicatedVertIndices.Count;
            int variableCount = dupVertCount * 3;
            var system = new SimulationSystem(variableCount);

            // 初期座標のセット
            for (int i = 0; i < dupVertCount; i++)
            {
                Point3d point = cutMesh.Vertices[duplicatedVertIndices[i][0]];
                system.X[3 * i] = point.X;
                system.X[3 * i + 1] = point.Y;
                system.X[3 * i + 2] = point.Z;
            }

            // データの準備
            int[] vertOrderInDup = cutMesh.VertOrderInDup();
            List<List<int>> dupConnectedVertIndices = cutMesh.DupConnectedVertIndices();
            List<List<List<int>>> category = CategolizeCutMesh(cutMesh, xyMirror, yzMirror, zxMirror, sortedOuterVertIndices, sortedFixVertIndices);
            List<bool[]> moveList = CategoryToMoveList(cutMesh.Vertices.Count, category, sortedFixVertIndices);
            HashSet<int> unMoveSet = MoveListToUnMoveSet(moveList,vertOrderInDup);

            HashSet<int> unMoveSetSmooth = new HashSet<int>(unMoveSet);
            foreach (int i in sortedUnSmoothVertIndices)
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


            double[] w = new double[] { 10.0, 30.0, 1.0, 1.0, 1.0, 1.0};

            //----------------------------------等式の制約条件はこのように加える-----------------------------------------

            //system.AddEnergy(new EdgeLengthEnergy3D(edgeW, cullDupEdges, vertOrderInDup, initialLength, unMoveSet));

            var devTerm = new DevelopableConsiderOtherEnergy(devW, cutMesh, category, aveLengthDone, aveLengthF, w, moveList, DoneMaps, FMaps);
            system.AddEnergy(devTerm);

            //system.AddEnergy(new SmoothingEnergy(smoothW, dupConnectedVertIndices, unMoveSetSmooth));

            //----------------------------------等式の制約条件はこのように加える 終-----------------------------------------

            system.Step(iterations);
            //求めたxをcutMeshのverticesに適用する
            for (int i = 0; i < dupVertCount; i++)
            {
                List<int> vertGroup = duplicatedVertIndices[i];
                Point3d point = new Point3d(system.X[3 * i], system.X[3 * i + 1], system.X[3 * i + 2]);
                foreach (int vertIndex in vertGroup)
                {
                    cutMesh.Vertices[vertIndex]  = point;
                }
            }

            double finalDevEnergy = devTerm.GetEnergy(system.X);

            DA.SetData(0, new GH_CutMesh(cutMesh));
            DA.SetData(1, finalDevEnergy);
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
            get { return new Guid("D3CC6C56-2131-48B1-BDF8-30061FAC7A53"); }
        }
    }
}