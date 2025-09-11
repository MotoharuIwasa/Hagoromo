using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace Hagoromo.GeometryTools
{
    public class MeshGraph : GH_Component
    {
        public MeshGraph()
          : base("MeshGraph", "MeshGraph",
              "Description",
              "Hagoromo", "Geometry")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Triangulated Mesh or CutMesh", "(C)M", "Mesh or CutMesh", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Mesh Graph", "T", "cut mesh tree", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            object input = null;
            if (!DA.GetData(0, ref input)) return;

            if (input is IGH_Goo goo)
            {
                // Mesh へのキャストを試す
                if (goo.CastTo(out Mesh mesh))
                {
                    int[][] connectedFaces = MeshDataTools.ConnectedFaces(mesh);

                    List<Curve> treeCurves = MeshCutTools.MeshTreeCentersToCurves(mesh, connectedFaces);
                    DA.SetDataList(0, treeCurves);
                }

                // CutMesh へのキャストを試す
                else if (goo.CastTo(out CutMesh cutMesh))
                {
                    //cutMeshに対する処理をする
                    int faceCount = cutMesh.Faces.GetLength(0);
                    Point3d[] faceCenters = new Point3d[faceCount];
                    // フェイス中心を計算
                    for (int i = 0; i < faceCount; i++)
                    {
                        faceCenters[i] = cutMesh.GetFaceCenter(i);
                    }

                    cutMesh.ReloadEdgeToFacesCache();
                    var curves = new List<Curve>();

                    for (int i = 0; i < faceCount; i++)
                    {
                        List<int> connectedFaces = cutMesh.ConnectedFacesForFace(i);
                        if (connectedFaces.Count == 0 || connectedFaces == null) continue;

                        foreach (int j in connectedFaces)
                        {
                            if (j < 0 || j >= faceCount) continue;

                            // 重複描画防止 (i<j のときだけ描画)
                            if (i > j) continue;

                            curves.Add(new Line(faceCenters[i], faceCenters[j]).ToNurbsCurve());
                        }
                    }
                    DA.SetDataList(0, curves);

                }
            }
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
            get { return new Guid("73CD1195-DC86-4B63-BB77-665D41928BD7"); }
        }
    }
}