using Grasshopper.Kernel;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Collections;
using Rhino.Render.ChangeQueue;
using System;
using System.Collections.Generic;
using System.Linq;
using Hagoromo.MathTools;

namespace Hagoromo.GeometryTools
{
    public static class CutMeshCalcTools
    {
        //interiorAngles[i][j]はi番目の点の周りのGetOrderedFacesAroundVertexの順でj番目のfaceにおける内角
        public static List<List<double>> InteriorAngles(CutMesh mesh)
        {
            List<List<double>> angles = new List<List<double>>();
            for (int i = 0; i < mesh.Vertices.Count; i++)
            {
                List<int> faces = mesh.GetFacesForVertex(i);
                List<double> angleList = new List<double>();
                for (int j = 0; j < faces.Count; j++)
                {
                    List<int> a = new List<int> { mesh.Faces[faces[j], 0], mesh.Faces[faces[j], 1], mesh.Faces[faces[j], 2] };

                    List<int> points = a.Where(x => x != i).ToList();
                    Point3d A = mesh.Vertices[i];
                    Vector3d AB = mesh.Vertices[points[0]] - A;
                    Vector3d AC = mesh.Vertices[points[1]] - A;
                    double angle = Vector3d.VectorAngle(AB, AC); // ラジアンで返る
                    angleList.Add(angle);
                }
                angles.Add(angleList);
            }
            return angles;
        }

        //ガウス曲率のリスト、境界上の点ではガウス曲率0とする。
        public static double[] GaussianCurvature(CutMesh mesh)
        {
            List<int> boundaryVertIndices = mesh.BoundaryVertIndices();
            List<List<double>> angles = InteriorAngles(mesh);
            double[] gaussian = new double[mesh.Vertices.Count];
            for (int i = 0; i < mesh.Vertices.Count; i++)
            {
                if (boundaryVertIndices.Contains(i))
                {
                    continue;
                }
                else
                {
                    List<double> angle = angles[i];
                    double a = 0;
                    for (int j = 0; j < angle.Count; j++)
                    {
                        a += angle[j];
                    }
                    gaussian[i] = 2 * Math.PI - a;
                }
            }
            return gaussian;
        }

        //測地曲率のリスト、境界上のみ定義する。BoundaryVertIndicesの順
        public static double[] GeodesicCurvature(CutMesh mesh)
        {
            List<int> boundaryVertIndices = mesh.BoundaryVertIndices();
            List<List<double>> angles = InteriorAngles(mesh);
            double[] geodesic = new double[boundaryVertIndices.Count];
            for (int i = 0; i < boundaryVertIndices.Count; i++)
            {
                double a = 0;
                List<double> angle = angles[boundaryVertIndices[i]];
                for (int j = 0; j < angle.Count; j++)
                {
                    a += angle[j];
                }
                geodesic[i] = Math.PI - a;
            }
            return geodesic;
        }

        public static int[][][] MakeInternalFaceID(CutMesh mesh)
        {
            List<int> boundaryVertIndices = mesh.BoundaryVertIndices();
            List<int> fullSet = Enumerable.Range(0, mesh.Vertices.Count).ToList();
            List<int> internalVertIndices = fullSet.Except(boundaryVertIndices).ToList();

            int[][][] faceID = new int[internalVertIndices.Count][][];
            for (int i = 0; i < internalVertIndices.Count; i++)
            {
                List<int> facesForVertex = mesh.GetFacesForVertex(internalVertIndices[i]);
                faceID[i] = new int[facesForVertex.Count][];
                for (int j =0; j < facesForVertex.Count; j++)
                {
                    faceID[i][j] = new int[2];
                    int[] facesVertices = new int[] { mesh.Faces[facesForVertex[j], 0], mesh.Faces[facesForVertex[j], 1], mesh.Faces[facesForVertex[j], 2] };
                    int[] twoVertices = facesVertices.Where(v => v != internalVertIndices[i]).ToArray();
                    faceID[i][j][0] = twoVertices[0];
                    faceID[i][j][1] = twoVertices[1];
                }
            }
            return faceID;
        }
    }
}