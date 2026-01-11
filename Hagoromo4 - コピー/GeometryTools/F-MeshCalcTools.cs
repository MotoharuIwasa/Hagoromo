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
    public static class MeshCalcTools
    {
        //元のメッシュからTopologyVerticesのindex番目の点を移動した後のメッシュを返す
        public static Rhino.Geometry.Mesh DeltaMesh(Rhino.Geometry.Mesh mesh, int TopoVertIndex, float deltaX, float deltaY, float deltaZ)
        {
            Rhino.Geometry.Mesh deltaMesh = mesh.DuplicateMesh();
            int[] vertexIndices = deltaMesh.TopologyVertices.MeshVertexIndices(TopoVertIndex);

            // 各頂点を移動（通常は1つだけだが、複数ある可能性もある）
            foreach (int vi in vertexIndices)
            {
                Point3f pt = deltaMesh.Vertices[vi];
                pt.X += deltaX;
                pt.Y += deltaY;
                pt.Z += deltaZ;
                deltaMesh.Vertices[vi] = pt;
            }
            return deltaMesh;
        }

        //3Dのメッシュの各辺の長さを求める。
        public static double[][] GetMeshEdgesLength(int[][] connectedVertices, Point3d[] newTopoVertices)
        {
            double[][] edgeLength = new double[connectedVertices.Length][];
            for (int i = 0; i < connectedVertices.Length; i++)
            {
                edgeLength[i] = new double[connectedVertices[i].Length];
                for (int j = 0; j < connectedVertices[i].Length; j++)
                {
                    edgeLength[i][j] = newTopoVertices[i].DistanceTo(newTopoVertices[connectedVertices[i][j]]);
                }
            }
            return edgeLength;
        }

        //2Dのメッシュの各辺の長さを求める。
        public static double[][] GetMeshEdgesLength(int[][] connectedVertices, double[][] NewTopoVertices2D)
        {
            double[][] edgeLength = new double[connectedVertices.Length][];
            for (int i = 0; i < connectedVertices.Length; i++)
            {
                edgeLength[i] = new double[connectedVertices[i].Length];
                for (int j = 0; j < connectedVertices[i].Length; j++)
                {
                    edgeLength[i][j] = Math.Sqrt((NewTopoVertices2D[i][0] - NewTopoVertices2D[connectedVertices[i][j]][0]) *
                        (NewTopoVertices2D[i][0] - NewTopoVertices2D[connectedVertices[i][j]][0]) +
                        (NewTopoVertices2D[i][1] - NewTopoVertices2D[connectedVertices[i][j]][1]) *
                        (NewTopoVertices2D[i][1] - NewTopoVertices2D[connectedVertices[i][j]][1]));
                }
            }
            return edgeLength;
        }

        //元のメッシュでのそれぞれのfaceにおける点A,Bでのsin,cosを求める(反時計回り正)
        public static double[][][] GetMeshSignAngle(Rhino.Geometry.Mesh mesh)
        {
            int faceCount = mesh.Faces.Count;
            double[][][] initilAngle = new double[faceCount][][];

            for (int i = 0; i < faceCount; i++)
            {
                initilAngle[i] = new double[2][];  // 頂点A,Bだけ
                initilAngle[i][0] = new double[2]; // A: cos,sin
                initilAngle[i][1] = new double[2]; // B: cos,sin

                // faceの頂点インデックス
                int idxA = mesh.Faces[i].A;
                int idxB = mesh.Faces[i].B;
                int idxC = mesh.Faces[i].C;

                Point3d A = mesh.Vertices[idxA];
                Point3d B = mesh.Vertices[idxB];
                Point3d C = mesh.Vertices[idxC];

                // --- 法線ベクトル ---
                Vector3d normal = Vector3d.CrossProduct(B - A, C - A);

                // --- 頂点A ---
                Vector3d uA = B - A;
                Vector3d vA = C - A;
                double cosA = uA * vA / (uA.Length * vA.Length);

                Vector3d crossA = Vector3d.CrossProduct(uA, vA);
                double sinA = crossA.Length / (uA.Length * vA.Length);
                sinA *= Math.Sign(normal * crossA); // 符号付き

                initilAngle[i][0][0] = cosA;
                initilAngle[i][0][1] = sinA;

                // --- 頂点B ---
                Vector3d uB = C - B;
                Vector3d vB = A - B;
                double cosB = uB * vB / (uB.Length * vB.Length);

                Vector3d crossB = Vector3d.CrossProduct(uB, vB);
                double sinB = crossB.Length / (uB.Length * vB.Length);
                sinB *= Math.Sign(normal * crossB);

                initilAngle[i][1][0] = cosB;
                initilAngle[i][1][1] = sinB;
            }

            return initilAngle;
        } 

        /// Faceの重心座標を返す
        public static Point3d FaceCentroid(Rhino.Geometry.Mesh mesh, int faceIndex)
        {
            MeshFace f = mesh.Faces[faceIndex];

            if (f.IsTriangle)
            {
                Point3f A = mesh.Vertices[f.A];
                Point3f B = mesh.Vertices[f.B];
                Point3f C = mesh.Vertices[f.C];
                return new Point3d(
                    (A.X + B.X + C.X) / 3.0,
                    (A.Y + B.Y + C.Y) / 3.0,
                    (A.Z + B.Z + C.Z) / 3.0
                );
            }
            else
            {
                Point3f A = mesh.Vertices[f.A];
                Point3f B = mesh.Vertices[f.B];
                Point3f C = mesh.Vertices[f.C];
                Point3f D = mesh.Vertices[f.D];
                return new Point3d(
                    (A.X + B.X + C.X + D.X) / 4.0,
                    (A.Y + B.Y + C.Y + D.Y) / 4.0,
                    (A.Z + B.Z + C.Z + D.Z) / 4.0
                );
            }
        }

        //meshのある一点での離散ガウス曲率の2乗を算出
        public static double CurvatureTwo(Rhino.Geometry.Mesh mesh, int topoIndex)
        {
            double angleSum = 0.0;
            int[] faces = mesh.TopologyVertices.ConnectedFaces(topoIndex);

            foreach (int fi in faces)
            {
                MeshFace face = mesh.Faces[fi];
                List<int> indices = new List<int> { face.A, face.B, face.C };
                if (face.IsQuad) indices.Add(face.D);

                List<int> topoIndices = new List<int>();
                foreach (int idx in indices)
                {
                    topoIndices.Add(mesh.TopologyVertices.TopologyVertexIndex(idx));
                }

                int vertPos = topoIndices.IndexOf(topoIndex);
                if (vertPos == -1) continue;

                Point3f p0 = mesh.Vertices[indices[(vertPos - 1 + indices.Count) % indices.Count]];
                Point3f p1 = mesh.Vertices[indices[vertPos]];
                Point3f p2 = mesh.Vertices[indices[(vertPos + 1) % indices.Count]];

                Vector3d v1 = p0 - p1;
                Vector3d v2 = p2 - p1;

                double angle = Vector3d.VectorAngle(v1, v2);
                angleSum += angle;
            }
            return (2 * Math.PI - angleSum) * (2 * Math.PI - angleSum);
        }


        //最も曲率が大きい点のTopoVerticesでのindexを返す
        public static int MeshBiggestCurvatureIndex(Rhino.Geometry.Mesh mesh, int[] internalVertexIndices)
        {
            int index = 0;
            double curvature = 0;
            double biggest = 0;
            for (int i = 0; i < internalVertexIndices.Length; i++)
            {
                curvature = CurvatureTwo(mesh, internalVertexIndices[i]);
                if (curvature > biggest)
                {
                    biggest = curvature;
                    index = internalVertexIndices[i];
                }
            }
            return index;
        }

    }
}
