using Grasshopper.Kernel;
using Hagoromo.GeometryTools;
using Hagoromo.MathTools;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Collections;
using Rhino.Render.ChangeQueue;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

namespace Hagoromo.DevelopableMesh
{
    public static class CurvatureTools
    {

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

        //元のメッシュからcurvatureの2乗の和をFとして最急降下法で最適化する際の次のステップのメッシュを得る。もう使ってない
        public static Rhino.Geometry.Mesh SDCrvNextMesh(Rhino.Geometry.Mesh mesh)
        {
            List<int> internalVertexIndices = MeshDataTools.TopoInternalVertIndices(mesh);

            //ガウス曲率の2乗のリストを作成
            List<double> crvSum = new List<double> { };
            int count = internalVertexIndices.Count;
            for (int i = 0; i < count; i++)
            {
                double curvatureTwo = CurvatureTwo(mesh, internalVertexIndices[i]);
                crvSum.Add(curvatureTwo);
            }
            double F = crvSum.Sum();

            double[] Jacobi = new double[3 * count];
            float epsilon = 1e-6f;
            for (int i = 0; i < count; i++)
            {
                Rhino.Geometry.Mesh deltaMesh = MeshCalcTools.DeltaMesh(mesh, internalVertexIndices[i], epsilon, 0, 0);
                Jacobi[i * 3] = (CurvatureTwo(deltaMesh, internalVertexIndices[i]) - crvSum[i]) / epsilon;
                deltaMesh = MeshCalcTools.DeltaMesh(mesh, internalVertexIndices[i], 0, epsilon, 0);
                Jacobi[i * 3 + 1] = (CurvatureTwo(deltaMesh, internalVertexIndices[i]) - crvSum[i]) / epsilon;
                deltaMesh = MeshCalcTools.DeltaMesh(mesh, internalVertexIndices[i], 0, 0, epsilon);
                Jacobi[i * 3 + 2] = (CurvatureTwo(deltaMesh, internalVertexIndices[i]) - crvSum[i]) / epsilon;
            }

            double[] A = MatrixUtils.GetLowerTriangleFromVector(Jacobi);
            double[] delta = MatrixUtils.Multiply(-0.1, Jacobi);
            Rhino.Geometry.Mesh nextMesh = mesh.DuplicateMesh();
            for (int i = 0; i < count; i++)
            {
                nextMesh = MeshCalcTools.DeltaMesh(nextMesh, internalVertexIndices[i], (float)delta[3 * i], (float)delta[3 * i + 1], (float)delta[3 * i + 2]);
            }
            return nextMesh;
        }

        //最急降下法で有限差分ではなく解析的に近似で解いた偏微分でヤコビを生成したバージョン。こっちのほうが性能良い。
        public static Rhino.Geometry.Mesh SDCrvNextMesh2(Rhino.Geometry.Mesh mesh)
        {
            List<int> internalVertexIndices = MeshDataTools.TopoInternalVertIndices(mesh);
            int count = internalVertexIndices.Count;
            double[] Jacobi = new double[3 * count];

            for (int i = 0; i < count; i++)
            {
                int[] faces = mesh.TopologyVertices.ConnectedFaces(internalVertexIndices[i]);
                double angleSum = 0;
                double deltaSumX = 0;
                double deltaSumY = 0;
                double deltaSumZ = 0;

                foreach (int fi in faces)
                {
                    MeshFace face = mesh.Faces[fi];
                    List<int> indices = new List<int> { face.A, face.B, face.C };

                    List<int> topoIndices = new List<int>();
                    foreach (int idx in indices)
                    {
                        topoIndices.Add(mesh.TopologyVertices.TopologyVertexIndex(idx));
                    }

                    int vertPos = topoIndices.IndexOf(internalVertexIndices[i]);
                    if (vertPos == -1) continue;

                    Point3f p0 = mesh.Vertices[indices[(vertPos - 1 + indices.Count) % indices.Count]];
                    Point3f p1 = mesh.Vertices[indices[vertPos]];
                    Point3f p2 = mesh.Vertices[indices[(vertPos + 1) % indices.Count]];

                    Vector3d v1 = p0 - p1;
                    Vector3d v2 = p2 - p1;
                    double nv1 = v1.Length;
                    double nv2 = v2.Length;

                    double angle = Vector3d.VectorAngle(v1, v2);
                    double ABsinTheta = nv1 * nv2 * Math.Sin(angle);
                    double k1 = 1 - nv2 * Math.Cos(angle) / nv1;
                    double k2 = 1 - nv1 * Math.Cos(angle) / nv2;
                    double k3 = k1 + k2;
                    deltaSumX += (k1 * p0.X + k2 * p2.X - k3 * p1.X) / ABsinTheta;
                    deltaSumY += (k1 * p0.Y + k2 * p2.Y - k3 * p1.Y) / ABsinTheta;
                    deltaSumZ += (k1 * p0.Z + k2 * p2.Z - k3 * p1.Z) / ABsinTheta;
                    angleSum += angle;
                }
                Jacobi[i * 3] = (2 * angleSum - 4 * Math.PI) * deltaSumX;
                Jacobi[i * 3 + 1] = (2 * angleSum - 4 * Math.PI) * deltaSumY;
                Jacobi[i * 3 + 2] = (2 * angleSum - 4 * Math.PI) * deltaSumZ;
            }

            double[] delta = MatrixUtils.Multiply(-0.1, Jacobi);
            Rhino.Geometry.Mesh nextMesh = mesh.DuplicateMesh();
            for (int i = 0; i < count; i++)
            {
                nextMesh = MeshCalcTools.DeltaMesh(nextMesh, internalVertexIndices[i], (float)delta[3 * i], (float)delta[3 * i + 1], (float)delta[3 * i + 2]);
            }
            return nextMesh;
        }

        //各ステップでメッシュを介するのを省略する。これがSD関係のもので一番性能がいい。
        public static Point3d[] SDnewTopoV(Point3d[] newTopoVertices, List<int> internalVertexIndices, int[][][] TriFaceID)
        {
            int count = internalVertexIndices.Count;
            Vector3d[] Jacobi = new Vector3d[count];
            int[][] faces = new int[][] { };

            //i番目の点について
            for (int i = 0; i < count; i++)
            {
                double angleSum = 0;
                double deltaSumX = 0;
                double deltaSumY = 0;
                double deltaSumZ = 0;
                Point3d p1 = newTopoVertices[internalVertexIndices[i]];

                //i番目の点周りにあるfaceのそれぞれの面に対して
                for (int j = 0; j < TriFaceID[i].Length; j++)
                {
                    Point3d p0 = newTopoVertices[TriFaceID[i][j][0]];
                    Point3d p2 = newTopoVertices[TriFaceID[i][j][1]];

                    Vector3d v1 = p0 - p1;
                    Vector3d v2 = p2 - p1;
                    double nv1 = v1.Length;
                    double nv2 = v2.Length;

                    double angle = Vector3d.VectorAngle(v1, v2);
                    double ABsinTheta = nv1 * nv2 * Math.Sin(angle);
                    double k1 = 1 - nv2 * Math.Cos(angle) / nv1;
                    double k2 = 1 - nv1 * Math.Cos(angle) / nv2;
                    double k3 = k1 + k2;

                    deltaSumX += (k1 * p0.X + k2 * p2.X - k3 * p1.X) / ABsinTheta;
                    deltaSumY += (k1 * p0.Y + k2 * p2.Y - k3 * p1.Y) / ABsinTheta;
                    deltaSumZ += (k1 * p0.Z + k2 * p2.Z - k3 * p1.Z) / ABsinTheta;
                    angleSum += angle;
                }
                double s = 2 * angleSum - 4 * Math.PI;
                Jacobi[i] = new Vector3d(s * deltaSumX, s * deltaSumY, s * deltaSumZ);
            }

            //係数αは0.1としている
            for (int i = 0; i < count; i++)
            {
                newTopoVertices[internalVertexIndices[i]] -= 1 * Jacobi[i];
            }

            return newTopoVertices;
        }

        //共役勾配法で次のステップのメッシュを得る。
        public static void CGCrvNextMesh(Point3d[] newTopoVertices, Vector3d[] Jacobi, Vector3d[] p, List<int> internalVertexIndices, int[][][] TriFaceID)
        {
            int count = internalVertexIndices.Count;
            //int[][] faces = new int[][] { };
            Vector3d[] nextJacobi = new Vector3d[count];
            Vector3d[] nextp = new Vector3d[count];

            //i番目の点について
            for (int i = 0; i < count; i++)
            {
                double angleSum = 0;
                double deltaSumX = 0;
                double deltaSumY = 0;
                double deltaSumZ = 0;
                Point3d p1 = newTopoVertices[internalVertexIndices[i]];

                //i番目の点周りにあるfaceのそれぞれの面に対して
                for (int j = 0; j < TriFaceID[i].Length; j++)
                {
                    Point3d p0 = newTopoVertices[TriFaceID[i][j][0]];
                    Point3d p2 = newTopoVertices[TriFaceID[i][j][1]];

                    Vector3d v1 = p0 - p1;
                    Vector3d v2 = p2 - p1;
                    double nv1 = v1.Length;
                    double nv2 = v2.Length;

                    double angle = Vector3d.VectorAngle(v1, v2);
                    double ABsinTheta = nv1 * nv2 * Math.Sin(angle);
                    double k1 = 1 - nv2 * Math.Cos(angle) / nv1;
                    double k2 = 1 - nv1 * Math.Cos(angle) / nv2;
                    double k3 = k1 + k2;

                    deltaSumX += (k1 * p0.X + k2 * p2.X - k3 * p1.X) / ABsinTheta;
                    deltaSumY += (k1 * p0.Y + k2 * p2.Y - k3 * p1.Y) / ABsinTheta;
                    deltaSumZ += (k1 * p0.Z + k2 * p2.Z - k3 * p1.Z) / ABsinTheta;
                    angleSum += angle;
                }
                double s = 2 * angleSum - 4 * Math.PI;

                nextJacobi[i] = new Vector3d(s * deltaSumX, s * deltaSumY, s * deltaSumZ);

            }

            double betaUpper = 0;
            double betaLower = 0;

            for (int i = 0; i < count; i++)
            {
                betaLower += Jacobi[i].SquareLength;
                betaUpper += (nextJacobi[i] - Jacobi[i]) * nextJacobi[i];
            }

            //係数αは1としている
            for (int i = 0; i < count; i++)
            {
                nextp[i] = -nextJacobi[i] + betaUpper * p[i] / betaLower;
                newTopoVertices[internalVertexIndices[i]] += 1 * nextp[i];
                p[i] = nextp[i];
                Jacobi[i] = nextJacobi[i];
            }
        }




        public static List<List<List<int>>> CategolizeCutMesh(CutMesh mesh, bool xyMirror, bool yzMirror, bool zxMirror, List<int> A, List<int> fixIndices)
        {
            List<int> xyMirrorIndices = new List<int>();
            List<int> yzMirrorIndices = new List<int>();
            List<int> zxMirrorIndices = new List<int>();
            List<List<int>> xyMirrorInnerCut = new List<List<int>>();
            List<List<int>> yzMirrorInnerCut = new List<List<int>>();
            List<List<int>> zxMirrorInnerCut = new List<List<int>>();
            if (xyMirror) { (xyMirrorIndices, xyMirrorInnerCut) = GetMirrorIndices(mesh, 2); }
            if (yzMirror) { (yzMirrorIndices, yzMirrorInnerCut) = GetMirrorIndices(mesh, 0); }
            if (zxMirror) { (zxMirrorIndices, zxMirrorInnerCut) = GetMirrorIndices(mesh, 1); }
            List<int> D = xyMirrorInnerCut.Concat(yzMirrorInnerCut).Concat(zxMirrorInnerCut).SelectMany(sublist => sublist).ToList();

            List<int> By = xyMirrorIndices.Intersect(yzMirrorIndices).ToList();
            List<int> Bz = yzMirrorIndices.Intersect(zxMirrorIndices).ToList();
            List<int> Bx = zxMirrorIndices.Intersect(xyMirrorIndices).ToList();
            List<int> ByCopy = new List<int>(By);
            List<int> BzCopy = new List<int>(Bz);
            List<int> BxCopy = new List<int>(Bx);

            List<List<int>> xyYZinnerCut = new List<List<int>>();
            List<List<int>> yzZXinnerCut = new List<List<int>>();
            List<List<int>> zxXYinnerCut = new List<List<int>>();

            List<List<int>> duplicatedVertIndices = mesh.DuplicatedVertIndices;
            if (ByCopy.Count > 0)
            {

                for (int i = 0; i < ByCopy.Count; i++)
                {
                    List<int> cut1 = xyMirrorInnerCut.FirstOrDefault(sublist => sublist.Contains(ByCopy[i]));
                    if (cut1 != null)
                    {
                        xyYZinnerCut.Add(cut1);
                        xyMirrorInnerCut.Remove(cut1);
                        yzMirrorInnerCut.Remove(cut1);
                        List<int> by1 = duplicatedVertIndices.FirstOrDefault(sublist => sublist.Contains(ByCopy[i]));
                        By = By.Except(by1).ToList();
                    }
                }
            }
            if (BzCopy.Count > 0)
            {
                for (int i = 0; i < BzCopy.Count; i++)
                {
                    List<int> cut1 = yzMirrorInnerCut.FirstOrDefault(sublist => sublist.Contains(BzCopy[i]));
                    if (cut1 != null)
                    {
                        yzZXinnerCut.Add(cut1);
                        yzMirrorInnerCut.Remove(cut1);
                        zxMirrorInnerCut.Remove(cut1);
                        List<int> bz1 = duplicatedVertIndices.FirstOrDefault(sublist => sublist.Contains(BzCopy[i]));
                        Bz = Bz.Except(bz1).ToList();
                    }
                }
            }
            if (BxCopy.Count > 0)
            {
                for (int i = 0; i < BxCopy.Count; i++)
                {
                    List<int> cut1 = zxMirrorInnerCut.FirstOrDefault(sublist => sublist.Contains(BxCopy[i]));
                    if (cut1 != null)
                    {
                        zxXYinnerCut.Add(cut1);
                        zxMirrorInnerCut.Remove(cut1);
                        xyMirrorInnerCut.Remove(cut1);
                        List<int> bx1 = duplicatedVertIndices.FirstOrDefault(sublist => sublist.Contains(BxCopy[i]));
                        Bx = Bx.Except(bx1).ToList();
                    }
                }
            }

            List<int> B = Bx.Concat(By).Concat(Bz).ToList();

            List<List<int>> DoneMirror = new List<List<int>>();
            if (xyMirrorInnerCut.Count > 0) { DoneMirror.AddRange(xyMirrorInnerCut); }
            if (yzMirrorInnerCut.Count > 0) { DoneMirror.AddRange(yzMirrorInnerCut); }
            if (zxMirrorInnerCut.Count > 0) { DoneMirror.AddRange(zxMirrorInnerCut); }

            List<List<int>> DtwoMirror = new List<List<int>>();
            if (xyYZinnerCut.Count > 0) { DtwoMirror.AddRange(xyYZinnerCut); }
            if (yzZXinnerCut.Count > 0) { DtwoMirror.AddRange(yzZXinnerCut); }
            if (zxXYinnerCut.Count > 0) { DtwoMirror.AddRange(zxXYinnerCut); }

            List<int> Axy = A.Intersect(xyMirrorIndices).ToList();
            List<int> Ayz = A.Intersect(yzMirrorIndices).ToList();
            List<int> Azx = A.Intersect(zxMirrorIndices).ToList();
            List<int> Afree = A.Except(Axy).Except(Ayz).Except(Azx).ToList();

            List<int> boundaryVertIndices = mesh.BoundaryVertIndices();
            List<int> fullSet = Enumerable.Range(0, mesh.Vertices.Count).ToList();
            List<int> E = fullSet.Except(boundaryVertIndices).ToList();

            List<int> Cxy = xyMirrorIndices.Except(Axy).Except(B).Except(xyMirrorInnerCut.SelectMany(sublist => sublist).ToList()).ToList();
            List<int> Cyz = yzMirrorIndices.Except(Ayz).Except(B).Except(yzMirrorInnerCut.SelectMany(sublist => sublist).ToList()).ToList();
            List<int> Czx = zxMirrorIndices.Except(Azx).Except(B).Except(zxMirrorInnerCut.SelectMany(sublist => sublist).ToList()).ToList();
            List<int> C = Cxy.Concat(Cyz).Concat(Czx).Distinct().ToList();

            List<int> F = boundaryVertIndices.Except(A).Except(D).Except(B).Except(C).ToList();
            F.Sort();
            List<int> boundaryEdges = mesh.BoundaryEdgeIndices();
            List<List<int>> Fblocks = new List<List<int>>();
            List<int> holeDict = new List<int>();

            for (int i = 0; i < F.Count; i++)
            {
                List<int> holeEdges = mesh.GetEdgesForVertex(F[i]).Intersect(boundaryEdges).ToList();
                List<int> holeVertices = new List<int>();
                for (int j = 0; j < holeEdges.Count; j++)
                {
                    holeVertices.Add(mesh.Edges[holeEdges[j]][0]);
                    holeVertices.Add(mesh.Edges[holeEdges[j]][1]);
                }
                if (!holeVertices.Contains(F[i] + 1)) { holeDict.Add(i); }
            }

            for (int i = 0; i < holeDict.Count; i++)
            {
                List<int> group = new List<int>();
                if (i == 0)
                {
                    for (int j = 0; j <= holeDict[i]; j++)
                    {
                        group.Add(F[j]);
                    }

                }
                else
                {
                    for (int j = holeDict[i - 1] + 1; j <= holeDict[i]; j++)
                    {
                        group.Add(F[j]);
                    }
                }
                Fblocks.Add(group);
            }

            List<List<int>> flatten = new List<List<int>> { A, B, C, D, E, F, Axy, Ayz, Azx, Afree, Bx, By, Bz, Cxy, Cyz, Czx };
            List<List<List<int>>> result = new List<List<List<int>>> { flatten, DoneMirror, xyMirrorInnerCut, yzMirrorInnerCut, zxMirrorInnerCut, DtwoMirror
            ,xyYZinnerCut, yzZXinnerCut, zxXYinnerCut, Fblocks};
            return result;
        }
        //共役勾配法で次のステップのメッシュを得る。ミラーに対応、ミラーはx,y,zのそれぞれmirror面に乗ってる点が多いほうの平面を軸とする
        //xyIndicesはxyMirrorするときのみ機能し、そのindicesの点は展開図でミラーとつながっていなくて良いとする。重なっている点の場合はDuplicatedの[0]のもののみ含む
        //A(つまりouterBoundaryVertIndices)はミラーして最終的に外側の境界となる部分の点のmesh.Verticesにおけるindices
        //A~FはDuplicatedしていてもそのすべての点が含まれている、meshはsortしておく

        public static CutMesh CGDevCutMeshConsiderOther(CutMesh mesh, int iterations, double alpha, bool xyMirror, bool yzMirror, bool zxMirror, List<int> A, List<int> fixIndices)
        {
            /*----------------------------------------------------A～Fの分類-----------------------------------------------------------------------*/
            List<List<List<int>>> category = CategolizeCutMesh(mesh, xyMirror, yzMirror, zxMirror, A, fixIndices);
            List<int> B = category[0][1];
            List<int> C = category[0][2];
            List<int> D = category[0][3];
            List<int> E = category[0][4];
            List<int> F = category[0][5];
            List<int> Axy = category[0][6];
            List<int> Ayz = category[0][7];
            List<int> Azx = category[0][8];
            List<int> Afree = category[0][9];
            List<int> Bx = category[0][10];
            List<int> By = category[0][11];
            List<int> Bz = category[0][12];
            List<int> Cxy = category[0][13];
            List<int> Cyz = category[0][14];
            List<int> Czx = category[0][15];
            List<List<int>> DoneMirror = category[1];
            List<List<int>> xyMirrorInnerCut = category[2];
            List<List<int>> yzMirrorInnerCut = category[3];
            List<List<int>> zxMirrorInnerCut = category[4];
            List<List<int>> DtwoMirror = category[5];
            List<List<int>> xyYZinnerCut = category[6];
            List<List<int>> yzZXinnerCut = category[7];
            List<List<int>> zxXYinnerCut = category[8];
            List<List<int>> Fblocks = category[9];


            /*--------トポロジカルに変わらなければ上記の分類は変わらない--------------------------------------------------------*/
            List<List<int>> duplicatedVertIndices = mesh.DuplicatedVertIndices;
            int dupVertCount = duplicatedVertIndices.Count;
            Vector3d[] preJacobi = new Vector3d[dupVertCount];
            preJacobi[0] = new Vector3d(1, 0, 0);
            Vector3d[] preP = new Vector3d[dupVertCount];

            /*------------------Jacobiによる点の移動で変わっていくのは以下の部分------------*/
            for (int u = 0; u < iterations; u++)
            {
                //ループのそれぞれの辺の長さを出しておく。それぞれ[i][0]はループの最初のインデックスとしていてラベル代わりとしている
                double[] angleSum = CutMeshCalcTools.AngleSum(mesh);
                List<List<double>> DoneLength = new List<List<double>>();
                List<List<double>> DoneCos = new List<List<double>>();
                List<List<double>> DoneSin = new List<List<double>>();
                for (int i = 0; i < DoneMirror.Count; i++)
                {
                    List<int> loopVertices = DoneMirror[i];
                    List<double> boxLength = new List<double> { loopVertices[0] };
                    List<double> boxCos = new List<double>();
                    List<double> boxSin = new List<double>();
                    double angleSumSum = 0;
                    for (int j = 0; j < loopVertices.Count - 1; j++)
                    {
                        double length = (mesh.Vertices[loopVertices[j]] - mesh.Vertices[loopVertices[j + 1]]).Length;
                        angleSumSum += angleSum[loopVertices[j]];
                        boxLength.Add(length);
                        boxCos.Add(Math.Cos(angleSumSum));
                        boxSin.Add(Math.Sin(angleSumSum));
                    }
                    DoneLength.Add(boxLength);
                    DoneCos.Add(boxCos);
                    DoneSin.Add(boxSin);
                }

                List<List<double>> FblocksLength = new List<List<double>>();
                List<List<double>> FblocksCos = new List<List<double>>();
                List<List<double>> FblocksSin = new List<List<double>>();
                for (int i = 0; i < Fblocks.Count; i++)
                {
                    List<int> loopVertices = Fblocks[i];
                    List<double> boxLength = new List<double> { (double)loopVertices[0] };
                    List<double> boxCos = new List<double>();
                    List<double> boxSin = new List<double>();
                    double angleSumSum = 0;
                    int loopVerticesCount = loopVertices.Count;
                    for (int j = 0; j < loopVerticesCount; j++)
                    {
                        double length = (mesh.Vertices[loopVertices[j]] - mesh.Vertices[loopVertices[(j + 1) % loopVerticesCount]]).Length;
                        boxLength.Add(length);
                        if (j == 0) { continue; }
                        angleSumSum += angleSum[loopVertices[j]];
                        boxCos.Add(Math.Cos(angleSumSum));
                        boxSin.Add(Math.Sin(angleSumSum));
                    }
                    FblocksLength.Add(boxLength);
                    FblocksCos.Add(boxCos);
                    FblocksSin.Add(boxSin);
                }

                /*----------------------------------------------------Jacobiの算出-----------------------------------------------------------------------*/
                Vector3d[] Jacobi = new Vector3d[dupVertCount];
                for (int i = 0; i < duplicatedVertIndices.Count; i++)
                {
                    //固定の点の場合はスキップ
                    List<int> vertGroup = duplicatedVertIndices[i];
                    vertGroup.Sort();
                    if (vertGroup.Intersect(fixIndices).ToList().Count != 0) { continue; }

                    else if (Axy.Contains(vertGroup[0]))
                    {
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, true, false); }
                                if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, true, false); }
                                if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, true, false); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, true, true, false);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, true, true, false);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, true, true, false);
                            }
                        }
                    }

                    else if (Ayz.Contains(vertGroup[0]))
                    {
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, false, true, true); }
                                if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, false, true, true); }
                                if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, false, true, true); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, false, true, true);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, false, true, true);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, false, true, true);
                            }
                        }
                    }

                    else if (Azx.Contains(vertGroup[0]))
                    {
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, false, true); }
                                if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, false, true); }
                                if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, false, true); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, true, false, true);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, true, false, true);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, true, false, true);
                            }
                        }
                    }

                    
                    else if (Afree.Contains(vertGroup[0]))
                    {
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, true, true); }
                                else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, true, true); }
                                else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, true, true); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }

                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, true, true, true);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, true, true, true);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, true, true, true);
                            }
                        }
                    }
                    

                    else if (Bx.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 8, true, false, false);
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, false, false); }
                                else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, false, false); }
                                else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, false, false); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, true, false, false);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, true, false, false);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, true, false, false);
                            }
                        }

                    }

                    else if (By.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 8, false, true, false);
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, false, true, false); }
                                else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, false, true, false); }
                                else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, false, true, false); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, false, true, false);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, false, true, false);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, false, true, false);
                            }
                        }

                    }

                    else if (Bz.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 8, false, false, true);
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, false, false, true); }
                                else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, false, false, true); }
                                else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, false, false, true); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, false, false, true);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, false, false, true);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, false, false, true);
                            }
                        }

                    }

                    else if (Cxy.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 4, true, true, false);
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, true, false); }
                                if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, true, false); }
                                if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, true, false); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, true, true, false);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, true, true, false);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, true, true, false);
                            }
                        }
                    }

                    else if (Cyz.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 4, false, true, true);
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, false, true, true); }
                                if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, false, true, true); }
                                if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, false, true, true); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, false, true, true);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, false, true, true);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, false, true, true);
                            }
                        }
                    }

                    else if (Czx.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 4, true, false, true);
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, false, true); }
                                if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, false, true); }
                                if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, false, true); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, true, false, true);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, true, false, true);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, true, false, true);
                            }
                        }
                    }

                    else if (E.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 2, true, true, true);
                        
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, true, true); }
                                else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, true, true); }
                                else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, true, true); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }

                        
                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, true, true, true);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, true, true, true);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, true, true, true);
                            }
                        }
                        
                    }

                    else if (DoneMirror.Any(sub => sub.Contains(vertGroup[0])))
                    {
                        List<int> loopVertices = DoneMirror.FirstOrDefault(sublist => sublist.Contains(vertGroup[0]));
                        int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                        List<double> loopLength = DoneLength[loopIndex];
                        List<double> loopCos = DoneCos[loopIndex];
                        List<double> loopSin = DoneSin[loopIndex];

                        /*-------------------------DFのeffectたちをリストアップしてグループ分け---------------------*/
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        effectDF = effectDF.Except(loopVertices).ToList();

                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        /*------------------------------------------------DFのグループ分け完了-----------------------------------------*/


                        if (xyMirrorInnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 2, true, true, false);
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, true, false); }
                                    else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, true, false); }
                                    else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, true, false); }
                                }
                            }
                            if (effectDoneBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDoneBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDoneBlocks[j][0];
                                    List<int> loopVertices2 = effectDoneBlocks[j][1];
                                    int loopIndex2 = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = DoneLength[loopIndex2];
                                    List<double> loopCos2 = DoneCos[loopIndex2];
                                    List<double> loopSin2 = DoneSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, true, true, false);
                                }
                            }

                            if (effectDtwoBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDtwoBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDtwoBlocks[j][0];
                                    List<int> loopVertices2 = effectDtwoBlocks[j][1];
                                    List<double> loopLength2 = new List<double>();
                                    List<double> loopCos2 = new List<double>();
                                    List<double> loopSin2 = new List<double>();
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, true, true, false);
                                }
                            }

                            if (effectFblocks.Count > 0)
                            {
                                for (int j = 0; j < effectFblocks.Count; j++)
                                {
                                    List<int> effectVertices = effectFblocks[j][0];
                                    List<int> loopVertices2 = effectFblocks[j][1];
                                    int loopIndex2 = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = FblocksLength[loopIndex2];
                                    List<double> loopCos2 = FblocksCos[loopIndex2];
                                    List<double> loopSin2 = FblocksSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, true, true, false);
                                }
                            }
                        }
                        else if (yzMirrorInnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 2, false, true, true);
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, false, true, true); }
                                    else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, false, true, true); }
                                    else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, false, true, true); }
                                }
                            }
                            if (effectDoneBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDoneBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDoneBlocks[j][0];
                                    List<int> loopVertices2 = effectDoneBlocks[j][1];
                                    int loopIndex2 = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = DoneLength[loopIndex2];
                                    List<double> loopCos2 = DoneCos[loopIndex2];
                                    List<double> loopSin2 = DoneSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, false, true, true);
                                }
                            }

                            if (effectDtwoBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDtwoBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDtwoBlocks[j][0];
                                    List<int> loopVertices2 = effectDtwoBlocks[j][1];
                                    List<double> loopLength2 = new List<double>();
                                    List<double> loopCos2 = new List<double>();
                                    List<double> loopSin2 = new List<double>();
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, false, true, true);
                                }
                            }

                            if (effectFblocks.Count > 0)
                            {
                                for (int j = 0; j < effectFblocks.Count; j++)
                                {
                                    List<int> effectVertices = effectFblocks[j][0];
                                    List<int> loopVertices2 = effectFblocks[j][1];
                                    int loopIndex2 = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = FblocksLength[loopIndex2];
                                    List<double> loopCos2 = FblocksCos[loopIndex2];
                                    List<double> loopSin2 = FblocksSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, false, true, true);
                                }
                            }
                        }
                        else if (zxMirrorInnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 2, true, false, true);
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, false, true); }
                                    else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, false, true); }
                                    else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, false, true); }
                                }
                            }
                            if (effectDoneBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDoneBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDoneBlocks[j][0];
                                    List<int> loopVertices2 = effectDoneBlocks[j][1];
                                    int loopIndex2 = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = DoneLength[loopIndex2];
                                    List<double> loopCos2 = DoneCos[loopIndex2];
                                    List<double> loopSin2 = DoneSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, true, false, true);
                                }
                            }

                            if (effectDtwoBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDtwoBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDtwoBlocks[j][0];
                                    List<int> loopVertices2 = effectDtwoBlocks[j][1];
                                    List<double> loopLength2 = new List<double>();
                                    List<double> loopCos2 = new List<double>();
                                    List<double> loopSin2 = new List<double>();
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, true, false, true);
                                }
                            }

                            if (effectFblocks.Count > 0)
                            {
                                for (int j = 0; j < effectFblocks.Count; j++)
                                {
                                    List<int> effectVertices = effectFblocks[j][0];
                                    List<int> loopVertices2 = effectFblocks[j][1];
                                    int loopIndex2 = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = FblocksLength[loopIndex2];
                                    List<double> loopCos2 = FblocksCos[loopIndex2];
                                    List<double> loopSin2 = FblocksSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, true, false, true);
                                }
                            }
                        }
                        else
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 2, true, true, true);
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, true, true); }
                                    else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, true, true); }
                                    else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, true, true); }
                                }
                            }
                            if (effectDoneBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDoneBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDoneBlocks[j][0];
                                    List<int> loopVertices2 = effectDoneBlocks[j][1];
                                    int loopIndex2 = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = DoneLength[loopIndex2];
                                    List<double> loopCos2 = DoneCos[loopIndex2];
                                    List<double> loopSin2 = DoneSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, true, true, true);
                                }
                            }

                            if (effectDtwoBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDtwoBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDtwoBlocks[j][0];
                                    List<int> loopVertices2 = effectDtwoBlocks[j][1];
                                    List<double> loopLength2 = new List<double>();
                                    List<double> loopCos2 = new List<double>();
                                    List<double> loopSin2 = new List<double>();
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, true, true, true);
                                }
                            }

                            if (effectFblocks.Count > 0)
                            {
                                for (int j = 0; j < effectFblocks.Count; j++)
                                {
                                    List<int> effectVertices = effectFblocks[j][0];
                                    List<int> loopVertices2 = effectFblocks[j][1];
                                    int loopIndex2 = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = FblocksLength[loopIndex2];
                                    List<double> loopCos2 = FblocksCos[loopIndex2];
                                    List<double> loopSin2 = FblocksSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, true, true, true);
                                }
                            }
                        }
                    }

                    else if (DtwoMirror.Any(sub => sub.Contains(vertGroup[0])))
                    {
                        List<int> loopVertices = DtwoMirror.FirstOrDefault(sublist => sublist.Contains(vertGroup[0]));
                        List<double> loopLength = new List<double>();
                        List<double> loopCos = new List<double>();
                        List<double> loopSin = new List<double>();

                        /*-------------------------DFのeffectたちをリストアップしてグループ分け---------------------*/
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        effectDF = effectDF.Except(loopVertices).ToList();

                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        /*------------------------------------------------DFのグループ分け完了-----------------------------------------*/


                        if (xyYZinnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 1, false, true, false);
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, false, true, false); }
                                    else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, false, true, false); }
                                    else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, false, true, false); }
                                }
                            }
                            if (effectDoneBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDoneBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDoneBlocks[j][0];
                                    List<int> loopVertices2 = effectDoneBlocks[j][1];
                                    int loopIndex2 = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = DoneLength[loopIndex2];
                                    List<double> loopCos2 = DoneCos[loopIndex2];
                                    List<double> loopSin2 = DoneSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, false, true, false);
                                }
                            }

                            if (effectDtwoBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDtwoBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDtwoBlocks[j][0];
                                    List<int> loopVertices2 = effectDtwoBlocks[j][1];
                                    List<double> loopLength2 = new List<double>();
                                    List<double> loopCos2 = new List<double>();
                                    List<double> loopSin2 = new List<double>();
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, false, true, false);
                                }
                            }

                            if (effectFblocks.Count > 0)
                            {
                                for (int j = 0; j < effectFblocks.Count; j++)
                                {
                                    List<int> effectVertices = effectFblocks[j][0];
                                    List<int> loopVertices2 = effectFblocks[j][1];
                                    int loopIndex2 = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = FblocksLength[loopIndex2];
                                    List<double> loopCos2 = FblocksCos[loopIndex2];
                                    List<double> loopSin2 = FblocksSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, false, true, false);
                                }
                            }
                        }
                        else if (yzZXinnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 1, false, false, true);
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, false, false, true); }
                                    else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, false, false, true); }
                                    else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, false, false, true); }
                                }
                            }
                            if (effectDoneBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDoneBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDoneBlocks[j][0];
                                    List<int> loopVertices2 = effectDoneBlocks[j][1];
                                    int loopIndex2 = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = DoneLength[loopIndex2];
                                    List<double> loopCos2 = DoneCos[loopIndex2];
                                    List<double> loopSin2 = DoneSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, false, false, true);
                                }
                            }

                            if (effectDtwoBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDtwoBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDtwoBlocks[j][0];
                                    List<int> loopVertices2 = effectDtwoBlocks[j][1];
                                    List<double> loopLength2 = new List<double>();
                                    List<double> loopCos2 = new List<double>();
                                    List<double> loopSin2 = new List<double>();
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, false, false, true);
                                }
                            }

                            if (effectFblocks.Count > 0)
                            {
                                for (int j = 0; j < effectFblocks.Count; j++)
                                {
                                    List<int> effectVertices = effectFblocks[j][0];
                                    List<int> loopVertices2 = effectFblocks[j][1];
                                    int loopIndex2 = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = FblocksLength[loopIndex2];
                                    List<double> loopCos2 = FblocksCos[loopIndex2];
                                    List<double> loopSin2 = FblocksSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, false, false, true);
                                }
                            }
                        }
                        else if (zxXYinnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 1, true, false, false);
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, false, false); }
                                    else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, false, false); }
                                    else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, false, false); }
                                }
                            }
                            if (effectDoneBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDoneBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDoneBlocks[j][0];
                                    List<int> loopVertices2 = effectDoneBlocks[j][1];
                                    int loopIndex2 = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = DoneLength[loopIndex2];
                                    List<double> loopCos2 = DoneCos[loopIndex2];
                                    List<double> loopSin2 = DoneSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, true, false, false);
                                }
                            }

                            if (effectDtwoBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDtwoBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDtwoBlocks[j][0];
                                    List<int> loopVertices2 = effectDtwoBlocks[j][1];
                                    List<double> loopLength2 = new List<double>();
                                    List<double> loopCos2 = new List<double>();
                                    List<double> loopSin2 = new List<double>();
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, true, false, false);
                                }
                            }

                            if (effectFblocks.Count > 0)
                            {
                                for (int j = 0; j < effectFblocks.Count; j++)
                                {
                                    List<int> effectVertices = effectFblocks[j][0];
                                    List<int> loopVertices2 = effectFblocks[j][1];
                                    int loopIndex2 = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = FblocksLength[loopIndex2];
                                    List<double> loopCos2 = FblocksCos[loopIndex2];
                                    List<double> loopSin2 = FblocksSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, true, false, false);
                                }
                            }
                        }
                        else
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 1, true, true, true);
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, true, true); }
                                    else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, true, true); }
                                    else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, true, true); }
                                }
                            }
                            if (effectDoneBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDoneBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDoneBlocks[j][0];
                                    List<int> loopVertices2 = effectDoneBlocks[j][1];
                                    int loopIndex2 = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = DoneLength[loopIndex2];
                                    List<double> loopCos2 = DoneCos[loopIndex2];
                                    List<double> loopSin2 = DoneSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, true, true, true);
                                }
                            }

                            if (effectDtwoBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDtwoBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDtwoBlocks[j][0];
                                    List<int> loopVertices2 = effectDtwoBlocks[j][1];
                                    List<double> loopLength2 = new List<double>();
                                    List<double> loopCos2 = new List<double>();
                                    List<double> loopSin2 = new List<double>();
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, true, true, true);
                                }
                            }

                            if (effectFblocks.Count > 0)
                            {
                                for (int j = 0; j < effectFblocks.Count; j++)
                                {
                                    List<int> effectVertices = effectFblocks[j][0];
                                    List<int> loopVertices2 = effectFblocks[j][1];
                                    int loopIndex2 = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = FblocksLength[loopIndex2];
                                    List<double> loopCos2 = FblocksCos[loopIndex2];
                                    List<double> loopSin2 = FblocksSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, true, true, true);
                                }
                            }
                        }
                    }

                    else if (F.Contains(vertGroup[0]))
                    {
                        List<int> loopVertices = Fblocks.FirstOrDefault(sublist => sublist.Contains(vertGroup[0]));
                        int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                        List<double> loopLength = FblocksLength[loopIndex];
                        List<double> loopCos = FblocksCos[loopIndex];
                        List<double> loopSin = FblocksSin[loopIndex];

                        /*-------------------------DFのeffectたちをリストアップしてグループ分け---------------------*/
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        effectDF = effectDF.Except(loopVertices).ToList();

                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        /*------------------------------------------------DFのグループ分け完了-----------------------------------------*/

                        Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 3, true, true, true);
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, true, true); }
                                else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, true, true); }
                                else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, true, true); }
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices2 = effectDoneBlocks[j][1];
                                int loopIndex2 = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                List<double> loopLength2 = DoneLength[loopIndex2];
                                List<double> loopCos2 = DoneCos[loopIndex2];
                                List<double> loopSin2 = DoneSin[loopIndex2];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, true, true, true);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices2 = effectDtwoBlocks[j][1];
                                List<double> loopLength2 = new List<double>();
                                List<double> loopCos2 = new List<double>();
                                List<double> loopSin2 = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, true, true, true);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices2 = effectFblocks[j][1];
                                int loopIndex2 = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                List<double> loopLength2 = FblocksLength[loopIndex2];
                                List<double> loopCos2 = FblocksCos[loopIndex2];
                                List<double> loopSin2 = FblocksSin[loopIndex2];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, true, true, true);
                            }
                        }

                    }
                }
                /*----------------------------------------------------Jacobiの算出終了-------------------------------------------*/

                //あとはJacobiから共役勾配法にしたがって点を移動させていく
                double betaUpper = 0;
                double betaLower = 0;

                for (int i = 0; i < dupVertCount; i++)
                {
                    betaLower += preJacobi[i].SquareLength;
                    betaUpper += (Jacobi[i] - preJacobi[i]) * Jacobi[i];
                }

                Vector3d[] p = new Vector3d[dupVertCount];
                for (int i = 0; i < dupVertCount; i++)
                {
                    p[i] = -Jacobi[i] + betaUpper * preP[i] / betaLower;
                    for (int j = 0; j < duplicatedVertIndices[i].Count; j++)
                    {
                        mesh.Vertices[duplicatedVertIndices[i][j]] += alpha * p[i];
                    }
                    preP[i] = p[i];
                    preJacobi[i] = Jacobi[i];
                }
            }
            return mesh;
        }

        public static CutMesh CGDevCutMeshConsiderJustSelf(CutMesh mesh, int iterations, double alpha, bool xyMirror, bool yzMirror, bool zxMirror, List<int> A, List<int> fixIndices)
        {
            /*----------------------------------------------------A～Fの分類-----------------------------------------------------------------------*/
            List<List<List<int>>> category = CategolizeCutMesh(mesh, xyMirror, yzMirror, zxMirror, A, fixIndices);
            List<int> B = category[0][1];
            List<int> C = category[0][2];
            List<int> D = category[0][3];
            List<int> E = category[0][4];
            List<int> F = category[0][5];
            List<int> Axy = category[0][6];
            List<int> Ayz = category[0][7];
            List<int> Azx = category[0][8];
            List<int> Afree = category[0][9];
            List<int> Bx = category[0][10];
            List<int> By = category[0][11];
            List<int> Bz = category[0][12];
            List<int> Cxy = category[0][13];
            List<int> Cyz = category[0][14];
            List<int> Czx = category[0][15];
            List<List<int>> DoneMirror = category[1];
            List<List<int>> xyMirrorInnerCut = category[2];
            List<List<int>> yzMirrorInnerCut = category[3];
            List<List<int>> zxMirrorInnerCut = category[4];
            List<List<int>> DtwoMirror = category[5];
            List<List<int>> xyYZinnerCut = category[6];
            List<List<int>> yzZXinnerCut = category[7];
            List<List<int>> zxXYinnerCut = category[8];
            List<List<int>> Fblocks = category[9];


            /*--------トポロジカルに変わらなければ上記の分類は変わらない--------------------------------------------------------*/
            List<List<int>> duplicatedVertIndices = mesh.DuplicatedVertIndices;
            int dupVertCount = duplicatedVertIndices.Count;
            Vector3d[] preJacobi = new Vector3d[dupVertCount];
            preJacobi[0] = new Vector3d(1, 0, 0);
            Vector3d[] preP = new Vector3d[dupVertCount];

            /*------------------Jacobiによる点の移動で変わっていくのは以下の部分------------*/
            for (int u = 0; u < iterations; u++)
            {
                //ループのそれぞれの辺の長さを出しておく。それぞれ[i][0]はループの最初のインデックスとしていてラベル代わりとしている
                double[] angleSum = CutMeshCalcTools.AngleSum(mesh);
                List<List<double>> DoneLength = new List<List<double>>();
                List<List<double>> DoneCos = new List<List<double>>();
                List<List<double>> DoneSin = new List<List<double>>();
                for (int i = 0; i < DoneMirror.Count; i++)
                {
                    List<int> loopVertices = DoneMirror[i];
                    List<double> boxLength = new List<double> { loopVertices[0] };
                    List<double> boxCos = new List<double>();
                    List<double> boxSin = new List<double>();
                    double angleSumSum = 0;
                    for (int j = 0; j < loopVertices.Count - 1; j++)
                    {
                        double length = (mesh.Vertices[loopVertices[j]] - mesh.Vertices[loopVertices[j + 1]]).Length;
                        angleSumSum += angleSum[loopVertices[j]];
                        boxLength.Add(length);
                        boxCos.Add(Math.Cos(angleSumSum));
                        boxSin.Add(Math.Sin(angleSumSum));
                    }
                    DoneLength.Add(boxLength);
                    DoneCos.Add(boxCos);
                    DoneSin.Add(boxSin);
                }

                List<List<double>> FblocksLength = new List<List<double>>();
                List<List<double>> FblocksCos = new List<List<double>>();
                List<List<double>> FblocksSin = new List<List<double>>();
                for (int i = 0; i < Fblocks.Count; i++)
                {
                    List<int> loopVertices = Fblocks[i];
                    List<double> boxLength = new List<double> { loopVertices[0] };
                    List<double> boxCos = new List<double>();
                    List<double> boxSin = new List<double>();
                    double angleSumSum = 0;
                    int loopVerticesCount = loopVertices.Count;
                    for (int j = 0; j < loopVerticesCount; j++)
                    {
                        double length = (mesh.Vertices[loopVertices[j]] - mesh.Vertices[loopVertices[(j + 1) % loopVerticesCount]]).Length;
                        boxLength.Add(length);
                        if (j == 0) { continue; }
                        angleSumSum += angleSum[loopVertices[j]];
                        boxCos.Add(Math.Cos(angleSumSum));
                        boxSin.Add(Math.Sin(angleSumSum));
                    }
                    FblocksLength.Add(boxLength);
                    FblocksCos.Add(boxCos);
                    FblocksSin.Add(boxSin);
                }

                /*----------------------------------------------------Jacobiの算出-----------------------------------------------------------------------*/
                Vector3d[] Jacobi = new Vector3d[dupVertCount];
                for (int i = 0; i < duplicatedVertIndices.Count; i++)
                {
                    //固定の点の場合はスキップ
                    List<int> vertGroup = duplicatedVertIndices[i];
                    vertGroup.Sort();
                    if (vertGroup.Intersect(fixIndices).ToList().Count != 0) { continue; }

                    else if (Bx.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 8, true, false, false);
                    }

                    else if (By.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 8, false, true, false);
                    }

                    else if (Bz.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 8, false, false, true);
                    }

                    else if (Cxy.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 4, true, true, false); 
                    }

                    else if (Cyz.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 4, false, true, true);
                    }

                    else if (Czx.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 4, true, false, true);
                    }

                    else if (E.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 2, true, true, true);
                    }

                    else if (DoneMirror.Any(sub => sub.Contains(vertGroup[0])))
                    {
                        List<int> loopVertices = DoneMirror.FirstOrDefault(sublist => sublist.Contains(vertGroup[0]));
                        int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                        List<double> loopLength = DoneLength[loopIndex];
                        List<double> loopCos = DoneCos[loopIndex];
                        List<double> loopSin = DoneSin[loopIndex];

                        /*-------------------------DFのeffectたちをリストアップしてグループ分け---------------------*/
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        effectDF = effectDF.Except(loopVertices).ToList();

                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        /*------------------------------------------------DFのグループ分け完了-----------------------------------------*/


                        if (xyMirrorInnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 2, true, true, false);
                        }
                        else if (yzMirrorInnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 2, false, true, true);
                        }
                        else if (zxMirrorInnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 2, true, false, true);
                        }
                        else
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 2, true, true, true);
                        }
                        
                    }

                    else if (DtwoMirror.Any(sub => sub.Contains(vertGroup[0])))
                    {
                        List<int> loopVertices = DtwoMirror.FirstOrDefault(sublist => sublist.Contains(vertGroup[0]));
                        List<double> loopLength = new List<double>();
                        List<double> loopCos = new List<double>();
                        List<double> loopSin = new List<double>();

                        /*-------------------------DFのeffectたちをリストアップしてグループ分け---------------------*/
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        effectDF = effectDF.Except(loopVertices).ToList();

                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        /*------------------------------------------------DFのグループ分け完了-----------------------------------------*/


                        if (xyYZinnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 1, false, true, false);
                           
                        }
                        else if (yzZXinnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 1, false, false, true);
                            
                        }
                        else if (zxXYinnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 1, true, false, false);
                            
                        }
                        else
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 1, true, true, true);
                            
                        }
                    }

                    else if (F.Contains(vertGroup[0]))
                    {
                        List<int> loopVertices = Fblocks.FirstOrDefault(sublist => sublist.Contains(vertGroup[0]));
                        int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                        List<double> loopLength = FblocksLength[loopIndex];
                        List<double> loopCos = FblocksCos[loopIndex];
                        List<double> loopSin = FblocksSin[loopIndex];

                        /*-------------------------DFのeffectたちをリストアップしてグループ分け---------------------*/
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        effectDF = effectDF.Except(loopVertices).ToList();

                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        /*------------------------------------------------DFのグループ分け完了-----------------------------------------*/

                        Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 3, true, true, true);
                        
                    }
                }
                /*----------------------------------------------------Jacobiの算出終了-------------------------------------------*/

                //あとはJacobiから共役勾配法にしたがって点を移動させていく
                double betaUpper = 0;
                double betaLower = 0;

                for (int i = 0; i < dupVertCount; i++)
                {
                    betaLower += preJacobi[i].SquareLength;
                    betaUpper += (Jacobi[i] - preJacobi[i]) * Jacobi[i];
                }

                Vector3d[] p = new Vector3d[dupVertCount];
                for (int i = 0; i < dupVertCount; i++)
                {
                    p[i] = -Jacobi[i] + betaUpper * preP[i] / betaLower;
                    for (int j = 0; j < duplicatedVertIndices[i].Count; j++)
                    {
                        mesh.Vertices[duplicatedVertIndices[i][j]] += alpha * p[i];
                    }
                    preP[i] = p[i];
                    preJacobi[i] = Jacobi[i];
                }
            }
            return mesh;
        }

        //Fの各項を隣接するface*2+1で割っているバージョン
        public static CutMesh CGDevCutMeshConsiderOther2(CutMesh mesh, int iterations, bool xyMirror, bool yzMirror, bool zxMirror, List<int> A, List<int> fixIndices)
        {
            /*----------------------------------------------------A～Fの分類-----------------------------------------------------------------------*/
            List<int> xyMirrorIndices = new List<int>();
            List<int> yzMirrorIndices = new List<int>();
            List<int> zxMirrorIndices = new List<int>();
            List<List<int>> xyMirrorInnerCut = new List<List<int>>();
            List<List<int>> yzMirrorInnerCut = new List<List<int>>();
            List<List<int>> zxMirrorInnerCut = new List<List<int>>();
            if (xyMirror) { (xyMirrorIndices, xyMirrorInnerCut) = GetMirrorIndices(mesh, 2); }
            if (yzMirror) { (yzMirrorIndices, yzMirrorInnerCut) = GetMirrorIndices(mesh, 0); }
            if (zxMirror) { (zxMirrorIndices, zxMirrorInnerCut) = GetMirrorIndices(mesh, 1); }
            List<int> D = xyMirrorInnerCut.Concat(yzMirrorInnerCut).Concat(zxMirrorInnerCut).SelectMany(sublist => sublist).ToList();

            List<int> By = xyMirrorIndices.Intersect(yzMirrorIndices).ToList();
            List<int> Bz = yzMirrorIndices.Intersect(zxMirrorIndices).ToList();
            List<int> Bx = zxMirrorIndices.Intersect(xyMirrorIndices).ToList();
            List<int> ByCopy = new List<int>(By);
            List<int> BzCopy = new List<int>(Bz);
            List<int> BxCopy = new List<int>(Bx);

            List<List<int>> xyYZinnerCut = new List<List<int>>();
            List<List<int>> yzZXinnerCut = new List<List<int>>();
            List<List<int>> zxXYinnerCut = new List<List<int>>();

            List<List<int>> duplicatedVertIndices = mesh.DuplicatedVertIndices;
            if (ByCopy.Count > 0)
            {

                for (int i = 0; i < ByCopy.Count; i++)
                {
                    List<int> cut1 = xyMirrorInnerCut.FirstOrDefault(sublist => sublist.Contains(ByCopy[i]));
                    if (cut1 != null)
                    {
                        xyYZinnerCut.Add(cut1);
                        xyMirrorInnerCut.Remove(cut1);
                        yzMirrorInnerCut.Remove(cut1);
                        List<int> by1 = duplicatedVertIndices.FirstOrDefault(sublist => sublist.Contains(ByCopy[i]));
                        By = By.Except(by1).ToList();
                    }
                }
            }
            if (BzCopy.Count > 0)
            {
                for (int i = 0; i < BzCopy.Count; i++)
                {
                    List<int> cut1 = yzMirrorInnerCut.FirstOrDefault(sublist => sublist.Contains(BzCopy[i]));
                    if (cut1 != null)
                    {
                        yzZXinnerCut.Add(cut1);
                        yzMirrorInnerCut.Remove(cut1);
                        zxMirrorInnerCut.Remove(cut1);
                        List<int> bz1 = duplicatedVertIndices.FirstOrDefault(sublist => sublist.Contains(BzCopy[i]));
                        Bz = Bz.Except(bz1).ToList();
                    }
                }
            }
            if (BxCopy.Count > 0)
            {
                for (int i = 0; i < BxCopy.Count; i++)
                {
                    List<int> cut1 = zxMirrorInnerCut.FirstOrDefault(sublist => sublist.Contains(BxCopy[i]));
                    if (cut1 != null)
                    {
                        zxXYinnerCut.Add(cut1);
                        zxMirrorInnerCut.Remove(cut1);
                        xyMirrorInnerCut.Remove(cut1);
                        List<int> bx1 = duplicatedVertIndices.FirstOrDefault(sublist => sublist.Contains(BxCopy[i]));
                        Bx = Bx.Except(bx1).ToList();
                    }
                }
            }

            List<int> B = Bx.Concat(By).Concat(Bz).ToList();

            List<List<int>> DoneMirror = new List<List<int>>();
            if (xyMirrorInnerCut.Count > 0) { DoneMirror.AddRange(xyMirrorInnerCut); }
            if (yzMirrorInnerCut.Count > 0) { DoneMirror.AddRange(yzMirrorInnerCut); }
            if (zxMirrorInnerCut.Count > 0) { DoneMirror.AddRange(zxMirrorInnerCut); }

            List<List<int>> DtwoMirror = new List<List<int>>();
            if (xyYZinnerCut.Count > 0) { DtwoMirror.AddRange(xyYZinnerCut); }
            if (yzZXinnerCut.Count > 0) { DtwoMirror.AddRange(yzZXinnerCut); }
            if (zxXYinnerCut.Count > 0) { DtwoMirror.AddRange(zxXYinnerCut); }

            List<int> Axy = A.Intersect(xyMirrorIndices).ToList();
            List<int> Ayz = A.Intersect(yzMirrorIndices).ToList();
            List<int> Azx = A.Intersect(zxMirrorIndices).ToList();
            List<int> Afree = A.Except(Axy).Except(Ayz).Except(Azx).ToList();

            List<int> boundaryVertIndices = mesh.BoundaryVertIndices();
            List<int> fullSet = Enumerable.Range(0, mesh.Vertices.Count).ToList();
            List<int> E = fullSet.Except(boundaryVertIndices).ToList();

            List<int> Cxy = xyMirrorIndices.Except(Axy).Except(B).Except(xyMirrorInnerCut.SelectMany(sublist => sublist).ToList()).ToList();
            List<int> Cyz = yzMirrorIndices.Except(Ayz).Except(B).Except(yzMirrorInnerCut.SelectMany(sublist => sublist).ToList()).ToList();
            List<int> Czx = zxMirrorIndices.Except(Azx).Except(B).Except(zxMirrorInnerCut.SelectMany(sublist => sublist).ToList()).ToList();
            List<int> C = Cxy.Concat(Cyz).Concat(Czx).Distinct().ToList();

            List<int> F = boundaryVertIndices.Except(A).Except(D).Except(B).Except(C).ToList();
            F.Sort();
            List<int> boundaryEdges = mesh.BoundaryEdgeIndices();
            List<List<int>> Fblocks = new List<List<int>>();

            for (int i = 0; i < F.Count; i++)
            {
                Fblocks.Add(new List<int>());
                List<int> holeEdges = mesh.GetEdgesForVertex(F[i]).Intersect(boundaryEdges).ToList();
                Fblocks[Fblocks.Count - 1].Add(F[i]);
                if (i == F.Count - 1) { break; }
                if (holeEdges.Count == 1) { boundaryEdges.Remove(holeEdges[0]); }
                else if (holeEdges.Count == 2) { boundaryEdges.Remove(holeEdges[0]); boundaryEdges.Remove(holeEdges[1]); }
                else { Fblocks.Add(new List<int>()); }
            }
            int dupVertCount = duplicatedVertIndices.Count;
            /*--------トポロジカルに変わらなければ上記の分類は変わらない--------------------------------------------------------*/


            Vector3d[] preJacobi = new Vector3d[dupVertCount];
            preJacobi[0] = new Vector3d(1, 0, 0);
            Vector3d[] preP = new Vector3d[dupVertCount];

            /*------------------Jacobiによる点の移動で変わっていくのは以下の部分------------*/
            for (int u = 0; u < iterations; u++)
            {
                //ループのそれぞれの辺の長さを出しておく。それぞれ[i][0]はループの最初のインデックスとしていてラベル代わりとしている
                double[] angleSum = CutMeshCalcTools.AngleSum(mesh);
                List<List<double>> DoneLength = new List<List<double>>();
                List<List<double>> DoneCos = new List<List<double>>();
                List<List<double>> DoneSin = new List<List<double>>();
                for (int i = 0; i < DoneMirror.Count; i++)
                {
                    List<int> loopVertices = DoneMirror[i];
                    List<double> boxLength = new List<double> { loopVertices[0] };
                    List<double> boxCos = new List<double>();
                    List<double> boxSin = new List<double>();
                    double angleSumSum = 0;
                    for (int j = 0; j < loopVertices.Count - 1; j++)
                    {
                        double length = (mesh.Vertices[loopVertices[j]] - mesh.Vertices[loopVertices[j + 1]]).Length;
                        angleSumSum += angleSum[loopVertices[j]];
                        boxLength.Add(length);
                        boxCos.Add(Math.Cos(angleSumSum));
                        boxSin.Add(Math.Sin(angleSumSum));
                    }
                    DoneLength.Add(boxLength);
                    DoneCos.Add(boxCos);
                    DoneSin.Add(boxSin);
                }

                List<List<double>> FblocksLength = new List<List<double>>();
                List<List<double>> FblocksCos = new List<List<double>>();
                List<List<double>> FblocksSin = new List<List<double>>();
                for (int i = 0; i < Fblocks.Count; i++)
                {
                    List<int> loopVertices = Fblocks[i];
                    List<double> boxLength = new List<double> { loopVertices[0] };
                    List<double> boxCos = new List<double>();
                    List<double> boxSin = new List<double>();
                    double angleSumSum = 0;
                    int loopVerticesCount = loopVertices.Count;
                    for (int j = 0; j < loopVerticesCount; j++)
                    {
                        double length = (mesh.Vertices[loopVertices[j]] - mesh.Vertices[loopVertices[(j + 1) % loopVerticesCount]]).Length;
                        boxLength.Add(length);
                        if (j == 0) { continue; }
                        angleSumSum += angleSum[loopVertices[j]];
                        boxCos.Add(Math.Cos(angleSumSum));
                        boxSin.Add(Math.Sin(angleSumSum));
                    }
                    FblocksLength.Add(boxLength);
                    FblocksCos.Add(boxCos);
                    FblocksSin.Add(boxSin);
                }

                /*----------------------------------------------------Jacobiの算出-----------------------------------------------------------------------*/
                Vector3d[] Jacobi = new Vector3d[dupVertCount];
                for (int i = 0; i < duplicatedVertIndices.Count; i++)
                {
                    //固定の点の場合はスキップ
                    List<int> vertGroup = duplicatedVertIndices[i];
                    vertGroup.Sort();
                    if (vertGroup.Intersect(fixIndices).ToList().Count != 0) { continue; }

                    else if (Axy.Contains(vertGroup[0]))
                    {
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, true, false); }
                                if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, true, false); }
                                if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, true, false); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, true, true, false);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, true, true, false);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, true, true, false);
                            }
                        }
                    }

                    else if (Ayz.Contains(vertGroup[0]))
                    {
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, false, true, true); }
                                if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, false, true, true); }
                                if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, false, true, true); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, false, true, true);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, false, true, true);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, false, true, true);
                            }
                        }
                    }

                    else if (Azx.Contains(vertGroup[0]))
                    {
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, false, true); }
                                if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, false, true); }
                                if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, false, true); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, true, false, true);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, true, false, true);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, true, false, true);
                            }
                        }
                    }


                    else if (Afree.Contains(vertGroup[0]))
                    {
                        int faceCount = 0;
                        for (int j = 0; j <  vertGroup.Count; j++)
                        {
                            faceCount += mesh.GetFacesForVertex(vertGroup[j]).Count;
                        }
                        int connect = 2 * faceCount + 1;
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, true, true)/connect; }
                                else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, true, true)/connect; }
                                else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, true, true)/connect; }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }

                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, true, true, true);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, true, true, true);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, true, true, true);
                            }
                        }
                    }


                    else if (Bx.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 8, true, false, false);
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, false, false); }
                                else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, false, false); }
                                else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, false, false); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, true, false, false);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, true, false, false);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, true, false, false);
                            }
                        }

                    }

                    else if (By.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 8, false, true, false);
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, false, true, false); }
                                else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, false, true, false); }
                                else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, false, true, false); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, false, true, false);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, false, true, false);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, false, true, false);
                            }
                        }

                    }

                    else if (Bz.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 8, false, false, true);
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, false, false, true); }
                                else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, false, false, true); }
                                else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, false, false, true); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, false, false, true);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, false, false, true);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, false, false, true);
                            }
                        }

                    }

                    else if (Cxy.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 4, true, true, false);
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, true, false); }
                                if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, true, false); }
                                if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, true, false); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, true, true, false);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, true, true, false);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, true, true, false);
                            }
                        }
                    }

                    else if (Cyz.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 4, false, true, true);
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, false, true, true); }
                                if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, false, true, true); }
                                if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, false, true, true); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, false, true, true);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, false, true, true);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, false, true, true);
                            }
                        }
                    }

                    else if (Czx.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 4, true, false, true);
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, false, true); }
                                if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, false, true); }
                                if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, false, true); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, true, false, true);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, true, false, true);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, true, false, true);
                            }
                        }
                    }

                    else if (E.Contains(vertGroup[0]))
                    {
                        int connect = 2 * mesh.GetFacesForVertex(vertGroup[0]).Count+1;
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 2, true, true, true)/connect;

                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, true, true)/connect; }
                                else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, true, true)/connect; }
                                else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, true, true)/connect; }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }


                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, true, true, true);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, true, true, true);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, true, true, true);
                            }
                        }

                    }

                    else if (DoneMirror.Any(sub => sub.Contains(vertGroup[0])))
                    {
                        List<int> loopVertices = DoneMirror.FirstOrDefault(sublist => sublist.Contains(vertGroup[0]));
                        int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                        List<double> loopLength = DoneLength[loopIndex];
                        List<double> loopCos = DoneCos[loopIndex];
                        List<double> loopSin = DoneSin[loopIndex];

                        /*-------------------------DFのeffectたちをリストアップしてグループ分け---------------------*/
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        effectDF = effectDF.Except(loopVertices).ToList();

                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        /*------------------------------------------------DFのグループ分け完了-----------------------------------------*/


                        if (xyMirrorInnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 2, true, true, false);
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, true, false); }
                                    else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, true, false); }
                                    else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, true, false); }
                                }
                            }
                            if (effectDoneBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDoneBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDoneBlocks[j][0];
                                    List<int> loopVertices2 = effectDoneBlocks[j][1];
                                    int loopIndex2 = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = DoneLength[loopIndex2];
                                    List<double> loopCos2 = DoneCos[loopIndex2];
                                    List<double> loopSin2 = DoneSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, true, true, false);
                                }
                            }

                            if (effectDtwoBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDtwoBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDtwoBlocks[j][0];
                                    List<int> loopVertices2 = effectDtwoBlocks[j][1];
                                    List<double> loopLength2 = new List<double>();
                                    List<double> loopCos2 = new List<double>();
                                    List<double> loopSin2 = new List<double>();
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, true, true, false);
                                }
                            }

                            if (effectFblocks.Count > 0)
                            {
                                for (int j = 0; j < effectFblocks.Count; j++)
                                {
                                    List<int> effectVertices = effectFblocks[j][0];
                                    List<int> loopVertices2 = effectFblocks[j][1];
                                    int loopIndex2 = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = FblocksLength[loopIndex2];
                                    List<double> loopCos2 = FblocksCos[loopIndex2];
                                    List<double> loopSin2 = FblocksSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, true, true, false);
                                }
                            }
                        }
                        else if (yzMirrorInnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 2, false, true, true);
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, false, true, true); }
                                    else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, false, true, true); }
                                    else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, false, true, true); }
                                }
                            }
                            if (effectDoneBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDoneBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDoneBlocks[j][0];
                                    List<int> loopVertices2 = effectDoneBlocks[j][1];
                                    int loopIndex2 = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = DoneLength[loopIndex2];
                                    List<double> loopCos2 = DoneCos[loopIndex2];
                                    List<double> loopSin2 = DoneSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, false, true, true);
                                }
                            }

                            if (effectDtwoBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDtwoBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDtwoBlocks[j][0];
                                    List<int> loopVertices2 = effectDtwoBlocks[j][1];
                                    List<double> loopLength2 = new List<double>();
                                    List<double> loopCos2 = new List<double>();
                                    List<double> loopSin2 = new List<double>();
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, false, true, true);
                                }
                            }

                            if (effectFblocks.Count > 0)
                            {
                                for (int j = 0; j < effectFblocks.Count; j++)
                                {
                                    List<int> effectVertices = effectFblocks[j][0];
                                    List<int> loopVertices2 = effectFblocks[j][1];
                                    int loopIndex2 = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = FblocksLength[loopIndex2];
                                    List<double> loopCos2 = FblocksCos[loopIndex2];
                                    List<double> loopSin2 = FblocksSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, false, true, true);
                                }
                            }
                        }
                        else if (zxMirrorInnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 2, true, false, true);
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, false, true); }
                                    else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, false, true); }
                                    else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, false, true); }
                                }
                            }
                            if (effectDoneBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDoneBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDoneBlocks[j][0];
                                    List<int> loopVertices2 = effectDoneBlocks[j][1];
                                    int loopIndex2 = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = DoneLength[loopIndex2];
                                    List<double> loopCos2 = DoneCos[loopIndex2];
                                    List<double> loopSin2 = DoneSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, true, false, true);
                                }
                            }

                            if (effectDtwoBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDtwoBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDtwoBlocks[j][0];
                                    List<int> loopVertices2 = effectDtwoBlocks[j][1];
                                    List<double> loopLength2 = new List<double>();
                                    List<double> loopCos2 = new List<double>();
                                    List<double> loopSin2 = new List<double>();
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, true, false, true);
                                }
                            }

                            if (effectFblocks.Count > 0)
                            {
                                for (int j = 0; j < effectFblocks.Count; j++)
                                {
                                    List<int> effectVertices = effectFblocks[j][0];
                                    List<int> loopVertices2 = effectFblocks[j][1];
                                    int loopIndex2 = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = FblocksLength[loopIndex2];
                                    List<double> loopCos2 = FblocksCos[loopIndex2];
                                    List<double> loopSin2 = FblocksSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, true, false, true);
                                }
                            }
                        }
                        else
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 2, true, true, true);
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, true, true); }
                                    else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, true, true); }
                                    else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, true, true); }
                                }
                            }
                            if (effectDoneBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDoneBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDoneBlocks[j][0];
                                    List<int> loopVertices2 = effectDoneBlocks[j][1];
                                    int loopIndex2 = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = DoneLength[loopIndex2];
                                    List<double> loopCos2 = DoneCos[loopIndex2];
                                    List<double> loopSin2 = DoneSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, true, true, true);
                                }
                            }

                            if (effectDtwoBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDtwoBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDtwoBlocks[j][0];
                                    List<int> loopVertices2 = effectDtwoBlocks[j][1];
                                    List<double> loopLength2 = new List<double>();
                                    List<double> loopCos2 = new List<double>();
                                    List<double> loopSin2 = new List<double>();
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, true, true, true);
                                }
                            }

                            if (effectFblocks.Count > 0)
                            {
                                for (int j = 0; j < effectFblocks.Count; j++)
                                {
                                    List<int> effectVertices = effectFblocks[j][0];
                                    List<int> loopVertices2 = effectFblocks[j][1];
                                    int loopIndex2 = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = FblocksLength[loopIndex2];
                                    List<double> loopCos2 = FblocksCos[loopIndex2];
                                    List<double> loopSin2 = FblocksSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, true, true, true);
                                }
                            }
                        }
                    }

                    else if (DtwoMirror.Any(sub => sub.Contains(vertGroup[0])))
                    {
                        List<int> loopVertices = DtwoMirror.FirstOrDefault(sublist => sublist.Contains(vertGroup[0]));
                        List<double> loopLength = new List<double>();
                        List<double> loopCos = new List<double>();
                        List<double> loopSin = new List<double>();

                        /*-------------------------DFのeffectたちをリストアップしてグループ分け---------------------*/
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        effectDF = effectDF.Except(loopVertices).ToList();

                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        /*------------------------------------------------DFのグループ分け完了-----------------------------------------*/


                        if (xyYZinnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 1, false, true, false);
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, false, true, false); }
                                    else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, false, true, false); }
                                    else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, false, true, false); }
                                }
                            }
                            if (effectDoneBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDoneBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDoneBlocks[j][0];
                                    List<int> loopVertices2 = effectDoneBlocks[j][1];
                                    int loopIndex2 = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = DoneLength[loopIndex2];
                                    List<double> loopCos2 = DoneCos[loopIndex2];
                                    List<double> loopSin2 = DoneSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, false, true, false);
                                }
                            }

                            if (effectDtwoBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDtwoBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDtwoBlocks[j][0];
                                    List<int> loopVertices2 = effectDtwoBlocks[j][1];
                                    List<double> loopLength2 = new List<double>();
                                    List<double> loopCos2 = new List<double>();
                                    List<double> loopSin2 = new List<double>();
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, false, true, false);
                                }
                            }

                            if (effectFblocks.Count > 0)
                            {
                                for (int j = 0; j < effectFblocks.Count; j++)
                                {
                                    List<int> effectVertices = effectFblocks[j][0];
                                    List<int> loopVertices2 = effectFblocks[j][1];
                                    int loopIndex2 = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = FblocksLength[loopIndex2];
                                    List<double> loopCos2 = FblocksCos[loopIndex2];
                                    List<double> loopSin2 = FblocksSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, false, true, false);
                                }
                            }
                        }
                        else if (yzZXinnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 1, false, false, true);
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, false, false, true); }
                                    else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, false, false, true); }
                                    else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, false, false, true); }
                                }
                            }
                            if (effectDoneBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDoneBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDoneBlocks[j][0];
                                    List<int> loopVertices2 = effectDoneBlocks[j][1];
                                    int loopIndex2 = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = DoneLength[loopIndex2];
                                    List<double> loopCos2 = DoneCos[loopIndex2];
                                    List<double> loopSin2 = DoneSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, false, false, true);
                                }
                            }

                            if (effectDtwoBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDtwoBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDtwoBlocks[j][0];
                                    List<int> loopVertices2 = effectDtwoBlocks[j][1];
                                    List<double> loopLength2 = new List<double>();
                                    List<double> loopCos2 = new List<double>();
                                    List<double> loopSin2 = new List<double>();
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, false, false, true);
                                }
                            }

                            if (effectFblocks.Count > 0)
                            {
                                for (int j = 0; j < effectFblocks.Count; j++)
                                {
                                    List<int> effectVertices = effectFblocks[j][0];
                                    List<int> loopVertices2 = effectFblocks[j][1];
                                    int loopIndex2 = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = FblocksLength[loopIndex2];
                                    List<double> loopCos2 = FblocksCos[loopIndex2];
                                    List<double> loopSin2 = FblocksSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, false, false, true);
                                }
                            }
                        }
                        else if (zxXYinnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 1, true, false, false);
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, false, false); }
                                    else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, false, false); }
                                    else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, false, false); }
                                }
                            }
                            if (effectDoneBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDoneBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDoneBlocks[j][0];
                                    List<int> loopVertices2 = effectDoneBlocks[j][1];
                                    int loopIndex2 = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = DoneLength[loopIndex2];
                                    List<double> loopCos2 = DoneCos[loopIndex2];
                                    List<double> loopSin2 = DoneSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, true, false, false);
                                }
                            }

                            if (effectDtwoBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDtwoBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDtwoBlocks[j][0];
                                    List<int> loopVertices2 = effectDtwoBlocks[j][1];
                                    List<double> loopLength2 = new List<double>();
                                    List<double> loopCos2 = new List<double>();
                                    List<double> loopSin2 = new List<double>();
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, true, false, false);
                                }
                            }

                            if (effectFblocks.Count > 0)
                            {
                                for (int j = 0; j < effectFblocks.Count; j++)
                                {
                                    List<int> effectVertices = effectFblocks[j][0];
                                    List<int> loopVertices2 = effectFblocks[j][1];
                                    int loopIndex2 = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = FblocksLength[loopIndex2];
                                    List<double> loopCos2 = FblocksCos[loopIndex2];
                                    List<double> loopSin2 = FblocksSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, true, false, false);
                                }
                            }
                        }
                        else
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 1, true, true, true);
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, true, true); }
                                    else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, true, true); }
                                    else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, true, true); }
                                }
                            }
                            if (effectDoneBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDoneBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDoneBlocks[j][0];
                                    List<int> loopVertices2 = effectDoneBlocks[j][1];
                                    int loopIndex2 = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = DoneLength[loopIndex2];
                                    List<double> loopCos2 = DoneCos[loopIndex2];
                                    List<double> loopSin2 = DoneSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, true, true, true);
                                }
                            }

                            if (effectDtwoBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDtwoBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDtwoBlocks[j][0];
                                    List<int> loopVertices2 = effectDtwoBlocks[j][1];
                                    List<double> loopLength2 = new List<double>();
                                    List<double> loopCos2 = new List<double>();
                                    List<double> loopSin2 = new List<double>();
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, true, true, true);
                                }
                            }

                            if (effectFblocks.Count > 0)
                            {
                                for (int j = 0; j < effectFblocks.Count; j++)
                                {
                                    List<int> effectVertices = effectFblocks[j][0];
                                    List<int> loopVertices2 = effectFblocks[j][1];
                                    int loopIndex2 = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = FblocksLength[loopIndex2];
                                    List<double> loopCos2 = FblocksCos[loopIndex2];
                                    List<double> loopSin2 = FblocksSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, true, true, true);
                                }
                            }
                        }
                    }

                    else if (F.Contains(vertGroup[0]))
                    {
                        List<int> loopVertices = Fblocks.FirstOrDefault(sublist => sublist.Contains(vertGroup[0]));
                        int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                        List<double> loopLength = FblocksLength[loopIndex];
                        List<double> loopCos = FblocksCos[loopIndex];
                        List<double> loopSin = FblocksSin[loopIndex];

                        /*-------------------------DFのeffectたちをリストアップしてグループ分け---------------------*/
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        effectDF = effectDF.Except(loopVertices).ToList();

                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        /*------------------------------------------------DFのグループ分け完了-----------------------------------------*/

                        Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 3, true, true, true);
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, true, true); }
                                else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, true, true); }
                                else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, true, true); }
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices2 = effectDoneBlocks[j][1];
                                int loopIndex2 = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                List<double> loopLength2 = DoneLength[loopIndex2];
                                List<double> loopCos2 = DoneCos[loopIndex2];
                                List<double> loopSin2 = DoneSin[loopIndex2];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, true, true, true);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices2 = effectDtwoBlocks[j][1];
                                List<double> loopLength2 = new List<double>();
                                List<double> loopCos2 = new List<double>();
                                List<double> loopSin2 = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, true, true, true);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices2 = effectFblocks[j][1];
                                int loopIndex2 = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                List<double> loopLength2 = FblocksLength[loopIndex2];
                                List<double> loopCos2 = FblocksCos[loopIndex2];
                                List<double> loopSin2 = FblocksSin[loopIndex2];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, true, true, true);
                            }
                        }

                    }
                }
                /*----------------------------------------------------Jacobiの算出終了-------------------------------------------*/

                //あとはJacobiから共役勾配法にしたがって点を移動させていく
                double betaUpper = 0;
                double betaLower = 0;

                for (int i = 0; i < dupVertCount; i++)
                {
                    betaLower += preJacobi[i].SquareLength;
                    betaUpper += (Jacobi[i] - preJacobi[i]) * Jacobi[i];
                }

                //係数αは1としている
                Vector3d[] p = new Vector3d[dupVertCount];
                for (int i = 0; i < dupVertCount; i++)
                {
                    p[i] = -Jacobi[i] + betaUpper * preP[i] / betaLower;
                    mesh.Vertices[i] += 1 * p[i];
                    preP[i] = p[i];
                    preJacobi[i] = Jacobi[i];
                }
            }
            return mesh;
        }

        //xyz = 2ならXY平面ミラー、0ならYZ平面ミラー、１ならZX平面ミラー、IndicesはCGDevCutMeshのxyIndicesなどと同じ
        //ミラー平面上の点のindicesと外側境界のうちミラー後にinnerCutになる部分のindicesを返す
        public static (List<int> MirrorIndices, List<List<int>> MirrorInnerCut) GetMirrorIndices(CutMesh mesh, int xyz)
        {
            List<int> MirrorIndicesMax = new List<int>();
            List<List<int>> MirrorInnerCutMax = new List<List<int>>();
            List<int> MirrorIndicesMin = new List<int>();
            List<List<int>> MirrorInnerCutMin = new List<List<int>>();
            double zMax = mesh.Vertices.Max(pt => pt[xyz]);
            double zMin = mesh.Vertices.Min(pt => pt[xyz]);
            int mirrorCountMax = 0;
            int mirrorCountMin = 0;

            for (int i = 0; i < mesh.DuplicatedVertIndices.Count; i++)
            {
                List<int> dupIndices = mesh.DuplicatedVertIndices[i];
                int dupIndex = dupIndices[0];

                if (mesh.Vertices[dupIndex][xyz] == zMax)
                {
                    mirrorCountMax++;
                    MirrorIndicesMax.Add(dupIndex);
                    if (dupIndices.Count > 1)
                    {
                        dupIndices.Sort();
                        MirrorInnerCutMax.Add(Enumerable.Range(dupIndices[0], dupIndices[dupIndices.Count - 1] - dupIndices[0] + 1).ToList());
                    }
                }

                else if (mesh.Vertices[dupIndex][xyz] == zMin)
                {
                    mirrorCountMin++;
                    MirrorIndicesMin.Add(dupIndex);
                    if (dupIndices.Count > 1)
                    {
                        dupIndices.Sort();
                        MirrorInnerCutMin.Add(Enumerable.Range(dupIndices[0], dupIndices[dupIndices.Count - 1] - dupIndices[0] + 1).ToList());
                    }
                }
            }
            if (mirrorCountMax > mirrorCountMin) { return (MirrorIndicesMax, MirrorInnerCutMax); }
            else { return (MirrorIndicesMin, MirrorInnerCutMin); }
        }

        //effectVertがgroupB,C,Eのいずれかの点とする。Bならratio:8, Cならratio:4, Eならratio:2
        public static Vector3d EffectToGroupBCEVertex(CutMesh cutMesh, double[] angleSum, int vertIndex, int effectVert, int ratio, bool moveX, bool moveY, bool moveZ)
        {
            List<Point3d> vertices = cutMesh.Vertices;
            double sX = 0;
            double sY = 0;
            double sZ = 0;
            if (vertIndex == effectVert)
            {
                List<int> faces = cutMesh.GetFacesForVertex(vertIndex);
                foreach (int face in faces)
                {
                    List<int> twoVert = cutMesh.GetVerticesForFace(face).Where(v => v != vertIndex).ToList();
                    Point3d p1 = vertices[twoVert[0]];
                    Point3d p2 = vertices[twoVert[1]];
                    Point3d p3 = vertices[vertIndex];
                    Vector3d a = p1 - p3;
                    Vector3d b = p2 - p3;
                    double abSinTheta = Vector3d.CrossProduct(a, b).Length;
                    double dot = a * b;
                    double abaa = 1 - dot / a.SquareLength;
                    double abbb = 1 - dot / b.SquareLength;
                    double wa = abaa + abbb;
                    if (moveX) { sX += (abaa * p1.X + abbb * p2.X - wa * p3.X) / abSinTheta; }
                    if (moveY) { sY += (abaa * p1.Y + abbb * p2.Y - wa * p3.Y) / abSinTheta; }
                    if (moveZ) { sZ += (abaa * p1.Z + abbb * p2.Z - wa * p3.Z) / abSinTheta; }

                }
            }

            else
            {
                int edgeIndex = cutMesh.GetEdgeForEndPoints(vertIndex, effectVert);
                List<int> faces = cutMesh.GetFacesForEdge(edgeIndex);
                foreach (int face in faces)
                {
                    int anotherVert = cutMesh.GetVerticesForFace(face).Where(v => v != vertIndex && v != effectVert).ToList()[0];
                    Point3d p1 = vertices[effectVert];
                    Point3d p2 = vertices[anotherVert];
                    Point3d p3 = vertices[vertIndex];
                    Vector3d a = p3 - p1;
                    Vector3d b = p2 - p1;

                    double abaa = (a * b) / a.SquareLength;
                    double abSinTheta = Vector3d.CrossProduct(a, b).Length;
                    if (moveX) { sX -= (p2.X - p1.X - (p3.X - p1.X) * abaa) / abSinTheta; }
                    if (moveY) { sY -= (p2.Y - p1.Y - (p3.Y - p1.Y) * abaa) / abSinTheta; }
                    if (moveZ) { sZ -= (p2.Z - p1.Z - (p3.Z - p1.Z) * abaa) / abSinTheta; }
                }
            }
            Vector3d Jacobi = new Vector3d();
            double angle = angleSum[effectVert];
            double calc = 0.5*ratio*(ratio * angle - 4 * Math.PI);
            Jacobi.X = sX * calc;
            Jacobi.Y = sY * calc;
            Jacobi.Z = sZ * calc;
            return Jacobi;
        }

        //loopが１方向ミラーならpattern = 2,2方向ミラーならpattern = 1,Fならpattern = 3
        public static Vector3d EffectToGroupDFVertex(CutMesh cutMesh, double[] angleSum, List<double> loopLength, List<double> loopCos, List<double> loopSin,
            List<int> vertGroup, List<int> effectVertices, List<int> loopVertices, int pattern, bool moveX, bool moveY, bool moveZ)
        {
            int n = loopVertices.Count;
            int k = 0;
            int ratio = 0;
            List<Point3d> vertices = cutMesh.Vertices;
            double sX = 0;
            double sY = 0;
            double sZ = 0;

            if (pattern == 2) { k = 2 * n; ratio = 2; }
            else if (pattern == 1){ k = 4 * n - 2; ratio = 4; }
            else { k = n + 2; ratio = 1; }

            double angle = 0;
            foreach (int vert in loopVertices)
            {
                angle += angleSum[vert];
            }

            List<double> sXList = new List<double>();
            List<double> sYList = new List<double>();
            List<double> sZList = new List<double>();

            foreach (int effectVert in effectVertices)
            {
                foreach (int vertIndex in vertGroup)
                {
                    int edgeIndex = cutMesh.GetEdgeForEndPoints(vertIndex, effectVert);
                    if (edgeIndex == -1) { continue; }
                    List<int> faces = cutMesh.GetFacesForEdge(edgeIndex);
                    foreach (int face in faces)
                    {
                        int anotherVert = cutMesh.GetVerticesForFace(face).Where(v => v != vertIndex && v != effectVert).ToList()[0];
                        Point3d p1 = vertices[effectVert];
                        Point3d p2 = vertices[anotherVert];
                        Point3d p3 = vertices[vertIndex];
                        Vector3d a = p3 - p1;
                        Vector3d b = p2 - p1;

                        double abaa = (a * b) / a.SquareLength;
                        double abSinTheta = Vector3d.CrossProduct(a, b).Length;
                        if (moveX) { sX -= (p2.X - p1.X - (p3.X - p1.X) * abaa) / abSinTheta; }
                        if (moveY) { sY -= (p2.Y - p1.Y - (p3.Y - p1.Y) * abaa) / abSinTheta; }
                        if (moveZ) { sZ -= (p2.Z - p1.Z - (p3.Z - p1.Z) * abaa) / abSinTheta; }
                    }
                }
                //if (effectVert == loopVertices[loopVertices.Count - 1]) { continue; }
                if (moveX) { sXList.Add(sX); }
                if (moveY) { sYList.Add(sY); }
                if (moveZ) { sZList.Add(sZ); }
            }

            Vector3d Jacobi = new Vector3d();
            double calc = 10 * (-2) * ratio * (k * Math.PI - ratio * angle) / pattern;
            //double calc =(-4) * ratio * (k * Math.PI - ratio * angle)/(pattern*k);
            Jacobi.X = sX * calc;
            Jacobi.Y = sY * calc;
            Jacobi.Z = sZ * calc;
            //2方向ミラーの場合
            if (pattern == 1) { return Jacobi; }
            //1方向ミラーの場合
            else if (pattern == 2)
            {
                double averageLength = loopLength[0];
                int count = loopVertices.Count;
                double s = 0;
                for (int j = 0; j < count - 1; j++)
                {
                    if (j % 2 == 0) { s += loopLength[j + 1] * loopSin[j]; }
                    else { s -= loopLength[j + 1] * loopSin[j]; }
                }
                double sumX = 0;
                double sumY = 0;
                double sumZ = 0;
                double tX = 0;
                double tY = 0;
                double tZ = 0;
                for (int j = 0; j < count - 1; j++)
                {
                    if (effectVertices.Contains(loopVertices[j]))
                    {
                        int listIndex = effectVertices.IndexOf(loopVertices[j]);
                        if (moveX) { tX = sXList[listIndex]; }
                        if (moveY) { tY = sYList[listIndex]; }
                        if (moveZ) { tZ = sZList[listIndex]; }
                    }
                    double value = loopLength[j + 1] * loopCos[j];
                    if (moveX) { if (j % 2 == 0) { sumX += value * tX; } else { sumX -= value * tX; } }
                    if (moveY) { if (j % 2 == 0) { sumY += value * tY; } else { sumY -= value * tY; } }
                    if (moveZ) { if (j % 2 == 0) { sumZ += value * tZ; } else { sumZ -= value * tZ; } }
                }
                double value2 = s * count * count * Math.PI * Math.PI / (averageLength * averageLength);
                Jacobi.X += sumX * value2;
                Jacobi.Y += sumY * value2;
                Jacobi.Z += sumZ * value2;

            }
            //Ｆの場合
            else
            {
                double averageLength = loopLength[0];
                int count = loopVertices.Count;
                double s = loopLength[1];
                for (int j = 1; j < count ; j++)
                {
                    if (j % 2 == 0) { s += loopLength[j + 1] * loopCos[j - 1]; }
                    else { s -= loopLength[j + 1] * loopCos[j - 1]; }
                }
                double sumX = 0;
                double sumY = 0;
                double sumZ = 0;
                double tX = 0;
                double tY = 0;
                double tZ = 0;
                for (int j = 1; j < count; j++)
                {
                    if (effectVertices.Contains(loopVertices[j]))
                    {
                        int listIndex = effectVertices.IndexOf(loopVertices[j]);
                        tX = sXList[listIndex];
                        tY = sYList[listIndex];
                        tZ = sZList[listIndex];
                    }
                    double value = loopLength[j + 1] * loopSin[j-1];
                    if (moveX) { if (j % 2 == 0) { sumX -= value * tX; } else { sumX += value * tX; } }
                    if (moveY) { if (j % 2 == 0) { sumY -= value * tY; } else { sumY += value * tY; } }
                    if (moveZ) { if (j % 2 == 0) { sumZ -= value * tZ; } else { sumZ += value * tZ; } }
                }
                //double value2 = 2 * s * count * count * Math.PI * Math.PI / (3 * averageLength * averageLength);
                double value2 = 2 * s * count * count * Math.PI * Math.PI / (3 * averageLength * averageLength);
                Jacobi.X += sumX * value2;
                Jacobi.Y += sumY * value2;
                Jacobi.Z += sumZ * value2;

                s = 0;
                for (int j = 1; j < count; j++)
                {
                    if (j % 2 == 0) { s += loopLength[j + 1] * loopSin[j - 1]; }
                    else { s -= loopLength[j + 1] * loopSin[j - 1]; }
                }
                sumX = 0;
                sumY = 0;
                sumZ = 0;
                tX = 0;
                tY = 0;
                tZ = 0;
                for (int j = 1; j < count; j++)
                {
                    if (effectVertices.Contains(loopVertices[j]))
                    {
                        int listIndex = effectVertices.IndexOf(loopVertices[j]);
                        if (moveX) { tX = sXList[listIndex]; }
                        if (moveY) { tY = sYList[listIndex]; }
                        if (moveZ) { tZ = sZList[listIndex]; }
                    }
                    double value = loopLength[j + 1] * loopCos[j - 1];
                    if (moveX) { if (j % 2 == 0) { sumX += value * tX; } else { sumX -= value * tX; } }
                    if (moveY) { if (j % 2 == 0) { sumY += value * tY; } else { sumX -= value * tY; } }
                    if (moveZ) { if (j % 2 == 0) { sumZ += value * tZ; } else { sumX -= value * tZ; } }
                }

                Jacobi.X += sumX * value2;
                Jacobi.Y += sumY * value2;
                Jacobi.Z += sumZ * value2;
            }

            double bairitu = 1000;
            Jacobi.X = Jacobi.X / bairitu;
            Jacobi.Y = Jacobi.Y / bairitu;
            Jacobi.Z = Jacobi.Z / bairitu;
            return Jacobi;
        }

        //loopが１方向ミラーならpattern = 2,2方向ミラーならpattern = 1, Fなら pattern = 3
        public static Vector3d VertGroupDFJacobi(CutMesh cutMesh, double[] angleSum, List<double> loopLength, List<double> loopCos, List<double> loopSin,
            List<int> vertGroup, List<int> loopVertices, int pattern, bool moveX, bool moveY, bool moveZ)
        {
            List<int> nextVertices = new List<int>();
            foreach (int vertIndex in vertGroup)
            {
                List<int> next = (cutMesh.GetVerticesForVertex(vertIndex).Intersect(loopVertices)).Except(nextVertices).ToList();
                nextVertices.AddRange(next);
            }
            int n = loopVertices.Count;
            int k = 0;
            int ratio = 0;
            List<Point3d> vertices = cutMesh.Vertices;
            double sX = 0;
            double sY = 0;
            double sZ = 0;

            if (pattern == 2) { k = 2 * n; ratio = 2; }
            else if (pattern == 1) { k = 4 * n - 2; ratio = 4; }
            else { k = n + 2; ratio = 1; }

            double angle = 0;
            foreach (int vert in loopVertices)
            {
                angle += angleSum[vert];
            }

            List<double> sXList = new List<double>();
            List<double> sYList = new List<double>();
            List<double> sZList = new List<double>();
            foreach (int effectVert in nextVertices)
            {
                foreach (int vertIndex in vertGroup)
                {
                    int edgeIndex = cutMesh.GetEdgeForEndPoints(vertIndex, effectVert);
                    if (edgeIndex == -1) { continue; }
                    List<int> faces = cutMesh.GetFacesForEdge(edgeIndex);
                    foreach (int face in faces)
                    {
                        int anotherVert = cutMesh.GetVerticesForFace(face).Where(v => v != vertIndex && v != effectVert).ToList()[0];
                        Point3d p1 = vertices[effectVert];
                        Point3d p2 = vertices[anotherVert];
                        Point3d p3 = vertices[vertIndex];
                        Vector3d a = p3 - p1;
                        Vector3d b = p2 - p1;

                        double abaa = (a * b) / a.SquareLength;
                        double abSinTheta = Vector3d.CrossProduct(a, b).Length;
                        if (moveX) { sX -= (p2.X - p1.X - (p3.X - p1.X) * abaa) / abSinTheta; }
                        if (moveY) { sY -= (p2.Y - p1.Y - (p3.Y - p1.Y) * abaa) / abSinTheta; }
                        if (moveZ) { sZ -= (p2.Z - p1.Z - (p3.Z - p1.Z) * abaa) / abSinTheta; }
                    }
                }
                //if (effectVert == loopVertices[loopVertices.Count - 1]) { continue; }
                if (moveX) { sXList.Add(sX); }
                if (moveY) { sYList.Add(sY); }
                if (moveZ) { sZList.Add(sZ); }
            }

            List<double> cXList = new List<double>();
            List<double> cYList = new List<double>();
            List<double> cZList = new List<double>();
            double sX2 = 0;
            double sY2 = 0;
            double sZ2 = 0;
            foreach (int vertIndex in vertGroup)
            {
                List<int> faces2 = cutMesh.GetFacesForVertex(vertIndex);
                foreach (int face in faces2)
                {
                    List<int> twoVert = cutMesh.GetVerticesForFace(face).Where(v => v != vertIndex).ToList();
                    Point3d p1 = vertices[twoVert[0]];
                    Point3d p2 = vertices[twoVert[1]];
                    Point3d p3 = vertices[vertIndex];
                    Vector3d a = p1 - p3;
                    Vector3d b = p2 - p3;
                    double abSinTheta = Vector3d.CrossProduct(a, b).Length;
                    double dot = a * b;
                    double abaa = 1 - dot / a.SquareLength;
                    double abbb = 1 - dot / b.SquareLength;
                    double wa = abaa + abbb;
                    if (moveX) { sX2 += (abaa * p1.X + abbb * p2.X - wa * p3.X) / abSinTheta; }
                    if (moveY) { sY2 += (abaa * p1.Y + abbb * p2.Y - wa * p3.Y) / abSinTheta; }
                    if (moveZ) { sZ2 += (abaa * p1.Z + abbb * p2.Z - wa * p3.Z) / abSinTheta; }
                }
                if (moveX) { cXList.Add(sX2); }
                if (moveY) { cYList.Add(sY2); }
                if (moveZ) { cZList.Add(sZ2); }
            }

            Vector3d Jacobi = new Vector3d();
            double calc = 10 *(-2) * ratio * (k * Math.PI - ratio * angle)/(pattern);
            Jacobi.X = (sX + sX2) * calc;
            Jacobi.Y = (sY + sY2) * calc;
            Jacobi.Z = (sZ + sZ2) * calc;
            //2方向ミラーの場合
            if (pattern == 1) { return Jacobi; }
            //1方向ミラーの場合
            else if (pattern == 2)
            {
                double averageLength = loopLength[0];
                int count = loopVertices.Count;
                double s = 0;
                for (int j = 0; j < count - 1; j++)
                {
                    if (j % 2 == 0) { s += loopLength[j + 1] * loopSin[j]; }
                    else { s -= loopLength[j + 1] * loopSin[j]; }
                }
                double sumX = 0;
                double sumY = 0;
                double sumZ = 0;
                double tX = 0;
                double tY = 0;
                double tZ = 0;
                double tX2 = 0;
                double tY2 = 0;
                double tZ2= 0;
                Point3d point = cutMesh.Vertices[vertGroup[0]];
                for (int j = 0; j < count - 1; j++)
                {
                    if (nextVertices.Contains(loopVertices[j]))
                    {
                        int listIndex = nextVertices.IndexOf(loopVertices[j]);
                        if (moveX) { tX = sXList[listIndex]; }
                        if (moveY) { tY = sYList[listIndex]; }
                        if (moveZ) { tZ = sZList[listIndex]; }
                    }
                    double dldx = 0;
                    double dldy = 0;
                    double dldz = 0;
                    if (vertGroup.Contains(loopVertices[j]))
                    {
                        int listIndex = vertGroup.IndexOf(loopVertices[j]);
                        if (moveX) { tX2 = cXList[listIndex]; dldx = (point.X - cutMesh.Vertices[loopVertices[j + 1]].X) / loopLength[j + 1]; }
                        if (moveY) { tY2 = cYList[listIndex]; dldy = (point.Y - cutMesh.Vertices[loopVertices[j + 1]].Y) / loopLength[j + 1]; }
                        if (moveZ) { tZ2 = cZList[listIndex]; dldz = (point.Z - cutMesh.Vertices[loopVertices[j + 1]].Z) / loopLength[j + 1]; }
                    }
                    if (vertGroup.Contains(loopVertices[j + 1]))
                    {
                        if (moveX) { dldx = (point.X - cutMesh.Vertices[loopVertices[j]].X) / loopLength[j + 1]; }
                        if (moveY) { dldy = (point.Y - cutMesh.Vertices[loopVertices[j]].Y) / loopLength[j + 1]; }
                        if (moveZ) { dldz = (point.Z - cutMesh.Vertices[loopVertices[j]].Z) / loopLength[j + 1]; }
                    }
                    double lencos = loopLength[j + 1] * loopCos[j];
                    if (moveX) { if (j % 2 == 0) { sumX += (loopSin[j] * dldx + lencos * (tX + tX2)); } else { sumX -= (loopSin[j] * dldx + lencos * (tX + tX2)); } }
                    if (moveY) { if (j % 2 == 0) { sumY += (loopSin[j] * dldy + lencos * (tY + tY2)); } else { sumY -= (loopSin[j] * dldy + lencos * (tY + tY2)); } }
                    if (moveZ) { if (j % 2 == 0) { sumZ += (loopSin[j] * dldz + lencos * (tZ + tZ2)); } else { sumZ -= (loopSin[j] * dldz + lencos * (tZ + tZ2)); } }
                }
                double value2 = s * count * count * Math.PI * Math.PI / (averageLength * averageLength);
                Jacobi.X += sumX * value2;
                Jacobi.Y += sumY * value2;
                Jacobi.Z += sumZ * value2;
            }
            //Ｆの場合
            else
            {
                double averageLength = loopLength[0];
                int count = loopVertices.Count;
                double s = 0;
                for (int j = 1; j < count; j++)
                {
                    if (j % 2 == 0) { s += loopLength[j + 1] * loopSin[j-1]; }
                    else { s -= loopLength[j + 1] * loopSin[j-1]; }
                }
                double sumX = 0;
                double sumY = 0;
                double sumZ = 0;
                double tX = 0;
                double tY = 0;
                double tZ = 0;
                double tX2 = 0;
                double tY2 = 0;
                double tZ2 = 0;
                Point3d point = cutMesh.Vertices[vertGroup[0]];
                for (int j = 1; j < count; j++)
                {
                    double dldx = 0;
                    double dldy = 0;
                    double dldz = 0;
                    if (nextVertices.Contains(loopVertices[j]))
                    {
                        int listIndex = nextVertices.IndexOf(loopVertices[j]);
                        if (moveX) { tX = sXList[listIndex]; }
                        if (moveY) { tY = sYList[listIndex]; }
                        if (moveZ) { tZ = sZList[listIndex]; }
                    }
                    if (vertGroup.Contains(loopVertices[j]))
                    {
                        int listIndex = vertGroup.IndexOf(loopVertices[j]);
                        if (moveX) { tX2 = cXList[listIndex]; dldx = (point.X - cutMesh.Vertices[loopVertices[(j + 1) % count]].X) / loopLength[j + 1]; }
                        if (moveY) { tY2 = cYList[listIndex]; dldy = (point.Y - cutMesh.Vertices[loopVertices[(j + 1) % count]].Y) / loopLength[j + 1]; }
                        if (moveZ) { tZ2 = cZList[listIndex]; dldz = (point.Z - cutMesh.Vertices[loopVertices[(j + 1) % count]].Z) / loopLength[j + 1]; }
                    }
                    if (vertGroup.Contains(loopVertices[(j + 1) % count]))
                    {
                        if (moveX) { dldx = (point.X - cutMesh.Vertices[loopVertices[j]].X) / loopLength[j + 1]; }
                        if (moveY) { dldy = (point.Y - cutMesh.Vertices[loopVertices[j]].Y) / loopLength[j + 1]; }
                        if (moveZ) { dldz = (point.Z - cutMesh.Vertices[loopVertices[j]].Z) / loopLength[j + 1]; }
                    }
                    double lencos = loopLength[j + 1] * loopCos[j - 1];
                    if (moveX) { if (j % 2 == 0) { sumX += (dldx * loopSin[j - 1] + lencos * (tX + tX2)); } else { sumX -= (dldx * loopSin[j - 1] + lencos * (tX + tX2)); } }
                    if (moveY) { if (j % 2 == 0) { sumY += (dldy * loopSin[j - 1] + lencos * (tY + tY2)); } else { sumY -= (dldy * loopSin[j - 1] + lencos * (tY + tY2)); } }
                    if (moveZ) { if (j % 2 == 0) { sumZ += (dldz * loopSin[j - 1] + lencos * (tZ + tZ2)); } else { sumZ -= (dldz * loopSin[j - 1] + lencos * (tZ + tZ2)); } }
                }
                double value2 = 2 * s * count * count * Math.PI * Math.PI / (3 * averageLength * averageLength);
                Jacobi.X += sumX * value2;
                Jacobi.Y += sumY * value2;
                Jacobi.Z += sumZ * value2;

                s = loopLength[1];
                for (int j = 1; j < count; j++)
                {
                    if (j % 2 == 0) { s += loopLength[j + 1] * loopCos[j - 1]; }
                    else { s -= loopLength[j + 1] * loopCos[j - 1]; }
                }
                sumX = 0;
                sumY = 0;
                sumZ = 0;
                tX = 0;
                tY = 0;
                tZ = 0;
                tX2 = 0;
                tY2 = 0;
                tZ2 = 0;
                for (int j = 1; j < count; j++)
                {
                    double dldx = 0;
                    double dldy = 0;
                    double dldz = 0;
                    if (nextVertices.Contains(loopVertices[j]))
                    {
                        int listIndex = nextVertices.IndexOf(loopVertices[j]);
                        if (moveX) { tX = sXList[listIndex]; }
                        if (moveY) { tY = sYList[listIndex]; }
                        if (moveZ) { tZ = sZList[listIndex]; }
                    }
                    if (vertGroup.Contains(loopVertices[j]))
                    {
                        int listIndex = vertGroup.IndexOf(loopVertices[j]);
                        if (moveX) { tX2 = cXList[listIndex]; dldx = (point.X - cutMesh.Vertices[loopVertices[(j + 1) % count]].X) / loopLength[j + 1]; }
                        if (moveY) { tY2 = cYList[listIndex]; dldy = (point.Y - cutMesh.Vertices[loopVertices[(j + 1) % count]].Y) / loopLength[j + 1]; }
                        if (moveZ) { tZ2 = cZList[listIndex]; dldz = (point.Z - cutMesh.Vertices[loopVertices[(j + 1) % count]].Z) / loopLength[j + 1]; }
                    }
                    if (vertGroup.Contains(loopVertices[(j + 1) % count]))
                    {
                        if (moveX) { dldx = (point.X - cutMesh.Vertices[loopVertices[j]].X) / loopLength[j + 1]; }
                        if (moveY) { dldy = (point.Y - cutMesh.Vertices[loopVertices[j]].Y) / loopLength[j + 1]; }
                        if (moveZ) { dldz = (point.Z - cutMesh.Vertices[loopVertices[j]].Z) / loopLength[j + 1]; }
                    }
                    double lencos = -loopLength[j + 1] * loopSin[j - 1];
                    if (moveX) { if (j % 2 == 0) { sumX += (dldx * loopCos[j - 1] + lencos * (tX + tX2)); } else { sumX -= (dldx * loopCos[j - 1] + lencos * (tX + tX2)); } }
                    if (moveY) { if (j % 2 == 0) { sumY += (dldy * loopCos[j - 1] + lencos * (tY + tY2)); } else { sumY -= (dldy * loopCos[j - 1] + lencos * (tY + tY2)); } }
                    if (moveZ) { if (j % 2 == 0) { sumZ += (dldz * loopCos[j - 1] + lencos * (tZ + tZ2)); } else { sumZ -= (dldz * loopCos[j - 1] + lencos * (tZ + tZ2)); } }
                }
                Jacobi.X += sumX * value2;
                Jacobi.Y += sumY * value2;
                Jacobi.Z += sumZ * value2;
            }

            double bairitu = 1000;
            Jacobi.X = Jacobi.X / bairitu;
            Jacobi.Y = Jacobi.Y / bairitu;
            Jacobi.Z = Jacobi.Z / bairitu;
            return Jacobi;
        }


        public static CutMesh CGDevCutMeshConsiderOther3
            (CutMesh mesh, int iterations, double alpha, bool xyMirror, bool yzMirror, bool zxMirror, List<int> A, List<int> fixIndices)
        {
            /*----------------------------------------------------A～Fの分類-----------------------------------------------------------------------*/
            List<List<List<int>>> category = CategolizeCutMesh(mesh, xyMirror, yzMirror, zxMirror, A, fixIndices);
            List<int> B = category[0][1];
            List<int> C = category[0][2];
            List<int> D = category[0][3];
            List<int> E = category[0][4];
            List<int> F = category[0][5];
            List<int> Axy = category[0][6];
            List<int> Ayz = category[0][7];
            List<int> Azx = category[0][8];
            List<int> Afree = category[0][9];
            List<int> Bx = category[0][10];
            List<int> By = category[0][11];
            List<int> Bz = category[0][12];
            List<int> Cxy = category[0][13];
            List<int> Cyz = category[0][14];
            List<int> Czx = category[0][15];
            List<List<int>> DoneMirror = category[1];
            List<List<int>> xyMirrorInnerCut = category[2];
            List<List<int>> yzMirrorInnerCut = category[3];
            List<List<int>> zxMirrorInnerCut = category[4];
            List<List<int>> DtwoMirror = category[5];
            List<List<int>> xyYZinnerCut = category[6];
            List<List<int>> yzZXinnerCut = category[7];
            List<List<int>> zxXYinnerCut = category[8];
            List<List<int>> Fblocks = category[9];


            /*--------トポロジカルに変わらなければ上記の分類は変わらない--------------------------------------------------------*/
            List<List<int>> duplicatedVertIndices = mesh.DuplicatedVertIndices;
            int dupVertCount = duplicatedVertIndices.Count;
            Vector3d[] preJacobi = new Vector3d[dupVertCount];
            preJacobi[0] = new Vector3d(1, 0, 0);
            Vector3d[] preP = new Vector3d[dupVertCount];

            /*------------------Jacobiによる点の移動で変わっていくのは以下の部分------------*/
            for (int u = 0; u < iterations; u++)
            {
                //ループのそれぞれの辺の長さを出しておく。それぞれ[i][0]はループの最初のインデックスとしていてラベル代わりとしている
                double[] angleSum = CutMeshCalcTools.AngleSum(mesh);
                List<List<double>> DoneLength = new List<List<double>>();
                List<List<double>> DoneCos = new List<List<double>>();
                List<List<double>> DoneSin = new List<List<double>>();
                for (int i = 0; i < DoneMirror.Count; i++)
                {
                    List<int> loopVertices = DoneMirror[i];
                    List<double> boxLength = new List<double> { loopVertices[0] };
                    List<double> boxCos = new List<double>();
                    List<double> boxSin = new List<double>();
                    double angleSumSum = 0;
                    for (int j = 0; j < loopVertices.Count - 1; j++)
                    {
                        double length = (mesh.Vertices[loopVertices[j]] - mesh.Vertices[loopVertices[j + 1]]).Length;
                        angleSumSum += angleSum[loopVertices[j]];
                        boxLength.Add(length);
                        boxCos.Add(Math.Cos(angleSumSum));
                        boxSin.Add(Math.Sin(angleSumSum));
                    }
                    DoneLength.Add(boxLength);
                    DoneCos.Add(boxCos);
                    DoneSin.Add(boxSin);
                }

                List<List<double>> FblocksLength = new List<List<double>>();
                List<List<double>> FblocksCos = new List<List<double>>();
                List<List<double>> FblocksSin = new List<List<double>>();
                for (int i = 0; i < Fblocks.Count; i++)
                {
                    List<int> loopVertices = Fblocks[i];
                    List<double> boxLength = new List<double> { (double)loopVertices[0] };
                    List<double> boxCos = new List<double>();
                    List<double> boxSin = new List<double>();
                    double angleSumSum = 0;
                    int loopVerticesCount = loopVertices.Count;
                    for (int j = 0; j < loopVerticesCount; j++)
                    {
                        double length = (mesh.Vertices[loopVertices[j]] - mesh.Vertices[loopVertices[(j + 1) % loopVerticesCount]]).Length;
                        boxLength.Add(length);
                        if (j == 0) { continue; }
                        angleSumSum += angleSum[loopVertices[j]];
                        boxCos.Add(Math.Cos(angleSumSum));
                        boxSin.Add(Math.Sin(angleSumSum));
                    }
                    FblocksLength.Add(boxLength);
                    FblocksCos.Add(boxCos);
                    FblocksSin.Add(boxSin);
                }

                /*----------------------------------------------------Jacobiの算出-----------------------------------------------------------------------*/
                Vector3d[] Jacobi = new Vector3d[dupVertCount];
                for (int i = 0; i < duplicatedVertIndices.Count; i++)
                {
                    //固定の点の場合はスキップ
                    List<int> vertGroup = duplicatedVertIndices[i];
                    vertGroup.Sort();
                    if (vertGroup.Intersect(fixIndices).ToList().Count != 0) { continue; }

                    else if (Axy.Contains(vertGroup[0]))
                    {
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, true, false); }
                                if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, true, false); }
                                if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, true, false); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, true, true, false);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, true, true, false);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, true, true, false);
                            }
                        }
                    }

                    else if (Ayz.Contains(vertGroup[0]))
                    {
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, false, true, true); }
                                if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, false, true, true); }
                                if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, false, true, true); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, false, true, true);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, false, true, true);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, false, true, true);
                            }
                        }
                    }

                    else if (Azx.Contains(vertGroup[0]))
                    {
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, false, true); }
                                if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, false, true); }
                                if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, false, true); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, true, false, true);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, true, false, true);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, true, false, true);
                            }
                        }
                    }


                    else if (Afree.Contains(vertGroup[0]))
                    {
                        List<int> effectDF = new List<int>();
                        int connectFaces = 0;
                        foreach (int vert in vertGroup)
                        {
                            connectFaces += mesh.GetFacesForVertex(vert).Count;
                        }
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, true, true); }
                                else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, true, true); }
                                else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, true, true); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }

                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, true, true, true);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, true, true, true);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, true, true, true);
                            }
                        }
                        Jacobi[i] = Jacobi[i] / (connectFaces * 2 + 1);
                    }


                    else if (Bx.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 8, true, false, false);
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, false, false); }
                                else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, false, false); }
                                else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, false, false); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, true, false, false);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, true, false, false);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, true, false, false);
                            }
                        }

                    }

                    else if (By.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 8, false, true, false);
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, false, true, false); }
                                else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, false, true, false); }
                                else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, false, true, false); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, false, true, false);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, false, true, false);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, false, true, false);
                            }
                        }

                    }

                    else if (Bz.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 8, false, false, true);
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, false, false, true); }
                                else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, false, false, true); }
                                else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, false, false, true); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, false, false, true);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, false, false, true);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, false, false, true);
                            }
                        }

                    }

                    else if (Cxy.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 4, true, true, false);
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, true, false); }
                                if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, true, false); }
                                if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, true, false); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, true, true, false);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, true, true, false);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, true, true, false);
                            }
                        }
                    }

                    else if (Cyz.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 4, false, true, true);
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, false, true, true); }
                                if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, false, true, true); }
                                if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, false, true, true); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, false, true, true);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, false, true, true);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, false, true, true);
                            }
                        }
                    }

                    else if (Czx.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 4, true, false, true);
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, false, true); }
                                if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, false, true); }
                                if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, false, true); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, true, false, true);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, true, false, true);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, true, false, true);
                            }
                        }
                    }

                    else if (E.Contains(vertGroup[0]))
                    {
                        int connectFaces = 0;
                        foreach (int vert in vertGroup)
                        {
                            connectFaces += mesh.GetFacesForVertex(vert).Count;
                        }
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 2, true, true, true);

                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, true, true); }
                                else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, true, true); }
                                else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, true, true); }
                                else if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }


                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices = effectDoneBlocks[j][1];
                                int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = DoneLength[loopIndex];
                                List<double> loopCos = DoneCos[loopIndex];
                                List<double> loopSin = DoneSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, true, true, true);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices = effectDtwoBlocks[j][1];
                                List<double> loopLength = new List<double>();
                                List<double> loopCos = new List<double>();
                                List<double> loopSin = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, true, true, true);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices = effectFblocks[j][1];
                                int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                                List<double> loopLength = FblocksLength[loopIndex];
                                List<double> loopCos = FblocksCos[loopIndex];
                                List<double> loopSin = FblocksSin[loopIndex];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, true, true, true);
                            }
                        }
                        Jacobi[i] = Jacobi[i] / (2 * connectFaces + 1);
                    }

                    else if (DoneMirror.Any(sub => sub.Contains(vertGroup[0])))
                    {
                        List<int> loopVertices = DoneMirror.FirstOrDefault(sublist => sublist.Contains(vertGroup[0]));
                        int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                        List<double> loopLength = DoneLength[loopIndex];
                        List<double> loopCos = DoneCos[loopIndex];
                        List<double> loopSin = DoneSin[loopIndex];

                        /*-------------------------DFのeffectたちをリストアップしてグループ分け---------------------*/
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        effectDF = effectDF.Except(loopVertices).ToList();

                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        /*------------------------------------------------DFのグループ分け完了-----------------------------------------*/


                        if (xyMirrorInnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 2, true, true, false);
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, true, false); }
                                    else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, true, false); }
                                    else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, true, false); }
                                }
                            }
                            if (effectDoneBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDoneBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDoneBlocks[j][0];
                                    List<int> loopVertices2 = effectDoneBlocks[j][1];
                                    int loopIndex2 = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = DoneLength[loopIndex2];
                                    List<double> loopCos2 = DoneCos[loopIndex2];
                                    List<double> loopSin2 = DoneSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, true, true, false);
                                }
                            }

                            if (effectDtwoBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDtwoBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDtwoBlocks[j][0];
                                    List<int> loopVertices2 = effectDtwoBlocks[j][1];
                                    List<double> loopLength2 = new List<double>();
                                    List<double> loopCos2 = new List<double>();
                                    List<double> loopSin2 = new List<double>();
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, true, true, false);
                                }
                            }

                            if (effectFblocks.Count > 0)
                            {
                                for (int j = 0; j < effectFblocks.Count; j++)
                                {
                                    List<int> effectVertices = effectFblocks[j][0];
                                    List<int> loopVertices2 = effectFblocks[j][1];
                                    int loopIndex2 = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = FblocksLength[loopIndex2];
                                    List<double> loopCos2 = FblocksCos[loopIndex2];
                                    List<double> loopSin2 = FblocksSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, true, true, false);
                                }
                            }
                        }
                        else if (yzMirrorInnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 2, false, true, true);
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, false, true, true); }
                                    else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, false, true, true); }
                                    else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, false, true, true); }
                                }
                            }
                            if (effectDoneBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDoneBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDoneBlocks[j][0];
                                    List<int> loopVertices2 = effectDoneBlocks[j][1];
                                    int loopIndex2 = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = DoneLength[loopIndex2];
                                    List<double> loopCos2 = DoneCos[loopIndex2];
                                    List<double> loopSin2 = DoneSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, false, true, true);
                                }
                            }

                            if (effectDtwoBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDtwoBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDtwoBlocks[j][0];
                                    List<int> loopVertices2 = effectDtwoBlocks[j][1];
                                    List<double> loopLength2 = new List<double>();
                                    List<double> loopCos2 = new List<double>();
                                    List<double> loopSin2 = new List<double>();
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, false, true, true);
                                }
                            }

                            if (effectFblocks.Count > 0)
                            {
                                for (int j = 0; j < effectFblocks.Count; j++)
                                {
                                    List<int> effectVertices = effectFblocks[j][0];
                                    List<int> loopVertices2 = effectFblocks[j][1];
                                    int loopIndex2 = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = FblocksLength[loopIndex2];
                                    List<double> loopCos2 = FblocksCos[loopIndex2];
                                    List<double> loopSin2 = FblocksSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, false, true, true);
                                }
                            }
                        }
                        else if (zxMirrorInnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 2, true, false, true);
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, false, true); }
                                    else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, false, true); }
                                    else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, false, true); }
                                }
                            }
                            if (effectDoneBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDoneBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDoneBlocks[j][0];
                                    List<int> loopVertices2 = effectDoneBlocks[j][1];
                                    int loopIndex2 = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = DoneLength[loopIndex2];
                                    List<double> loopCos2 = DoneCos[loopIndex2];
                                    List<double> loopSin2 = DoneSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, true, false, true);
                                }
                            }

                            if (effectDtwoBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDtwoBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDtwoBlocks[j][0];
                                    List<int> loopVertices2 = effectDtwoBlocks[j][1];
                                    List<double> loopLength2 = new List<double>();
                                    List<double> loopCos2 = new List<double>();
                                    List<double> loopSin2 = new List<double>();
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, true, false, true);
                                }
                            }

                            if (effectFblocks.Count > 0)
                            {
                                for (int j = 0; j < effectFblocks.Count; j++)
                                {
                                    List<int> effectVertices = effectFblocks[j][0];
                                    List<int> loopVertices2 = effectFblocks[j][1];
                                    int loopIndex2 = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = FblocksLength[loopIndex2];
                                    List<double> loopCos2 = FblocksCos[loopIndex2];
                                    List<double> loopSin2 = FblocksSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, true, false, true);
                                }
                            }
                        }
                        else
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 2, true, true, true);
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, true, true); }
                                    else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, true, true); }
                                    else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, true, true); }
                                }
                            }
                            if (effectDoneBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDoneBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDoneBlocks[j][0];
                                    List<int> loopVertices2 = effectDoneBlocks[j][1];
                                    int loopIndex2 = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = DoneLength[loopIndex2];
                                    List<double> loopCos2 = DoneCos[loopIndex2];
                                    List<double> loopSin2 = DoneSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, true, true, true);
                                }
                            }

                            if (effectDtwoBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDtwoBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDtwoBlocks[j][0];
                                    List<int> loopVertices2 = effectDtwoBlocks[j][1];
                                    List<double> loopLength2 = new List<double>();
                                    List<double> loopCos2 = new List<double>();
                                    List<double> loopSin2 = new List<double>();
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, true, true, true);
                                }
                            }

                            if (effectFblocks.Count > 0)
                            {
                                for (int j = 0; j < effectFblocks.Count; j++)
                                {
                                    List<int> effectVertices = effectFblocks[j][0];
                                    List<int> loopVertices2 = effectFblocks[j][1];
                                    int loopIndex2 = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = FblocksLength[loopIndex2];
                                    List<double> loopCos2 = FblocksCos[loopIndex2];
                                    List<double> loopSin2 = FblocksSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, true, true, true);
                                }
                            }
                        }
                    }

                    else if (DtwoMirror.Any(sub => sub.Contains(vertGroup[0])))
                    {
                        List<int> loopVertices = DtwoMirror.FirstOrDefault(sublist => sublist.Contains(vertGroup[0]));
                        List<double> loopLength = new List<double>();
                        List<double> loopCos = new List<double>();
                        List<double> loopSin = new List<double>();

                        /*-------------------------DFのeffectたちをリストアップしてグループ分け---------------------*/
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        effectDF = effectDF.Except(loopVertices).ToList();

                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        /*------------------------------------------------DFのグループ分け完了-----------------------------------------*/


                        if (xyYZinnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 1, false, true, false);
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, false, true, false); }
                                    else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, false, true, false); }
                                    else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, false, true, false); }
                                }
                            }
                            if (effectDoneBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDoneBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDoneBlocks[j][0];
                                    List<int> loopVertices2 = effectDoneBlocks[j][1];
                                    int loopIndex2 = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = DoneLength[loopIndex2];
                                    List<double> loopCos2 = DoneCos[loopIndex2];
                                    List<double> loopSin2 = DoneSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, false, true, false);
                                }
                            }

                            if (effectDtwoBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDtwoBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDtwoBlocks[j][0];
                                    List<int> loopVertices2 = effectDtwoBlocks[j][1];
                                    List<double> loopLength2 = new List<double>();
                                    List<double> loopCos2 = new List<double>();
                                    List<double> loopSin2 = new List<double>();
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, false, true, false);
                                }
                            }

                            if (effectFblocks.Count > 0)
                            {
                                for (int j = 0; j < effectFblocks.Count; j++)
                                {
                                    List<int> effectVertices = effectFblocks[j][0];
                                    List<int> loopVertices2 = effectFblocks[j][1];
                                    int loopIndex2 = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = FblocksLength[loopIndex2];
                                    List<double> loopCos2 = FblocksCos[loopIndex2];
                                    List<double> loopSin2 = FblocksSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, false, true, false);
                                }
                            }
                        }
                        else if (yzZXinnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 1, false, false, true);
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, false, false, true); }
                                    else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, false, false, true); }
                                    else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, false, false, true); }
                                }
                            }
                            if (effectDoneBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDoneBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDoneBlocks[j][0];
                                    List<int> loopVertices2 = effectDoneBlocks[j][1];
                                    int loopIndex2 = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = DoneLength[loopIndex2];
                                    List<double> loopCos2 = DoneCos[loopIndex2];
                                    List<double> loopSin2 = DoneSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, false, false, true);
                                }
                            }

                            if (effectDtwoBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDtwoBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDtwoBlocks[j][0];
                                    List<int> loopVertices2 = effectDtwoBlocks[j][1];
                                    List<double> loopLength2 = new List<double>();
                                    List<double> loopCos2 = new List<double>();
                                    List<double> loopSin2 = new List<double>();
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, false, false, true);
                                }
                            }

                            if (effectFblocks.Count > 0)
                            {
                                for (int j = 0; j < effectFblocks.Count; j++)
                                {
                                    List<int> effectVertices = effectFblocks[j][0];
                                    List<int> loopVertices2 = effectFblocks[j][1];
                                    int loopIndex2 = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = FblocksLength[loopIndex2];
                                    List<double> loopCos2 = FblocksCos[loopIndex2];
                                    List<double> loopSin2 = FblocksSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, false, false, true);
                                }
                            }
                        }
                        else if (zxXYinnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 1, true, false, false);
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, false, false); }
                                    else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, false, false); }
                                    else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, false, false); }
                                }
                            }
                            if (effectDoneBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDoneBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDoneBlocks[j][0];
                                    List<int> loopVertices2 = effectDoneBlocks[j][1];
                                    int loopIndex2 = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = DoneLength[loopIndex2];
                                    List<double> loopCos2 = DoneCos[loopIndex2];
                                    List<double> loopSin2 = DoneSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, true, false, false);
                                }
                            }

                            if (effectDtwoBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDtwoBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDtwoBlocks[j][0];
                                    List<int> loopVertices2 = effectDtwoBlocks[j][1];
                                    List<double> loopLength2 = new List<double>();
                                    List<double> loopCos2 = new List<double>();
                                    List<double> loopSin2 = new List<double>();
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, true, false, false);
                                }
                            }

                            if (effectFblocks.Count > 0)
                            {
                                for (int j = 0; j < effectFblocks.Count; j++)
                                {
                                    List<int> effectVertices = effectFblocks[j][0];
                                    List<int> loopVertices2 = effectFblocks[j][1];
                                    int loopIndex2 = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = FblocksLength[loopIndex2];
                                    List<double> loopCos2 = FblocksCos[loopIndex2];
                                    List<double> loopSin2 = FblocksSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, true, false, false);
                                }
                            }
                        }
                        else
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 1, true, true, true);
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, true, true); }
                                    else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, true, true); }
                                    else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, true, true); }
                                }
                            }
                            if (effectDoneBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDoneBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDoneBlocks[j][0];
                                    List<int> loopVertices2 = effectDoneBlocks[j][1];
                                    int loopIndex2 = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = DoneLength[loopIndex2];
                                    List<double> loopCos2 = DoneCos[loopIndex2];
                                    List<double> loopSin2 = DoneSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, true, true, true);
                                }
                            }

                            if (effectDtwoBlocks.Count > 0)
                            {
                                for (int j = 0; j < effectDtwoBlocks.Count; j++)
                                {
                                    List<int> effectVertices = effectDtwoBlocks[j][0];
                                    List<int> loopVertices2 = effectDtwoBlocks[j][1];
                                    List<double> loopLength2 = new List<double>();
                                    List<double> loopCos2 = new List<double>();
                                    List<double> loopSin2 = new List<double>();
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, true, true, true);
                                }
                            }

                            if (effectFblocks.Count > 0)
                            {
                                for (int j = 0; j < effectFblocks.Count; j++)
                                {
                                    List<int> effectVertices = effectFblocks[j][0];
                                    List<int> loopVertices2 = effectFblocks[j][1];
                                    int loopIndex2 = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                    List<double> loopLength2 = FblocksLength[loopIndex2];
                                    List<double> loopCos2 = FblocksCos[loopIndex2];
                                    List<double> loopSin2 = FblocksSin[loopIndex2];
                                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, true, true, true);
                                }
                            }
                        }
                    }

                    else if (F.Contains(vertGroup[0]))
                    {
                        List<int> loopVertices = Fblocks.FirstOrDefault(sublist => sublist.Contains(vertGroup[0]));
                        int loopIndex = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                        List<double> loopLength = FblocksLength[loopIndex];
                        List<double> loopCos = FblocksCos[loopIndex];
                        List<double> loopSin = FblocksSin[loopIndex];

                        /*-------------------------DFのeffectたちをリストアップしてグループ分け---------------------*/
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (D.Contains(effectVert)) { effectDF.Add(effectVert); }
                                else if (F.Contains(effectVert)) { effectDF.Add(effectVert); }
                            }
                        }
                        effectDF = effectDF.Except(loopVertices).ToList();

                        //D,Fをブロックで分けて処理、D,FがvertIndexならvertIndexのグループのインデックスをeffectD,Fから省く
                        List<List<List<int>>> effectDoneBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectDtwoBlocks = new List<List<List<int>>>();
                        List<List<List<int>>> effectFblocks = new List<List<List<int>>>();

                        while (effectDF.Count > 0)
                        {
                            int a = effectDF[0];
                            List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDoneBlocks.Add(new List<List<int>>());
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int> { a });
                                effectDoneBlocks[effectDoneBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDoneBlocks[effectDoneBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = DtwoMirror.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectDtwoBlocks.Add(new List<List<int>>());
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int> { a });
                                effectDtwoBlocks[effectDtwoBlocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectDtwoBlocks[effectDtwoBlocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }

                            found = Fblocks.FirstOrDefault(sub => sub.Contains(a));
                            if (found != null)
                            {
                                effectFblocks.Add(new List<List<int>>());
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int> { a });
                                effectFblocks[effectFblocks.Count - 1].Add(new List<int>(found));
                                effectDF.RemoveAt(0);
                                List<int> intersect = found.Intersect(effectDF).ToList();
                                if (intersect.Count > 0) { effectFblocks[effectFblocks.Count - 1][0].AddRange(intersect); effectDF = effectDF.Except(intersect).ToList(); }
                                continue;
                            }
                        }
                        /*------------------------------------------------DFのグループ分け完了-----------------------------------------*/

                        Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 3, true, true, true);
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, true, true, true); }
                                else if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, true, true, true); }
                                else if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, true, true, true); }
                            }
                        }
                        if (effectDoneBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDoneBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDoneBlocks[j][0];
                                List<int> loopVertices2 = effectDoneBlocks[j][1];
                                int loopIndex2 = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                List<double> loopLength2 = DoneLength[loopIndex2];
                                List<double> loopCos2 = DoneCos[loopIndex2];
                                List<double> loopSin2 = DoneSin[loopIndex2];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, true, true, true);
                            }
                        }

                        if (effectDtwoBlocks.Count > 0)
                        {
                            for (int j = 0; j < effectDtwoBlocks.Count; j++)
                            {
                                List<int> effectVertices = effectDtwoBlocks[j][0];
                                List<int> loopVertices2 = effectDtwoBlocks[j][1];
                                List<double> loopLength2 = new List<double>();
                                List<double> loopCos2 = new List<double>();
                                List<double> loopSin2 = new List<double>();
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, true, true, true);
                            }
                        }

                        if (effectFblocks.Count > 0)
                        {
                            for (int j = 0; j < effectFblocks.Count; j++)
                            {
                                List<int> effectVertices = effectFblocks[j][0];
                                List<int> loopVertices2 = effectFblocks[j][1];
                                int loopIndex2 = FblocksLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices2[0]);
                                List<double> loopLength2 = FblocksLength[loopIndex2];
                                List<double> loopCos2 = FblocksCos[loopIndex2];
                                List<double> loopSin2 = FblocksSin[loopIndex2];
                                Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, true, true, true);
                            }
                        }

                    }
                }
                /*----------------------------------------------------Jacobiの算出終了-------------------------------------------*/

                //あとはJacobiから共役勾配法にしたがって点を移動させていく
                double betaUpper = 0;
                double betaLower = 0;

                for (int i = 0; i < dupVertCount; i++)
                {
                    betaLower += preJacobi[i].SquareLength;
                    betaUpper += (Jacobi[i] - preJacobi[i]) * Jacobi[i];
                }

                Vector3d[] p = new Vector3d[dupVertCount];
                for (int i = 0; i < dupVertCount; i++)
                {
                    p[i] = -Jacobi[i] + betaUpper * preP[i] / betaLower;
                    for (int j = 0; j < duplicatedVertIndices[i].Count; j++)
                    {
                        mesh.Vertices[duplicatedVertIndices[i][j]] += alpha * p[i];
                    }
                    preP[i] = p[i];
                    preJacobi[i] = Jacobi[i];
                }
            }
            return mesh;
        }
    }
}