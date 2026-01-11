using Cloo.Bindings;
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
using System.Security.Policy;
using System.Xml.Linq;
using static Hagoromo.DevelopableMesh.CGNR3D;
using static Hagoromo.DevelopableMesh.DevelopableTools;
using static Hagoromo.GeometryTools.CutMeshCalcTools;
using static Hagoromo.GeometryTools.MeshCalcTools;
using static Hagoromo.GeometryTools.MeshCutTools;

namespace Hagoromo.DevelopableMesh
{
    public static class DevelopableTools
    {

        //--------------------------------------------------切込み非対応 最急降下法----------------------------------------------------
        
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
        
        //--------------------------------------------------切込み非対応 最急降下法　終わり----------------------------------------------------



        //--------------------------------------------------切込み非対応 共役勾配法----------------------------------------------------
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
                newTopoVertices[internalVertexIndices[i]] += 5000 * nextp[i];
                p[i] = nextp[i];
                Jacobi[i] = nextJacobi[i];
            }
        }
        
        //--------------------------------------------------切込み非対応 共役勾配法　終わり----------------------------------------------------




        //--------------------------------------------------ミラー、切込みのための準備----------------------------------------------------
        
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

        //共役勾配法で次のステップのメッシュを得る。ミラーに対応、ミラーはx,y,zのそれぞれmirror面に乗ってる点が多いほうの平面を軸とする
        //xyIndicesはxyMirrorするときのみ機能し、そのindicesの点は展開図でミラーとつながっていなくて良いとする。重なっている点の場合はDuplicatedの[0]のもののみ含む
        //A(つまりouterBoundaryVertIndices)はミラーして最終的に外側の境界となる部分の点のmesh.Verticesにおけるindices
        //A～FはDuplicatedしていてもそのすべての点が含まれている、入力するmeshはsortしておく
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

            //FからFblocksの作成
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

        public static List<List<List<int>>> CategolizeMulti(CutMesh mesh, bool xyMirror, bool yzMirror, bool zxMirror)
        {
            List<MeshSplitResult> meshSplit = SplitIntoConnectedComponents2(mesh);
            List<int> newA = new List<int>();
            List<int> newB = new List<int>();
            List<int> newC = new List<int>();
            List<int> newD = new List<int>();
            List<int> newE = new List<int>();
            List<int> newF = new List<int>();
            List<int> newAxy = new List<int>();
            List<int> newAyz = new List<int>();
            List<int> newAzx = new List<int>();
            List<int> newAfree = new List<int>();
            List<int> newBx = new List<int>();
            List<int> newBy = new List<int>();
            List<int> newBz = new List<int>();
            List<int> newCxy = new List<int>();
            List<int> newCyz = new List<int>();
            List<int> newCzx = new List<int>();
            List<List<int>> newDone = new List<List<int>>();
            List<List<int>> newXY = new List<List<int>>();
            List<List<int>> newYZ = new List<List<int>>();
            List<List<int>> newZX = new List<List<int>>();
            List<List<int>> newDtwo = new List<List<int>>();
            List<List<int>> newXYYZ = new List<List<int>>();
            List<List<int>> newYZZX = new List<List<int>>();
            List<List<int>> newZXXY = new List<List<int>>();
            List<List<int>> newFblocks = new List<List<int>>();

            foreach (MeshSplitResult meshData in meshSplit)
            {
                CutMesh cutMesh = meshData.Mesh;
                List<int> map = meshData.VertexMap;
                List<int> outerBoundary = FindOuterBoundaryVerts(cutMesh).OuterVerts;
                List<List<List<int>>> category = CategolizeCutMesh(cutMesh, xyMirror, yzMirror, zxMirror, outerBoundary, null);
                foreach (int vert in category[0][0]) { newA.Add(map[vert]); }
                foreach (int vert in category[0][1]) { newB.Add(map[vert]); }
                foreach (int vert in category[0][2]) { newC.Add(map[vert]); }
                foreach (int vert in category[0][3]) { newD.Add(map[vert]); }
                foreach (int vert in category[0][4]) { newE.Add(map[vert]); }
                foreach (int vert in category[0][5]) { newF.Add(map[vert]); }
                foreach (int vert in category[0][6]) { newAxy.Add(map[vert]); }
                foreach (int vert in category[0][7]) { newAyz.Add(map[vert]); }
                foreach (int vert in category[0][8]) { newAzx.Add(map[vert]); }
                foreach (int vert in category[0][9]) { newAfree.Add(map[vert]); }
                foreach (int vert in category[0][10]) { newBx.Add(map[vert]); }
                foreach (int vert in category[0][11]) { newBy.Add(map[vert]); }
                foreach (int vert in category[0][12]) { newBz.Add(map[vert]); }
                foreach (int vert in category[0][13]) { newCxy.Add(map[vert]); }
                foreach (int vert in category[0][14]) { newCyz.Add(map[vert]); }
                foreach (int vert in category[0][15]) { newCzx.Add(map[vert]); }

                foreach (List<int> loop in category[1])
                {
                    List<int> newLoop = new List<int>();
                    foreach (int vert in loop) { newLoop.Add(map[vert]); }
                    newLoop.Sort();
                    newDone.Add(newLoop);
                }
                foreach (List<int> loop in category[2])
                {
                    List<int> newLoop = new List<int>();
                    foreach (int vert in loop) { newLoop.Add(map[vert]); }
                    newLoop.Sort();
                    newXY.Add(newLoop);
                }
                foreach (List<int> loop in category[3])
                {
                    List<int> newLoop = new List<int>();
                    foreach (int vert in loop) { newLoop.Add(map[vert]); }
                    newLoop.Sort();
                    newYZ.Add(newLoop);
                }
                foreach (List<int> loop in category[4])
                {
                    List<int> newLoop = new List<int>();
                    foreach (int vert in loop) { newLoop.Add(map[vert]); }
                    newLoop.Sort();
                    newZX.Add(newLoop);
                }
                foreach (List<int> loop in category[5])
                {
                    List<int> newLoop = new List<int>();
                    foreach (int vert in loop) { newLoop.Add(map[vert]); }
                    newLoop.Sort();
                    newDtwo.Add(newLoop);
                }
                foreach (List<int> loop in category[6])
                {
                    List<int> newLoop = new List<int>();
                    foreach (int vert in loop) { newLoop.Add(map[vert]); }
                    newLoop.Sort();
                    newXYYZ.Add(newLoop);
                }
                foreach (List<int> loop in category[7])
                {
                    List<int> newLoop = new List<int>();
                    foreach (int vert in loop) { newLoop.Add(map[vert]); }
                    newLoop.Sort();
                    newYZZX.Add(newLoop);
                }
                foreach (List<int> loop in category[8])
                {
                    List<int> newLoop = new List<int>();
                    foreach (int vert in loop) { newLoop.Add(map[vert]); }
                    newLoop.Sort();
                    newZXXY.Add(newLoop);
                }
                List<List<int>> Fblocks = category[9];
                foreach (List<int> loop in Fblocks)
                {
                    List<int> newLoop = new List<int>();
                    foreach (int vert in loop) { newLoop.Add(map[vert]); }
                    newLoop.Sort();
                    newFblocks.Add(newLoop);
                }
            }
            newA.Sort();
            newB.Sort();
            newC.Sort();
            newD.Sort();
            newE.Sort();
            newF.Sort();
            newAxy.Sort();
            newAyz.Sort();
            newAzx.Sort();
            newAfree.Sort();
            newBx.Sort();
            newBy.Sort();
            newBz.Sort();
            newCxy.Sort();
            newCyz.Sort();
            newCzx.Sort();
            List<List<int>> flatten = new List<List<int>> { newA, newB, newC, newD, newE, newF, newAxy, newAyz, newAzx, newAfree, newBx, newBy, newBz, newCxy, newCyz, newCzx };
            List<List<List<int>>> result = new List<List<List<int>>> { flatten, newDone, newXY, newYZ, newZX, newDtwo, newXYYZ, newYZZX, newZXXY, newFblocks };
            return result;
        }



        //DFのeffectたちをリストアップしてグループ分け
        public static (List<List<List<int>>> effectDoneBlocks, List<List<List<int>>> effectDtwoBlocks, List<List<List<int>>> effectFblocks) DFGrouping
            (CutMesh mesh, List<int> vertGroup, List<List<List<int>>> category, List<int> loopVertices)
        {
            List<int> D = category[0][3];
            List<int> F = category[0][5];
            List<List<int>> DoneMirror = category[1];
            List<List<int>> DtwoMirror = category[5];
            List<List<int>> Fblocks = category[9];

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
            return (effectDoneBlocks,effectDtwoBlocks,effectFblocks);
        }


        //--------------------------------------------------ミラー、切込みのための準備　終わり----------------------------------------------------



        //--------------------------------------------------ミラー、切込み対応　共役勾配法----------------------------------------------------

        //目的関数自体の値を返す。どんな式かはDevelop Mesh2のpdfのP6を参照
        public static double ObjectiveCalc(List<List<int>> duplicatedVertIndices, List<Point3d> vertices, double[] angleSum, List<List<List<int>>> category)
        {
            List<int> B = category[0][1];
            List<int> C = category[0][2];
            List<int> E = category[0][4];
            List<List<int>> DoneMirror = category[1];
            List<List<int>> DtwoMirror = category[5];
            List<List<int>> Fblocks = category[9];

            double obj = 0;
            while (B.Count > 0)
            {
                int vert = B[0];
                double a = 2 * Math.PI - 4 * angleSum[vert];
                obj += a * a;
                List<int> found = duplicatedVertIndices.FirstOrDefault(sublist => sublist.Contains(vert));
                B.RemoveAt(0);
                if (found != null) { B = B.Except(found).ToList(); }
            }

            while (C.Count > 0)
            {
                int vert = C[0];
                double a = 2 * Math.PI - 2 * angleSum[vert];
                obj += a * a;
                List<int> found = duplicatedVertIndices.FirstOrDefault(sublist => sublist.Contains(vert));
                C.RemoveAt(0);
                if (found != null) { C = C.Except(found).ToList(); }
            }

            while (E.Count > 0)
            {
                int vert = E[0];
                double a = 2 * Math.PI - angleSum[vert];
                obj += a * a;
                List<int> found = duplicatedVertIndices.FirstOrDefault(sublist => sublist.Contains(vert));
                E.RemoveAt(0);
                if (found != null) { E = E.Except(found).ToList(); }
            }

            foreach (List<int> Dtwo in DtwoMirror)
            {
                int n = Dtwo.Count;
                double sum = 0;
                foreach (int v in Dtwo)
                {
                    sum += angleSum[v];
                }
                double a = n * Math.PI - sum;
                obj += 16 * a * a / n;
            }

            foreach (List<int> Done in DoneMirror)
            {
                int n = Done.Count;
                double sum = 0;
                foreach (int v in Done)
                {
                    sum += angleSum[v];
                }
                double a = n * Math.PI - sum;
                obj += 4 * a * a / n;

                List<double> loopLength = new List<double> { Done[0] };
                List<double> loopCos = new List<double>();
                List<double> loopSin = new List<double>();
                double angleSumSum = 0;
                for (int j = 0; j < Done.Count - 1; j++)
                {
                    double length = (vertices[Done[j]] - vertices[Done[j + 1]]).Length;
                    angleSumSum += angleSum[Done[j]];
                    loopLength.Add(length);
                    loopCos.Add(Math.Cos(angleSumSum));
                    loopSin.Add(Math.Sin(angleSumSum));
                }
                int count = Done.Count;
                double averageLength = 0;
                for (int ii = 1; ii < loopLength.Count; ii++)
                {
                    averageLength += loopLength[ii];
                }
                averageLength = averageLength / (n - 1);
                double s = 0;
                for (int j = 0; j < count - 1; j++)
                {
                    if (j % 2 == 0) { s += loopLength[j + 1] * loopSin[j]; }
                    else { s -= loopLength[j + 1] * loopSin[j]; }
                }
                obj += 4 * n * Math.PI * s * s / (averageLength * averageLength);
            }

            foreach (List<int> Fblock in Fblocks)
            {
                int n = Fblock.Count;
                double sum = 0;
                foreach (int v in Fblock)
                {
                    sum += angleSum[v];
                }
                double a = (n + 2) * Math.PI - sum;
                obj += 4 * a * a / (3 * n);

                List<double> loopLength = new List<double> { Fblock[0] };
                List<double> loopCos = new List<double>();
                List<double> loopSin = new List<double>();
                double angleSumSum = 0;
                int loopVerticesCount = Fblock.Count;
                for (int j = 0; j < loopVerticesCount; j++)
                {
                    double length = (vertices[Fblock[j]] - vertices[Fblock[(j + 1) % loopVerticesCount]]).Length;
                    loopLength.Add(length);
                    if (j == 0) { continue; }
                    angleSumSum += angleSum[Fblock[j]];
                    loopCos.Add(Math.Cos(angleSumSum));
                    loopSin.Add(Math.Sin(angleSumSum));
                }

                int count = Fblock.Count;
                double averageLength = 0;
                for (int ii = 1; ii < loopLength.Count; ii++)
                {
                    averageLength += loopLength[ii];
                }
                averageLength = averageLength / n;

                double s = 0;
                for (int j = 1; j < count; j++)
                {
                    if (j % 2 == 0) { s += loopLength[j + 1] * loopSin[j - 1]; }
                    else { s -= loopLength[j + 1] * loopSin[j - 1]; }
                }

                double c = loopLength[1];
                for (int j = 1; j < count; j++)
                {
                    if (j % 2 == 0) { c += loopLength[j + 1] * loopCos[j - 1]; }
                    else { c -= loopLength[j + 1] * loopCos[j - 1]; }
                }

                obj += 4 * n * Math.PI * (s * s + c * c) / (3 * averageLength * averageLength);
            }

            return obj;
        }

        //共役勾配法で最適化、ミラー、切込みに対応
        public static CutMesh CGDevCutMeshConsiderOther(CutMesh mesh, int iterations, double alpha, bool xyMirror, bool yzMirror, bool zxMirror, List<int> A, List<int> fixIndices)
        {
            /*----------------------------------------------------A～Fの分類-----------------------------------------------------------------------*/
            List<List<List<int>>> category = CategolizeMulti(mesh, xyMirror, yzMirror, zxMirror/*, A, fixIndices*/);
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
                var loopData = CalcLoopData(mesh, angleSum, DoneMirror, Fblocks);
                List<List<double>> DoneLength = loopData.DoneLength;
                List<List<double>> DoneCos = loopData.DoneCos;
                List<List<double>> DoneSin = loopData.DoneSin;
                List<List<double>> FblocksLength = loopData.FblocksLength;
                List<List<double>> FblocksCos = loopData.FblocksCos;
                List<List<double>> FblocksSin = loopData.FblocksSin;

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
                        CalcEffectOfABCEVert(Jacobi, i, mesh, vertGroup, category, loopData, angleSum, true, true, false);
                    }

                    else if (Ayz.Contains(vertGroup[0]))
                    {
                        CalcEffectOfABCEVert(Jacobi, i, mesh, vertGroup, category, loopData, angleSum, false, true, true);
                    }

                    else if (Azx.Contains(vertGroup[0]))
                    {
                        CalcEffectOfABCEVert(Jacobi, i, mesh, vertGroup, category, loopData, angleSum, true, false, true);
                    }

                    else if (Afree.Contains(vertGroup[0]))
                    {
                        CalcEffectOfABCEVert(Jacobi, i, mesh, vertGroup, category, loopData, angleSum, true, true, true);
                    }
                    
                    else if (Bx.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 8, true, false, false);
                        CalcEffectOfABCEVert(Jacobi, i, mesh, vertGroup, category, loopData, angleSum, true, false, false);
                    }

                    else if (By.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 8, false, true, false);
                        CalcEffectOfABCEVert(Jacobi, i, mesh, vertGroup, category, loopData, angleSum, false, true, false);
                    }

                    else if (Bz.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 8, false, false, true);
                        CalcEffectOfABCEVert(Jacobi, i, mesh, vertGroup, category, loopData, angleSum, false, false, true);
                    }

                    else if (Cxy.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 4, true, true, false);
                        CalcEffectOfABCEVert(Jacobi, i, mesh, vertGroup, category, loopData, angleSum, true, true, false);
                    }

                    else if (Cyz.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 4, false, true, true);
                        CalcEffectOfABCEVert(Jacobi, i, mesh, vertGroup, category, loopData, angleSum, false, true, true);
                    }

                    else if (Czx.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 4, true, false, true);
                        CalcEffectOfABCEVert(Jacobi, i, mesh, vertGroup, category, loopData, angleSum, true, false, true);
                    }

                    else if (E.Contains(vertGroup[0]))
                    {
                        Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertGroup[0], vertGroup[0], 2, true, true, true);
                        CalcEffectOfABCEVert(Jacobi, i, mesh, vertGroup, category, loopData, angleSum, true, true, true);
                    }

                    else if (DoneMirror.Any(sub => sub.Contains(vertGroup[0])))
                    {
                        List<int> loopVertices = DoneMirror.FirstOrDefault(sublist => sublist.Contains(vertGroup[0]));
                        int loopIndex = DoneLength.FindIndex(sub => sub.Count > 0 && sub[0] == loopVertices[0]);
                        List<double> loopLength = DoneLength[loopIndex];
                        List<double> loopCos = DoneCos[loopIndex];
                        List<double> loopSin = DoneSin[loopIndex];

                        //-------------------------DFのeffectたちをリストアップしてグループ分け---------------------
                        var DFGroup = DFGrouping(mesh, vertGroup, category, loopVertices);
                        List<List<List<int>>> effectDoneBlocks = DFGroup.effectDoneBlocks;
                        List< List < List<int> >> effectDtwoBlocks = DFGroup.effectDtwoBlocks;
                        List<List<List<int>>> effectFblocks = DFGroup.effectFblocks;
                        /*------------------------------------------------DFのグループ分け完了-----------------------------------------*/


                        if (xyMirrorInnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 2, true, true, false);
                            CalcEffectToBCE(Jacobi, i, mesh, vertGroup, category, angleSum, true, true, false);
                            CalcEffectToDF(Jacobi, i, mesh, vertGroup, angleSum, effectDoneBlocks, effectDtwoBlocks, effectFblocks, loopData, true, true, false);
                        }
                        else if (yzMirrorInnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 2, false, true, true);
                            CalcEffectToBCE(Jacobi, i, mesh, vertGroup, category, angleSum, false, true, true);
                            CalcEffectToDF(Jacobi, i, mesh, vertGroup, angleSum, effectDoneBlocks, effectDtwoBlocks, effectFblocks, loopData, false, true, true);
                        }
                        else if (zxMirrorInnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 2, true, false, true);
                            CalcEffectToBCE(Jacobi, i, mesh, vertGroup, category, angleSum, true, false, true);
                            CalcEffectToDF(Jacobi, i, mesh, vertGroup, angleSum, effectDoneBlocks, effectDtwoBlocks, effectFblocks, loopData, true, false, true);
                        }
                        else
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 2, true, true, true);
                            CalcEffectToBCE(Jacobi, i, mesh, vertGroup, category, angleSum, true, true, true);
                            CalcEffectToDF(Jacobi, i, mesh, vertGroup, angleSum, effectDoneBlocks, effectDtwoBlocks, effectFblocks, loopData, true, true, true);
                        }
                    }

                    else if (DtwoMirror.Any(sub => sub.Contains(vertGroup[0])))
                    {
                        List<int> loopVertices = DtwoMirror.FirstOrDefault(sublist => sublist.Contains(vertGroup[0]));
                        List<double> loopLength = new List<double>();
                        List<double> loopCos = new List<double>();
                        List<double> loopSin = new List<double>();

                        /*-------------------------DFのeffectたちをリストアップしてグループ分け---------------------*/
                        var DFGroup = DFGrouping(mesh, vertGroup, category, loopVertices);
                        List<List<List<int>>> effectDoneBlocks = DFGroup.effectDoneBlocks;
                        List<List<List<int>>> effectDtwoBlocks = DFGroup.effectDtwoBlocks;
                        List<List<List<int>>> effectFblocks = DFGroup.effectFblocks;
                        /*------------------------------------------------DFのグループ分け完了-----------------------------------------*/


                        if (xyYZinnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 1, false, true, false);
                            CalcEffectToBCE(Jacobi, i, mesh, vertGroup, category, angleSum, false, true, false);
                            CalcEffectToDF(Jacobi, i, mesh, vertGroup, angleSum, effectDoneBlocks, effectDtwoBlocks, effectFblocks, loopData, false, true, false);
                        }
                        else if (yzZXinnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 1, false, false, true);
                            CalcEffectToBCE(Jacobi, i, mesh, vertGroup, category, angleSum, false, false, true);
                            CalcEffectToDF(Jacobi, i, mesh, vertGroup, angleSum, effectDoneBlocks, effectDtwoBlocks, effectFblocks, loopData, false, false, true);
                        }
                        else if (zxXYinnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 1, true, false, false);
                            CalcEffectToBCE(Jacobi, i, mesh, vertGroup, category, angleSum, true, false, false);
                            CalcEffectToDF(Jacobi, i, mesh, vertGroup, angleSum, effectDoneBlocks, effectDtwoBlocks, effectFblocks, loopData, true, false, false);
                        }
                        else
                        {
                            Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 1, true, true, true);
                            CalcEffectToBCE(Jacobi, i, mesh, vertGroup, category, angleSum, true, true, true);
                            CalcEffectToDF(Jacobi, i, mesh, vertGroup, angleSum, effectDoneBlocks, effectDtwoBlocks, effectFblocks, loopData, true, true, true);
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
                        var DFGroup = DFGrouping(mesh, vertGroup, category, loopVertices);
                        List<List<List<int>>> effectDoneBlocks = DFGroup.effectDoneBlocks;
                        List<List<List<int>>> effectDtwoBlocks = DFGroup.effectDtwoBlocks;
                        List<List<List<int>>> effectFblocks = DFGroup.effectFblocks;
                        /*------------------------------------------------DFのグループ分け完了-----------------------------------------*/

                        Jacobi[i] += VertGroupDFJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 3, true, true, true);
                        CalcEffectToBCE(Jacobi, i, mesh, vertGroup, category, angleSum, true, true, true);
                        CalcEffectToDF(Jacobi, i, mesh, vertGroup, angleSum, effectDoneBlocks, effectDtwoBlocks, effectFblocks, loopData, true, true, true);

                    }
                }
                /*----------------------------------------------------Jacobiの算出終了-------------------------------------------*/

                //あとはJacobiから共役勾配法にしたがって点を移動させていく
                double betaUpper = 0;
                double betaLower = 0;

                for (int i = 0; i < dupVertCount; i++)
                {
                    betaLower += preJacobi[i].SquareLength;
                    //betaLower += (Jacobi[i] - preJacobi[i]) * preP[i];
                    betaUpper += (Jacobi[i] - preJacobi[i]) * Jacobi[i];
                }

                Vector3d[] p = new Vector3d[dupVertCount];

                double alpha2 = alpha;

                //--------------------------------タイプA：α固定------------------------------------
                /*
                for (int i = 0; i < dupVertCount; i++)
                {
                    p[i] = -Jacobi[i] + betaUpper * preP[i] / betaLower;
                }
                */
                //--------------------------------タイプA：α固定 終了------------------------------------


                //--------------------------------タイプB：α探索------------------------------------

                var lineSearch = LineSearch(mesh, category, angleSum, Jacobi, p, preP, betaUpper, betaLower, alpha);
                alpha2 = lineSearch.alpha2;
                p = lineSearch.p;
                //--------------------------------タイプB：α探索 終了------------------------------------

                for (int i = 0; i < dupVertCount; i++)
                {
                    for (int j = 0; j < duplicatedVertIndices[i].Count; j++)
                    {
                        mesh.Vertices[duplicatedVertIndices[i][j]] += alpha2 * p[i];
                    }
                    preP[i] = p[i];
                    preJacobi[i] = Jacobi[i];
                }
            }
            return mesh;
        }

        //共役勾配法で最適化、Jacobiでselfのみ考慮。ミラー、切込みに対応
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
                var loopData = CalcLoopData(mesh, angleSum, DoneMirror, Fblocks);
                List<List<double>> DoneLength = loopData.DoneLength;
                List<List<double>> DoneCos = loopData.DoneCos;
                List<List<double>> DoneSin = loopData.DoneSin;
                List<List<double>> FblocksLength = loopData.FblocksLength;
                List<List<double>> FblocksCos = loopData.FblocksCos;
                List<List<double>> FblocksSin = loopData.FblocksSin;

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
                        var DFGroup = DFGrouping(mesh, vertGroup, category, loopVertices);
                        List<List<List<int>>> effectDoneBlocks = DFGroup.effectDoneBlocks;
                        List<List<List<int>>> effectDtwoBlocks = DFGroup.effectDtwoBlocks;
                        List<List<List<int>>> effectFblocks = DFGroup.effectFblocks;
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
                        var DFGroup = DFGrouping(mesh, vertGroup, category, loopVertices);
                        List<List<List<int>>> effectDoneBlocks = DFGroup.effectDoneBlocks;
                        List<List<List<int>>> effectDtwoBlocks = DFGroup.effectDtwoBlocks;
                        List<List<List<int>>> effectFblocks = DFGroup.effectFblocks;
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
                        var DFGroup = DFGrouping(mesh, vertGroup, category, loopVertices);
                        List<List<List<int>>> effectDoneBlocks = DFGroup.effectDoneBlocks;
                        List<List<List<int>>> effectDtwoBlocks = DFGroup.effectDtwoBlocks;
                        List<List<List<int>>> effectFblocks = DFGroup.effectFblocks;
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

                double alpha2 = alpha;

                //--------------------------------タイプA：α固定------------------------------------
                /*
                for (int i = 0; i < dupVertCount; i++)
                {
                    p[i] = -Jacobi[i] + betaUpper * preP[i] / betaLower;
                }
                */
                //--------------------------------タイプA：α固定 終了------------------------------------


                //--------------------------------タイプB：α探索------------------------------------

                var lineSearch = LineSearch(mesh, category, angleSum, Jacobi, p, preP, betaUpper, betaLower, alpha);
                alpha2 = lineSearch.alpha2;
                p = lineSearch.p;
                //--------------------------------タイプB：α探索 終了------------------------------------

                for (int i = 0; i < dupVertCount; i++)
                {
                    for (int j = 0; j < duplicatedVertIndices[i].Count; j++)
                    {
                        mesh.Vertices[duplicatedVertIndices[i][j]] += alpha2 * p[i];
                    }
                    preP[i] = p[i];
                    preJacobi[i] = Jacobi[i];
                }
            }
            return mesh;
        }

        //Done、Fblocksにおいて制約のあるsin,cosのJacobi算出のための情報をあらかじめ計算しておく関数
        public static (List<List<double>> DoneLength, List<List<double>> DoneCos, List<List<double>> DoneSin, List<List<double>> FblocksLength, List<List<double>> FblocksCos, List<List<double>> FblocksSin) CalcLoopData(CutMesh mesh, double[] angleSum, List<List<int>> DoneMirror, List<List<int>> Fblocks)
        {
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
            return (DoneLength,DoneCos,DoneSin,FblocksLength,FblocksCos,FblocksSin);
        }

        //ABCEの点(のx,y,z、Vec3d Jacobiの[i]に対応)による、ほかの点、ループの項の微分値をJacobiのi番目に足す。
        public static void CalcEffectOfABCEVert(Vector3d[] Jacobi, int i, CutMesh mesh, List<int> vertGroup,
            List<List<List<int>>> category,
            (List<List<double>> DoneLength,List<List<double>> DoneCos,List<List<double>> DoneSin,
            List<List<double>> FblocksLength,List<List<double>> FblocksCos,List<List<double>> FblocksSin) loopData, 
            double[] angleSum, bool x, bool y, bool z)
        {
            List<int> B = category[0][1];
            List<int> C = category[0][2];
            List<int> D = category[0][3];
            List<int> E = category[0][4];
            List<int> F = category[0][5];
            List<List<int>> DoneMirror = category[1];
            List<List<int>> DtwoMirror = category[5];
            List<List<int>> Fblocks = category[9];

            List<List<double>> DoneLength = loopData.DoneLength;
            List<List<double>> DoneCos = loopData.DoneCos;
            List<List<double>> DoneSin = loopData.DoneSin;
            List<List<double>> FblocksLength = loopData.FblocksLength;
            List<List<double>> FblocksCos = loopData.FblocksCos;
            List<List<double>> FblocksSin = loopData.FblocksSin;

            List<int> effectDF = new List<int>();
            foreach (int vertIndex in vertGroup)
            {
                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                foreach (int effectVert in connectedVertices)
                {
                    if (B.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 8, x, y, z); }
                    if (C.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 4, x, y, z); }
                    if (E.Contains(effectVert)) { Jacobi[i] += EffectToGroupBCEVertex(mesh, angleSum, vertIndex, effectVert, 2, x, y, z); }
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

            CalcEffectToDF(Jacobi, i, mesh, vertGroup, angleSum, effectDoneBlocks, effectDtwoBlocks, effectFblocks, loopData, x, y, z);
        }

        public static void CalcEffectToBCE(Vector3d[] Jacobi, int i, CutMesh mesh, List<int> vertGroup, List<List<List<int>>> category, 
            double[] angleSum,bool x, bool y, bool z)
        {
            List<int> B = category[0][1];
            List<int> C = category[0][2];
            List<int> E = category[0][4];
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
        }


        public static void CalcEffectToDF(Vector3d[] Jacobi, int i, CutMesh mesh, List<int> vertGroup, double[] angleSum,
            List<List<List<int>>> effectDoneBlocks, List<List<List<int>>> effectDtwoBlocks, List<List<List<int>>> effectFblocks,
            (List<List<double>> DoneLength, List<List<double>> DoneCos, List<List<double>> DoneSin,
            List<List<double>> FblocksLength, List<List<double>> FblocksCos, List<List<double>> FblocksSin) loopData,
            bool x, bool y, bool z)
        {
            List<List<double>> DoneLength = loopData.DoneLength;
            List<List<double>> DoneCos = loopData.DoneCos;
            List<List<double>> DoneSin = loopData.DoneSin;
            List<List<double>> FblocksLength = loopData.FblocksLength;
            List<List<double>> FblocksCos = loopData.FblocksCos;
            List<List<double>> FblocksSin = loopData.FblocksSin;
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
                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, x, y, z);
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
                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, x, y, z);
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
                    Jacobi[i] += EffectToGroupDFVertex(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, x, y, z);
                }
            }
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
            double calc = 0.5 * ratio * (ratio * angle - 4 * Math.PI);
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
            /*
            int k = 0;
            int ratio = 0;
            */
            List<Point3d> vertices = cutMesh.Vertices;
            double sX = 0;
            double sY = 0;
            double sZ = 0;

            /*
            if (pattern == 2) { k = 2 * n; ratio = 2; }
            else if (pattern == 1){ k = 4 * n - 2; ratio = 4; }
            else { k = n + 2; ratio = 1; }
            */

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
            double calc = 0;
            if (pattern == 1) { calc = -32 * (n * Math.PI - angle) / n; }
            if (pattern == 2) { calc = -8 * (n * Math.PI - angle) / n; }
            if (pattern == 3) { calc = -8 * ((n + 2) * Math.PI - angle) / (3 * n); }
            //double calc =(-2) * ratio * (k * Math.PI - ratio * angle)/ pattern;
            Jacobi.X = sX * calc;
            Jacobi.Y = sY * calc;
            Jacobi.Z = sZ * calc;
            //2方向ミラーの場合
            if (pattern == 1) { return Jacobi; }
            //1方向ミラーの場合
            else if (pattern == 2)
            {
                double averageLength = 0;
                for (int ii = 1; ii < loopLength.Count; ii++)
                {
                    averageLength += loopLength[ii];
                }
                averageLength = averageLength / (n - 1);
                int count = loopVertices.Count;//つまりnと同じ
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
                double value2 = s * count * 4 * Math.PI / (averageLength * averageLength);
                Jacobi.X += sumX * value2;
                Jacobi.Y += sumY * value2;
                Jacobi.Z += sumZ * value2;

            }
            //Ｆの場合
            else
            {
                double averageLength = 0;
                for (int ii = 1; ii < loopLength.Count; ii++)
                {
                    averageLength += loopLength[ii];
                }
                averageLength = averageLength / n;
                int count = loopVertices.Count;//つまりnと同じ
                double s = loopLength[1];
                for (int j = 1; j < count; j++)
                {
                    if (j % 2 == 0) { s += loopLength[j + 1] * loopCos[j - 1]; }
                    else { s -= loopLength[j + 1] * loopCos[j - 1]; }
                }

                //sinの項を足す
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
                    double value = loopLength[j + 1] * loopSin[j - 1];
                    if (moveX) { if (j % 2 == 0) { sumX -= value * tX; } else { sumX += value * tX; } }
                    if (moveY) { if (j % 2 == 0) { sumY -= value * tY; } else { sumY += value * tY; } }
                    if (moveZ) { if (j % 2 == 0) { sumZ -= value * tZ; } else { sumZ += value * tZ; } }
                }
                //double value2 = 2 * s * count * count * Math.PI * Math.PI / (3 * averageLength * averageLength);
                double value2 = 2 * s * 4 * count * Math.PI / (3 * averageLength * averageLength);
                Jacobi.X += sumX * value2;
                Jacobi.Y += sumY * value2;
                Jacobi.Z += sumZ * value2;

                //cosの項を足す
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
            /*
            double bairitu = 1000;
            Jacobi.X = Jacobi.X / bairitu;
            Jacobi.Y = Jacobi.Y / bairitu;
            Jacobi.Z = Jacobi.Z / bairitu;
            */
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
            List<Point3d> vertices = cutMesh.Vertices;
            double sX = 0;
            double sY = 0;
            double sZ = 0;

            if (pattern == 2) { k = 2 * n; }
            else if (pattern == 1) { k = 4 * n - 2; }
            else { k = n + 2; }

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
            double calc = 0;
            if (pattern == 1) { calc = -32 * (n * Math.PI - angle) / n; }
            if (pattern == 2) { calc = -8 * (n * Math.PI - angle) / n; }
            if (pattern == 3) { calc = -8 * ((n + 2) * Math.PI - angle) / (3 * n); }
            //double calc = 10 *(-2) * ratio * (k * Math.PI - ratio * angle)/(pattern);
            Jacobi.X = (sX + sX2) * calc;
            Jacobi.Y = (sY + sY2) * calc;
            Jacobi.Z = (sZ + sZ2) * calc;
            //2方向ミラーの場合
            if (pattern == 1) { return Jacobi; }
            //1方向ミラーの場合
            else if (pattern == 2)
            {
                double averageLength = 0;
                for (int ii = 1; ii < loopLength.Count; ii++)
                {
                    averageLength += loopLength[ii];
                }
                averageLength = averageLength / (n - 1);
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
                double tZ2 = 0;
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
                double value2 = s * count * 4 * Math.PI / (averageLength * averageLength);
                Jacobi.X += sumX * value2;
                Jacobi.Y += sumY * value2;
                Jacobi.Z += sumZ * value2;
            }
            //Ｆの場合
            else
            {
                double averageLength = 0;
                for (int ii = 1; ii < loopLength.Count; ii++)
                {
                    averageLength += loopLength[ii];
                }
                averageLength = averageLength / n;
                int count = loopVertices.Count;
                double s = 0;
                for (int j = 1; j < count; j++)
                {
                    if (j % 2 == 0) { s += loopLength[j + 1] * loopSin[j - 1]; }
                    else { s -= loopLength[j + 1] * loopSin[j - 1]; }
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
                double value2 = 2 * s * count * 4 * Math.PI / (3 * averageLength * averageLength);
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
            /*
            double bairitu = 1000;
            Jacobi.X = Jacobi.X / bairitu;
            Jacobi.Y = Jacobi.Y / bairitu;
            Jacobi.Z = Jacobi.Z / bairitu;
            */
            return Jacobi;
        }



        //line searchにより適切なalphaの値を求める
        public static (double alpha2, Vector3d[] p) LineSearch(CutMesh mesh, List<List<List<int>>> category, double[] angleSum,
            Vector3d[] Jacobi, Vector3d[] p, Vector3d[] preP, double betaUpper, double betaLower,double alpha)
        {
            List<List<int>> duplicatedVertIndices = mesh.DuplicatedVertIndices;
            int dupVertCount = duplicatedVertIndices.Count;
            for (int i = 0; i < dupVertCount; i++)
            {
                if (Math.Abs(betaUpper / betaLower) > 1000000)
                {
                    p[i] = -Jacobi[i];
                    p[i] = p[i] * alpha; //長さ平均が20位の時に0.1倍がちょうどいい。ここは要調整、これをalphaとしてもいいか
                }
                else
                {
                    p[i] = -Jacobi[i] + betaUpper * preP[i] / betaLower;
                    p[i] = p[i] * alpha; //長さ平均が20位の時に0.1倍がちょうどいい。ここは要調整、これをalphaとしてもいいか
                }
            }

            //ステップk内で点を移動する前の節点座標
            List<Point3d> vertices = new List<Point3d>(mesh.Vertices);
            List<Point3d> newVertices = new List<Point3d>(mesh.Vertices);
            double faiZero = ObjectiveCalc(duplicatedVertIndices, vertices, angleSum, category);
            double faiDashZero = 0;
            for (int i = 0; i < dupVertCount; i++)
            {
                faiDashZero += Jacobi[i] * p[i];
            }

            if (faiDashZero > 0)
            {
                faiDashZero = 0;
                for (int i = 0; i < dupVertCount; i++)
                {
                    p[i] = -Jacobi[i];
                    p[i] = p[i] * alpha;
                    faiDashZero += Jacobi[i] * p[i];
                }
            }
            double c1 = 0.0001;
            double alpha2 = 1.0;
            bool alphaFlag = true;
            while (alphaFlag)
            {
                for (int i = 0; i < dupVertCount; i++)
                {
                    for (int j = 0; j < duplicatedVertIndices[i].Count; j++)
                    {
                        newVertices[duplicatedVertIndices[i][j]] = vertices[duplicatedVertIndices[i][j]] + alpha2 * p[i];
                    }
                }
                double faiOne = ObjectiveCalc(duplicatedVertIndices, newVertices, angleSum, category);
                double rightValue = faiZero + c1 * alpha2 * faiDashZero;
                if (faiOne <= rightValue) { alphaFlag = false; }
                else { alpha2 = alpha2 * 0.5; }
                if (alpha2 < 0.0001) { alphaFlag = false; }
            }
            return (alpha2, p);
        }

        //--------------------------------------------------ミラー、切込み対応　共役勾配法　終わり----------------------------------------------------







        //-------------------------------------------------------------------------------CGNR法---------------------------------------------

        //craneと同様のやり方。developableを制約条件にする、wは各制約条件、エネルギー（最小化対象)の重みづけの値
        //ヤコビ行列はdevelopableの
        public static CutMesh CGNRConsiderOther(CutMesh mesh, int iterations, bool xyMirror, bool yzMirror, bool zxMirror, List<int> A, List<int> fixIndices,
            double[] w)
        {
            double w0 = w[0];
            double w1 = w[1];
            double w2 = w[2];
            double w3 = w[3];
            double w4 = w[4];
            double w5 = w[5];
            double w6 = w[6];
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
            int n = dupVertCount * 3;
            //ｍは制約条件+エネルギー関数の個数。m行の順番はdupVertの上から順、ループの時もループ内で一番最初に出てくるdupVertの位置を基準とする。
            //ループの時は、角度の和の条件、sinの条件、cosの条件の順とする
            int devConstraintCount = B.Count + C.Count + E.Count + DoneMirror.Count * 2 + DtwoMirror.Count + Fblocks.Count * 3;
            int edgeCount = mesh.Edges.Count;

            //エネルギー関数の数
            int energyCount = edgeCount;
            //developable以外の制約条件の数
            int constraintMoreCount = 0;
            int m = devConstraintCount + constraintMoreCount + energyCount;


            int[] constraintsOrder = new int[dupVertCount];
            for (int i = 0; i < constraintsOrder.Length; i++)
            {
                constraintsOrder[i] = -1;
            }
            int guide = 0;
            List<int> skipVert = new List<int>();
            int iii = 0;
            foreach (List<int> vertGroup in duplicatedVertIndices)
            {
                if (skipVert.Contains(vertGroup[0])) { iii += 1; continue; }
                if (B.Contains(vertGroup[0])) { constraintsOrder[iii] = guide; guide += 1; iii += 1; continue; }
                if (C.Contains(vertGroup[0])) { constraintsOrder[iii] = guide; guide += 1; iii += 1; continue; }
                if (E.Contains(vertGroup[0])) { constraintsOrder[iii] = guide; guide += 1; iii += 1; continue; }

                List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(vertGroup[0]));
                if (found != null)
                {
                    foreach (int f in found)
                    {
                        for (int i = 0; i < dupVertCount; i++)
                        {
                            if (duplicatedVertIndices[i].Contains(f))
                            {
                                constraintsOrder[i] = guide; // f は i 番目のサブリストにある
                                break;
                            }
                        }
                    }

                    guide += 2;
                    skipVert.AddRange(found);
                    iii += 1;
                    continue;
                }

                found = DtwoMirror.FirstOrDefault(sub => sub.Contains(vertGroup[0]));
                if (found != null)
                {
                    foreach (int f in found)
                    {
                        for (int i = 0; i < dupVertCount; i++)
                        {
                            if (duplicatedVertIndices[i].Contains(f))
                            {
                                constraintsOrder[i] = guide; // f は i 番目のサブリストにある
                                break;
                            }
                        }
                    }
                    guide += 1;
                    skipVert.AddRange(found);
                    iii += 1;
                    continue;
                }

                found = Fblocks.FirstOrDefault(sub => sub.Contains(vertGroup[0]));
                if (found != null)
                {
                    foreach (int f in found)
                    {
                        for (int i = 0; i < dupVertCount; i++)
                        {
                            if (duplicatedVertIndices[i].Contains(f))
                            {
                                constraintsOrder[i] = guide; // f は i 番目のサブリストにある
                                break;
                            }
                        }
                    }
                    guide += 3;
                    skipVert.AddRange(found);
                    iii += 1;
                    continue;
                }
                iii += 1;
            }

            //verticesのi番目の点がdupVertの何番目のグループの点なのかの対応のリスト
            int[] vertOrderInDup = new int[mesh.Vertices.Count];
            for (int i = 0; i < vertOrderInDup.Length; i++)
            {
                vertOrderInDup[i] = -1;
            }
            for (int i = 0; i < duplicatedVertIndices.Count; i++)
            {
                foreach (int vertIndex in duplicatedVertIndices[i])
                {
                    vertOrderInDup[vertIndex] = i; 
                }
            }

            //verticesのi番目の点がconstraintsの何行目(～何行目)なのかの対応のリスト
            int[] vertToConstraints = new int[mesh.Vertices.Count];
            for (int i = 0; i < vertToConstraints.Length; i++)
            {
                vertToConstraints[i] = -1;
            }
            for (int i = 0; i < mesh.Vertices.Count; i++)
            {
                vertToConstraints[i] = constraintsOrder[vertOrderInDup[i]];
            }

            double[] initialLength = CutMeshCalcTools.EdgeLength(mesh);

            /*------------------Jacobiによる点の移動で変わっていくのは以下の部分------------*/
            for (int u = 0; u < iterations; u++)
            {
                double[] currentLength = CutMeshCalcTools.EdgeLength(mesh);
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
                List<MatrixElement> J_data = new List<MatrixElement>();
                for (int i = 0; i < duplicatedVertIndices.Count; i++)
                {
                    //固定の点の場合はスキップ
                    List<int> vertGroup = duplicatedVertIndices[i];
                    vertGroup.Sort();
                    int colX = 3 * i;
                    int colY = colX + 1;
                    int colZ = colX + 2;
                    if (vertGroup.Intersect(fixIndices).ToList().Count != 0) { continue; }

                    else if (Axy.Contains(vertGroup[0]))
                    {
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedEdges = mesh.GetEdgesForVertex(vertIndex);
                            foreach(int edge in connectedEdges)
                            {
                                double length = currentLength[edge];
                                Vector3d vector = new Vector3d();
                                if (mesh.Edges[edge][0] == vertIndex) { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][1]]; }
                                else { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][0]]; }
                                J_data.Add(new MatrixElement(devConstraintCount + edge, colX, w0 * vector.X / length));
                                J_data.Add(new MatrixElement(devConstraintCount + edge, colY, w0 * vector.Y / length));
                            }
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                int rowStart = vertToConstraints[effectVert];
                                if (B.Contains(effectVert))
                                {
                                    double[] jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 8, true, true, false, w1);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                }

                                if (C.Contains(effectVert))
                                {
                                    double[] jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 4, true, true, false, w1);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                }
                                if (E.Contains(effectVert))
                                {
                                    double[] jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 2, true, true, false, w1);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                }
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
                                int rowStart = vertToConstraints[loopVertices[0]];
                                double[] jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, true, true, false,
                                    w2, w3, w4, w5, w6);
                                //角度の和の条件式の微分
                                J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                //sinの条件式の微分
                                J_data.Add(new MatrixElement(rowStart + 1, colX, jacobi[3]));
                                J_data.Add(new MatrixElement(rowStart + 1, colY, jacobi[4]));
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
                                int rowStart = vertToConstraints[loopVertices[0]];
                                double[] jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, true, true, false,
                                    w2, w3, w4, w5, w6);
                                //角度の和の条件式の微分
                                J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
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
                                int rowStart = vertToConstraints[loopVertices[0]];
                                double[] jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, true, true, false,
                                    w2, w3, w4, w5, w6);
                                //角度の和の条件式の微分
                                J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                J_data.Add(new MatrixElement(rowStart + 1, colX, jacobi[3]));
                                J_data.Add(new MatrixElement(rowStart + 1, colY, jacobi[4]));
                                J_data.Add(new MatrixElement(rowStart + 2, colX, jacobi[6]));
                                J_data.Add(new MatrixElement(rowStart + 2, colY, jacobi[7]));
                            }
                        }
                    }

                    else if (Ayz.Contains(vertGroup[0]))
                    {
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedEdges = mesh.GetEdgesForVertex(vertIndex);
                            foreach (int edge in connectedEdges)
                            {
                                double length = currentLength[edge];
                                Vector3d vector = new Vector3d();
                                if (mesh.Edges[edge][0] == vertIndex) { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][1]]; }
                                else { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][0]]; }
                                J_data.Add(new MatrixElement(devConstraintCount + edge, colZ, w0 * vector.Z / length));
                                J_data.Add(new MatrixElement(devConstraintCount + edge, colY, w0 * vector.Y / length));
                            }
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                int rowStart = vertToConstraints[effectVert];
                                if (B.Contains(effectVert))
                                {
                                    double[] jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 8, false, true, true, w1);
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                }

                                if (C.Contains(effectVert))
                                {
                                    double[] jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 4, false, true, true, w1);
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                }
                                if (E.Contains(effectVert))
                                {
                                    double[] jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 2, false, true, true, w1);
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                }
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
                                int rowStart = vertToConstraints[loopVertices[0]];
                                double[] jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, false, true, true,
                                    w2, w3, w4, w5, w6);
                                //角度の和の条件式の微分
                                J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                //sinの条件式の微分
                                J_data.Add(new MatrixElement(rowStart + 1, colY, jacobi[4]));
                                J_data.Add(new MatrixElement(rowStart + 1, colZ, jacobi[5]));
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
                                int rowStart = vertToConstraints[loopVertices[0]];
                                double[] jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, false, true, true,
                                    w2, w3, w4, w5, w6);
                                //角度の和の条件式の微分
                                J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
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
                                int rowStart = vertToConstraints[loopVertices[0]];
                                double[] jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, false, true, true,
                                    w2, w3, w4, w5, w6);
                                //角度の和の条件式の微分
                                J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                J_data.Add(new MatrixElement(rowStart + 1, colY, jacobi[4]));
                                J_data.Add(new MatrixElement(rowStart + 1, colZ, jacobi[5]));
                                J_data.Add(new MatrixElement(rowStart + 2, colY, jacobi[7]));
                                J_data.Add(new MatrixElement(rowStart + 2, colZ, jacobi[8]));
                            }
                        }
                    }

                    else if (Azx.Contains(vertGroup[0]))
                    {
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedEdges = mesh.GetEdgesForVertex(vertIndex);
                            foreach (int edge in connectedEdges)
                            {
                                double length = currentLength[edge];
                                Vector3d vector = new Vector3d();
                                if (mesh.Edges[edge][0] == vertIndex) { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][1]]; }
                                else { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][0]]; }
                                J_data.Add(new MatrixElement(devConstraintCount + edge, colX, w0 * vector.X / length));
                                J_data.Add(new MatrixElement(devConstraintCount + edge, colZ, w0 * vector.Z / length));
                            }
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                int rowStart = vertToConstraints[effectVert];
                                if (B.Contains(effectVert))
                                {
                                    double[] jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 8, true, false, true, w1);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                }

                                if (C.Contains(effectVert))
                                {
                                    double[] jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 4, true, false, true, w1);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                }
                                if (E.Contains(effectVert))
                                {
                                    double[] jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 2, true, false, true, w1);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                }
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
                                int rowStart = vertToConstraints[loopVertices[0]];
                                double[] jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, true, false, true,
                                    w2, w3, w4, w5, w6);
                                //角度の和の条件式の微分
                                J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                //sinの条件式の微分
                                J_data.Add(new MatrixElement(rowStart + 1, colX, jacobi[3]));
                                J_data.Add(new MatrixElement(rowStart + 1, colZ, jacobi[5]));
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
                                int rowStart = vertToConstraints[loopVertices[0]];
                                double[] jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, true, false, true,
                                    w2, w3, w4, w5, w6);
                                //角度の和の条件式の微分
                                J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
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
                                int rowStart = vertToConstraints[loopVertices[0]];
                                double[] jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, true, false, true,
                                    w2, w3, w4, w5, w6);
                                //角度の和の条件式の微分
                                J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                J_data.Add(new MatrixElement(rowStart + 1, colX, jacobi[3]));
                                J_data.Add(new MatrixElement(rowStart + 1, colZ, jacobi[5]));
                                J_data.Add(new MatrixElement(rowStart + 2, colX, jacobi[6]));
                                J_data.Add(new MatrixElement(rowStart + 2, colZ, jacobi[8]));
                            }
                        }
                    }

                    else if (Afree.Contains(vertGroup[0]))
                    {
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedEdges = mesh.GetEdgesForVertex(vertIndex);
                            foreach (int edge in connectedEdges)
                            {
                                double length = currentLength[edge];
                                Vector3d vector = new Vector3d();
                                if (mesh.Edges[edge][0] == vertIndex) { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][1]]; }
                                else { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][0]]; }
                                J_data.Add(new MatrixElement(devConstraintCount + edge, colX, w0 * vector.X / length));
                                J_data.Add(new MatrixElement(devConstraintCount + edge, colY, w0 * vector.Y / length));
                                J_data.Add(new MatrixElement(devConstraintCount + edge, colZ, w0 * vector.Z / length));
                            }
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                int rowStart = vertToConstraints[effectVert];
                                if (B.Contains(effectVert))
                                {
                                    double[] jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 8, true, true, true, w1);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                }

                                if (C.Contains(effectVert))
                                {
                                    double[] jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 4, true, true, true, w1);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                }
                                if (E.Contains(effectVert))
                                {
                                    double[] jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 2, true, true, true, w1);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                }
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
                                int rowStart = vertToConstraints[loopVertices[0]];
                                double[] jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, true, true, true,
                                    w2, w3, w4, w5, w6);
                                //角度の和の条件式の微分
                                J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                //sinの条件式の微分
                                J_data.Add(new MatrixElement(rowStart + 1, colX, jacobi[3]));
                                J_data.Add(new MatrixElement(rowStart + 1, colY, jacobi[4]));
                                J_data.Add(new MatrixElement(rowStart + 1, colZ, jacobi[5]));
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
                                int rowStart = vertToConstraints[loopVertices[0]];
                                double[] jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, true, true, true,
                                    w2, w3, w4, w5, w6);
                                //角度の和の条件式の微分
                                J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
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
                                int rowStart = vertToConstraints[loopVertices[0]];
                                double[] jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, true, true, true,
                                    w2, w3, w4, w5, w6);
                                //角度の和の条件式の微分
                                J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                J_data.Add(new MatrixElement(rowStart + 1, colX, jacobi[3]));
                                J_data.Add(new MatrixElement(rowStart + 1, colY, jacobi[4]));
                                J_data.Add(new MatrixElement(rowStart + 1, colZ, jacobi[5]));
                                J_data.Add(new MatrixElement(rowStart + 2, colX, jacobi[6]));
                                J_data.Add(new MatrixElement(rowStart + 2, colY, jacobi[7]));
                                J_data.Add(new MatrixElement(rowStart + 2, colZ, jacobi[8]));
                            }
                        }
                    }

                    else if (Bx.Contains(vertGroup[0]))
                    {
                        int rowStart = vertToConstraints[vertGroup[0]];
                        double[] jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertGroup[0], vertGroup[0], 8, true, false, false, w1);
                        J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedEdges = mesh.GetEdgesForVertex(vertIndex);
                            foreach (int edge in connectedEdges)
                            {
                                double length = currentLength[edge];
                                Vector3d vector = new Vector3d();
                                if (mesh.Edges[edge][0] == vertIndex) { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][1]]; }
                                else { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][0]]; }
                                J_data.Add(new MatrixElement(devConstraintCount + edge, colX, w0 * vector.X / length));
                            }
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                rowStart = vertToConstraints[effectVert];
                                if (B.Contains(effectVert))
                                {
                                    jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 8, true, false, false, w1);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                }
                                else if (C.Contains(effectVert))
                                {
                                    jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 4, true, false, false, w1);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                }
                                else if (E.Contains(effectVert))
                                {
                                    jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 2, true, false, false, w1);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                }
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
                                rowStart = vertToConstraints[loopVertices[0]];
                                jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, true, false, false,
                                    w2, w3, w4, w5, w6);
                                J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                J_data.Add(new MatrixElement(rowStart + 1, colX, jacobi[3]));
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
                                rowStart = vertToConstraints[loopVertices[0]];
                                jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, true, false, false,
                                    w2, w3, w4, w5, w6);
                                J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
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
                                rowStart = vertToConstraints[loopVertices[0]];
                                jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, true, false, false,
                                    w2, w3, w4, w5, w6);
                                J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                J_data.Add(new MatrixElement(rowStart + 1, colX, jacobi[3]));
                                J_data.Add(new MatrixElement(rowStart + 2, colX, jacobi[6]));
                            }
                        }

                    }

                    else if (By.Contains(vertGroup[0]))
                    {
                        int rowStart = vertToConstraints[vertGroup[0]];
                        double[] jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertGroup[0], vertGroup[0], 8, false, true, false, w1);
                        J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedEdges = mesh.GetEdgesForVertex(vertIndex);
                            foreach (int edge in connectedEdges)
                            {
                                double length = currentLength[edge];
                                Vector3d vector = new Vector3d();
                                if (mesh.Edges[edge][0] == vertIndex) { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][1]]; }
                                else { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][0]]; }
                                J_data.Add(new MatrixElement(devConstraintCount + edge, colY, w0 * vector.Y / length));
                            }
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                rowStart = vertToConstraints[effectVert];
                                if (B.Contains(effectVert))
                                {
                                    jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 8, false, true, false, w1);
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                }
                                else if (C.Contains(effectVert))
                                {
                                    jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 4, false, true, false, w1);
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                }
                                else if (E.Contains(effectVert))
                                {
                                    jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 2, false, true, false, w1);
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                }
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
                                rowStart = vertToConstraints[loopVertices[0]];
                                jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, false, true, false,
                                    w2, w3, w4, w5, w6);
                                J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                J_data.Add(new MatrixElement(rowStart + 1, colY, jacobi[4]));
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
                                rowStart = vertToConstraints[loopVertices[0]];
                                jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, false, true, false,
                                    w2, w3, w4, w5, w6);
                                J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
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
                                rowStart = vertToConstraints[loopVertices[0]];
                                jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, false, true, false,
                                    w2, w3, w4, w5, w6);
                                J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                J_data.Add(new MatrixElement(rowStart + 1, colY, jacobi[4]));
                                J_data.Add(new MatrixElement(rowStart + 2, colY, jacobi[7]));
                            }
                        }
                    }

                    else if (Bz.Contains(vertGroup[0]))
                    {
                        int rowStart = vertToConstraints[vertGroup[0]];
                        double[] jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertGroup[0], vertGroup[0], 8, false, false, true, w1);
                        J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedEdges = mesh.GetEdgesForVertex(vertIndex);
                            foreach (int edge in connectedEdges)
                            {
                                double length = currentLength[edge];
                                Vector3d vector = new Vector3d();
                                if (mesh.Edges[edge][0] == vertIndex) { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][1]]; }
                                else { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][0]]; }
                                J_data.Add(new MatrixElement(devConstraintCount + edge, colZ, w0 * vector.Z / length));
                            }
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                rowStart = vertToConstraints[effectVert];
                                if (B.Contains(effectVert))
                                {
                                    jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 8, false, false, true, w1);
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                }
                                else if (C.Contains(effectVert))
                                {
                                    jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 4, false, false, true, w1);
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                }
                                else if (E.Contains(effectVert))
                                {
                                    jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 2, false, false, true, w1);
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                }
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
                                rowStart = vertToConstraints[loopVertices[0]];
                                jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, false, false, true,
                                    w2, w3, w4, w5, w6);
                                J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                J_data.Add(new MatrixElement(rowStart + 1, colZ, jacobi[5]));
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
                                rowStart = vertToConstraints[loopVertices[0]];
                                jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, false, false, true,
                                    w2, w3, w4, w5, w6);
                                J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
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
                                rowStart = vertToConstraints[loopVertices[0]];
                                jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, false, false, true,
                                    w2, w3, w4, w5, w6);
                                J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                J_data.Add(new MatrixElement(rowStart + 1, colZ, jacobi[5]));
                                J_data.Add(new MatrixElement(rowStart + 2, colZ, jacobi[8]));
                            }
                        }
                    }

                    else if (Cxy.Contains(vertGroup[0]))
                    {
                        int rowStart = vertToConstraints[vertGroup[0]];
                        double[] jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertGroup[0], vertGroup[0], 4, true, true, false, w1);
                        J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                        J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedEdges = mesh.GetEdgesForVertex(vertIndex);
                            foreach (int edge in connectedEdges)
                            {
                                double length = currentLength[edge];
                                Vector3d vector = new Vector3d();
                                if (mesh.Edges[edge][0] == vertIndex) { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][1]]; }
                                else { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][0]]; }
                                J_data.Add(new MatrixElement(devConstraintCount + edge, colX, w0 * vector.X / length));
                                J_data.Add(new MatrixElement(devConstraintCount + edge, colY, w0 * vector.Y / length));
                            }
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                rowStart = vertToConstraints[effectVert];
                                if (B.Contains(effectVert))
                                {
                                    jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 8, true, true, false, w1);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                }
                                if (C.Contains(effectVert))
                                {
                                    jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 4, true, true, false, w1);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                }
                                if (E.Contains(effectVert))
                                {
                                    jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 2, true, true, false, w1);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                }
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
                                rowStart = vertToConstraints[loopVertices[0]];
                                jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, true, true, false,
                                    w2, w3, w4, w5, w6);
                                J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                J_data.Add(new MatrixElement(rowStart + 1, colX, jacobi[3]));
                                J_data.Add(new MatrixElement(rowStart + 1, colY, jacobi[4]));
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
                                rowStart = vertToConstraints[loopVertices[0]];
                                jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, true, true, false,
                                    w2, w3, w4, w5, w6);
                                J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
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
                                rowStart = vertToConstraints[loopVertices[0]];
                                jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, true, true, false,
                                    w2, w3, w4, w5, w6);
                                J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                J_data.Add(new MatrixElement(rowStart + 1, colX, jacobi[3]));
                                J_data.Add(new MatrixElement(rowStart + 1, colY, jacobi[4]));
                                J_data.Add(new MatrixElement(rowStart + 2, colX, jacobi[6]));
                                J_data.Add(new MatrixElement(rowStart + 2, colY, jacobi[7]));
                            }
                        }
                    }

                    else if (Cyz.Contains(vertGroup[0]))
                    {
                        int rowStart = vertToConstraints[vertGroup[0]];
                        double[] jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertGroup[0], vertGroup[0], 4, false, true, true, w1);
                        J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                        J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedEdges = mesh.GetEdgesForVertex(vertIndex);
                            foreach (int edge in connectedEdges)
                            {
                                double length = currentLength[edge];
                                Vector3d vector = new Vector3d();
                                if (mesh.Edges[edge][0] == vertIndex) { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][1]]; }
                                else { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][0]]; }
                                J_data.Add(new MatrixElement(devConstraintCount + edge, colY, w0 * vector.Y / length));
                                J_data.Add(new MatrixElement(devConstraintCount + edge, colZ, w0 * vector.Z / length));
                            }
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                rowStart = vertToConstraints[effectVert];
                                if (B.Contains(effectVert))
                                {
                                    jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 8, false, true, true, w1);
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                }
                                if (C.Contains(effectVert))
                                {
                                    jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 4, false, true, true, w1);
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                }
                                if (E.Contains(effectVert))
                                {
                                    jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 2, false, true, true, w1);
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                }
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
                                rowStart = vertToConstraints[loopVertices[0]];
                                jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, false, true, true,
                                    w2, w3, w4, w5, w6);
                                J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                J_data.Add(new MatrixElement(rowStart + 1, colZ, jacobi[5]));
                                J_data.Add(new MatrixElement(rowStart + 1, colY, jacobi[4]));
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
                                rowStart = vertToConstraints[loopVertices[0]];
                                jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, false, true, true,
                                    w2, w3, w4, w5, w6);
                                J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
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
                                rowStart = vertToConstraints[loopVertices[0]];
                                jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, false, true, true,
                                    w2, w3, w4, w5, w6);
                                J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                J_data.Add(new MatrixElement(rowStart + 1, colZ, jacobi[5]));
                                J_data.Add(new MatrixElement(rowStart + 1, colY, jacobi[4]));
                                J_data.Add(new MatrixElement(rowStart + 2, colZ, jacobi[8]));
                                J_data.Add(new MatrixElement(rowStart + 2, colY, jacobi[7]));
                            }
                        }
                    }

                    else if (Czx.Contains(vertGroup[0]))
                    {
                        int rowStart = vertToConstraints[vertGroup[0]];
                        double[] jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertGroup[0], vertGroup[0], 4, true, false, true, w1);
                        J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                        J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedEdges = mesh.GetEdgesForVertex(vertIndex);
                            foreach (int edge in connectedEdges)
                            {
                                double length = currentLength[edge];
                                Vector3d vector = new Vector3d();
                                if (mesh.Edges[edge][0] == vertIndex) { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][1]]; }
                                else { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][0]]; }
                                J_data.Add(new MatrixElement(devConstraintCount + edge, colX, w0 * vector.X / length));
                                J_data.Add(new MatrixElement(devConstraintCount + edge, colZ, w0 * vector.Z / length));
                            }
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                rowStart = vertToConstraints[effectVert];
                                if (B.Contains(effectVert))
                                {
                                    jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 8, true, false, true, w1);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                }
                                if (C.Contains(effectVert))
                                {
                                    jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 4, true, false, true, w1);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                }
                                if (E.Contains(effectVert))
                                {
                                    jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 2, true, false, true, w1);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                }
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
                                rowStart = vertToConstraints[loopVertices[0]];
                                jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, true, false, true,
                                    w2, w3, w4, w5, w6);
                                J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                J_data.Add(new MatrixElement(rowStart + 1, colX, jacobi[3]));
                                J_data.Add(new MatrixElement(rowStart + 1, colZ, jacobi[5]));
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
                                rowStart = vertToConstraints[loopVertices[0]];
                                jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, true, false, true,
                                    w2, w3, w4, w5, w6);
                                J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
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
                                rowStart = vertToConstraints[loopVertices[0]];
                                jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, true, false, true,
                                    w2, w3, w4, w5, w6);
                                J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                J_data.Add(new MatrixElement(rowStart + 1, colX, jacobi[3]));
                                J_data.Add(new MatrixElement(rowStart + 1, colZ, jacobi[5]));
                                J_data.Add(new MatrixElement(rowStart + 2, colX, jacobi[6]));
                                J_data.Add(new MatrixElement(rowStart + 2, colZ, jacobi[8]));
                            }
                        }
                    }

                    else if (E.Contains(vertGroup[0]))
                    {
                        int rowStart = vertToConstraints[vertGroup[0]];
                        double[] jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertGroup[0], vertGroup[0], 2, true, true, true, w1);
                        J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                        J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                        J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));

                        List<int> effectDF = new List<int>();
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedEdges = mesh.GetEdgesForVertex(vertIndex);
                            foreach (int edge in connectedEdges)
                            {
                                double length = currentLength[edge];
                                Vector3d vector = new Vector3d();
                                if (mesh.Edges[edge][0] == vertIndex) { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][1]]; }
                                else { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][0]]; }
                                J_data.Add(new MatrixElement(devConstraintCount + edge, colX, w0 * vector.X / length));
                                J_data.Add(new MatrixElement(devConstraintCount + edge, colY, w0 * vector.Y / length));
                                J_data.Add(new MatrixElement(devConstraintCount + edge, colZ, w0 * vector.Z / length));
                            }
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                rowStart = vertToConstraints[effectVert];
                                if (B.Contains(effectVert))
                                {
                                    jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 8, true, true, true, w1);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                }
                                else if (C.Contains(effectVert))
                                {
                                    jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 4, true, true, true, w1);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                }
                                else if (E.Contains(effectVert))
                                {
                                    jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 2, true, true, true, w1);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                }
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
                                rowStart = vertToConstraints[loopVertices[0]];
                                jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 2, true, true, true,
                                    w2, w3, w4, w5, w6);
                                J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                J_data.Add(new MatrixElement(rowStart + 1, colX, jacobi[3]));
                                J_data.Add(new MatrixElement(rowStart + 1, colY, jacobi[4]));
                                J_data.Add(new MatrixElement(rowStart + 1, colZ, jacobi[5]));
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
                                rowStart = vertToConstraints[loopVertices[0]];
                                jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 1, true, true, true,
                                    w2, w3, w4, w5, w6);
                                J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
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
                                rowStart = vertToConstraints[loopVertices[0]];
                                jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, effectVertices, loopVertices, 3, true, true, true,
                                    w2, w3, w4, w5, w6);
                                J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                J_data.Add(new MatrixElement(rowStart + 1, colX, jacobi[3]));
                                J_data.Add(new MatrixElement(rowStart + 1, colY, jacobi[4]));
                                J_data.Add(new MatrixElement(rowStart + 1, colZ, jacobi[5]));
                                J_data.Add(new MatrixElement(rowStart + 2, colX, jacobi[6]));
                                J_data.Add(new MatrixElement(rowStart + 2, colY, jacobi[7]));
                                J_data.Add(new MatrixElement(rowStart + 2, colZ, jacobi[8]));
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
                            int rowStart = vertToConstraints[vertGroup[0]];
                            double[] jacobi = VertGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 2, true, true, false,
                                w2, w3, w4, w5, w6);
                            J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                            J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                            J_data.Add(new MatrixElement(rowStart + 1, colX, jacobi[3]));
                            J_data.Add(new MatrixElement(rowStart + 1, colY, jacobi[4]));
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedEdges = mesh.GetEdgesForVertex(vertIndex);
                                foreach (int edge in connectedEdges)
                                {
                                    double length = currentLength[edge];
                                    Vector3d vector = new Vector3d();
                                    if (mesh.Edges[edge][0] == vertIndex) { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][1]]; }
                                    else { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][0]]; }
                                    J_data.Add(new MatrixElement(devConstraintCount + edge, colX, w0 * vector.X / length));
                                    J_data.Add(new MatrixElement(devConstraintCount + edge, colY, w0 * vector.Y / length));
                                }
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    rowStart = vertToConstraints[effectVert];
                                    if (B.Contains(effectVert))
                                    {
                                        jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 8, true, true, false, w1);
                                        J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                        J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                    }
                                    else if (C.Contains(effectVert))
                                    {
                                        jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 4, true, true, false, w1);
                                        J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                        J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                    }
                                    else if (E.Contains(effectVert))
                                    {
                                        jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 2, true, true, false, w1);
                                        J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                        J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                    }
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
                                    rowStart = vertToConstraints[loopVertices2[0]];
                                    jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, true, true, false,
                                        w2, w3, w4, w5, w6);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                    J_data.Add(new MatrixElement(rowStart + 1, colX, jacobi[3]));
                                    J_data.Add(new MatrixElement(rowStart + 1, colY, jacobi[4]));
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
                                    rowStart = vertToConstraints[loopVertices2[0]];
                                    jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, true, true, false,
                                        w2, w3, w4, w5, w6);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
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
                                    rowStart = vertToConstraints[loopVertices2[0]];
                                    jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, true, true, false,
                                        w2, w3, w4, w5, w6);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                    J_data.Add(new MatrixElement(rowStart + 1, colX, jacobi[3]));
                                    J_data.Add(new MatrixElement(rowStart + 1, colY, jacobi[4]));
                                    J_data.Add(new MatrixElement(rowStart + 2, colX, jacobi[6]));
                                    J_data.Add(new MatrixElement(rowStart + 2, colY, jacobi[7]));
                                }
                            }
                        }
                        else if (yzMirrorInnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            int rowStart = vertToConstraints[vertGroup[0]];
                            double[] jacobi = VertGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 2, false, true, true,
                                w2, w3, w4, w5, w6);
                            J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                            J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                            J_data.Add(new MatrixElement(rowStart + 1, colZ, jacobi[5]));
                            J_data.Add(new MatrixElement(rowStart + 1, colY, jacobi[4]));
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedEdges = mesh.GetEdgesForVertex(vertIndex);
                                foreach (int edge in connectedEdges)
                                {
                                    double length = currentLength[edge];
                                    Vector3d vector = new Vector3d();
                                    if (mesh.Edges[edge][0] == vertIndex) { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][1]]; }
                                    else { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][0]]; }
                                    J_data.Add(new MatrixElement(devConstraintCount + edge, colY, w0 * vector.Y / length));
                                    J_data.Add(new MatrixElement(devConstraintCount + edge, colZ, w0 * vector.Z / length));
                                }
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    rowStart = vertToConstraints[effectVert];
                                    if (B.Contains(effectVert))
                                    {
                                        jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 8, false, true, true, w1);
                                        J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                        J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                    }
                                    else if (C.Contains(effectVert))
                                    {
                                        jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 4, false, true, true, w1);
                                        J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                        J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                    }
                                    else if (E.Contains(effectVert))
                                    {
                                        jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 2, false, true, true, w1);
                                        J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                        J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                    }
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
                                    rowStart = vertToConstraints[loopVertices2[0]];
                                    jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, false, true, true,
                                        w2, w3, w4, w5, w6);
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                    J_data.Add(new MatrixElement(rowStart + 1, colZ, jacobi[5]));
                                    J_data.Add(new MatrixElement(rowStart + 1, colY, jacobi[4]));
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
                                    rowStart = vertToConstraints[loopVertices2[0]];
                                    jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, false, true, true,
                                        w2, w3, w4, w5, w6);
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
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
                                    rowStart = vertToConstraints[loopVertices2[0]];
                                    jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, false, true, true,
                                        w2, w3, w4, w5, w6);
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                    J_data.Add(new MatrixElement(rowStart + 1, colZ, jacobi[5]));
                                    J_data.Add(new MatrixElement(rowStart + 1, colY, jacobi[4]));
                                    J_data.Add(new MatrixElement(rowStart + 2, colZ, jacobi[8]));
                                    J_data.Add(new MatrixElement(rowStart + 2, colY, jacobi[7]));
                                }
                            }
                        }
                        else if (zxMirrorInnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            int rowStart = vertToConstraints[vertGroup[0]];
                            double[] jacobi = VertGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 2, true, false, true,
                                w2, w3, w4, w5, w6);
                            J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                            J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                            J_data.Add(new MatrixElement(rowStart + 1, colX, jacobi[3]));
                            J_data.Add(new MatrixElement(rowStart + 1, colZ, jacobi[5]));
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedEdges = mesh.GetEdgesForVertex(vertIndex);
                                foreach (int edge in connectedEdges)
                                {
                                    double length = currentLength[edge];
                                    Vector3d vector = new Vector3d();
                                    if (mesh.Edges[edge][0] == vertIndex) { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][1]]; }
                                    else { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][0]]; }
                                    J_data.Add(new MatrixElement(devConstraintCount + edge, colX, w0 * vector.X / length));
                                    J_data.Add(new MatrixElement(devConstraintCount + edge, colZ, w0 * vector.Z / length));
                                }
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    rowStart = vertToConstraints[effectVert];
                                    if (B.Contains(effectVert))
                                    {
                                        jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 8, true, false, true, w1);
                                        J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                        J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                    }
                                    else if (C.Contains(effectVert))
                                    {
                                        jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 4, true, false, true, w1);
                                        J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                        J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                    }
                                    else if (E.Contains(effectVert))
                                    {
                                        jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 2, true, false, true, w1);
                                        J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                        J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                    }
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
                                    rowStart = vertToConstraints[loopVertices2[0]];
                                    jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, true, false, true,
                                        w2, w3, w4, w5, w6);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                    J_data.Add(new MatrixElement(rowStart + 1, colX, jacobi[3]));
                                    J_data.Add(new MatrixElement(rowStart + 1, colZ, jacobi[5]));
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
                                    rowStart = vertToConstraints[loopVertices2[0]];
                                    jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, true, false, true,
                                        w2, w3, w4, w5, w6);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
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
                                    rowStart = vertToConstraints[loopVertices2[0]];
                                    jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, true, false, true,
                                        w2, w3, w4, w5, w6);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                    J_data.Add(new MatrixElement(rowStart + 1, colX, jacobi[3]));
                                    J_data.Add(new MatrixElement(rowStart + 1, colZ, jacobi[5]));
                                    J_data.Add(new MatrixElement(rowStart + 2, colX, jacobi[6]));
                                    J_data.Add(new MatrixElement(rowStart + 2, colZ, jacobi[8]));
                                }
                            }
                        }
                        else
                        {
                            int rowStart = vertToConstraints[vertGroup[0]];
                            double[] jacobi = VertGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 2, true, true, true,
                                w2, w3, w4, w5, w6);
                            J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                            J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                            J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                            J_data.Add(new MatrixElement(rowStart + 1, colX, jacobi[3]));
                            J_data.Add(new MatrixElement(rowStart + 1, colY, jacobi[4]));
                            J_data.Add(new MatrixElement(rowStart + 1, colZ, jacobi[5]));
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedEdges = mesh.GetEdgesForVertex(vertIndex);
                                foreach (int edge in connectedEdges)
                                {
                                    double length = currentLength[edge];
                                    Vector3d vector = new Vector3d();
                                    if (mesh.Edges[edge][0] == vertIndex) { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][1]]; }
                                    else { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][0]]; }
                                    J_data.Add(new MatrixElement(devConstraintCount + edge, colX, w0 * vector.X / length));
                                    J_data.Add(new MatrixElement(devConstraintCount + edge, colY, w0 * vector.Y / length));
                                    J_data.Add(new MatrixElement(devConstraintCount + edge, colZ, w0 * vector.Z / length));
                                }
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    rowStart = vertToConstraints[effectVert];
                                    if (B.Contains(effectVert))
                                    {
                                        jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 8, true, true, true, w1);
                                        J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                        J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                        J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                    }
                                    else if (C.Contains(effectVert))
                                    {
                                        jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 4, true, true, true, w1);
                                        J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                        J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                        J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                    }
                                    else if (E.Contains(effectVert))
                                    {
                                        jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 2, true, true, true, w1);
                                        J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                        J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                        J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                    }
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
                                    rowStart = vertToConstraints[loopVertices2[0]];
                                    jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, true, true, true,
                                        w2, w3, w4, w5, w6);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                    J_data.Add(new MatrixElement(rowStart + 1, colX, jacobi[3]));
                                    J_data.Add(new MatrixElement(rowStart + 1, colY, jacobi[4]));
                                    J_data.Add(new MatrixElement(rowStart + 1, colZ, jacobi[5]));
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
                                    rowStart = vertToConstraints[loopVertices2[0]];
                                    jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, true, true, true,
                                        w2, w3, w4, w5, w6);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
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
                                    rowStart = vertToConstraints[loopVertices2[0]];
                                    jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, true, true, true,
                                        w2, w3, w4, w5, w6);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                    J_data.Add(new MatrixElement(rowStart + 1, colX, jacobi[3]));
                                    J_data.Add(new MatrixElement(rowStart + 1, colY, jacobi[4]));
                                    J_data.Add(new MatrixElement(rowStart + 1, colZ, jacobi[5]));
                                    J_data.Add(new MatrixElement(rowStart + 2, colX, jacobi[6]));
                                    J_data.Add(new MatrixElement(rowStart + 2, colY, jacobi[7]));
                                    J_data.Add(new MatrixElement(rowStart + 2, colZ, jacobi[8]));
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
                            int rowStart = vertToConstraints[vertGroup[0]];
                            double[] jacobi = VertGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 1, false, true, false,
                                w2, w3, w4, w5, w6);
                            J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedEdges = mesh.GetEdgesForVertex(vertIndex);
                                foreach (int edge in connectedEdges)
                                {
                                    double length = currentLength[edge];
                                    Vector3d vector = new Vector3d();
                                    if (mesh.Edges[edge][0] == vertIndex) { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][1]]; }
                                    else { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][0]]; }
                                    J_data.Add(new MatrixElement(devConstraintCount + edge, colY, w0 * vector.Y / length));
                                }
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    rowStart = vertToConstraints[effectVert];
                                    if (B.Contains(effectVert))
                                    {
                                        jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 8, false, true, false, w1);
                                        J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                    }
                                    else if (C.Contains(effectVert))
                                    {
                                        jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 4, false, true, false, w1);
                                        J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                    }
                                    else if (E.Contains(effectVert))
                                    {
                                        jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 2, false, true, false, w1);
                                        J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                    }
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
                                    rowStart = vertToConstraints[loopVertices2[0]];
                                    jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, false, true, false,
                                        w2, w3, w4, w5, w6);
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                    J_data.Add(new MatrixElement(rowStart + 1, colY, jacobi[4]));
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
                                    rowStart = vertToConstraints[loopVertices2[0]];
                                    jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, false, true, false,
                                        w2, w3, w4, w5, w6);
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
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
                                    rowStart = vertToConstraints[loopVertices2[0]];
                                    jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, false, true, false,
                                        w2, w3, w4, w5, w6);
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                    J_data.Add(new MatrixElement(rowStart + 1, colY, jacobi[4]));
                                    J_data.Add(new MatrixElement(rowStart + 2, colY, jacobi[7]));
                                }
                            }
                        }
                        else if (yzZXinnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            int rowStart = vertToConstraints[vertGroup[0]];
                            double[] jacobi = VertGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 1, false, false, true,
                                w2, w3, w4, w5, w6);
                            J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedEdges = mesh.GetEdgesForVertex(vertIndex);
                                foreach (int edge in connectedEdges)
                                {
                                    double length = currentLength[edge];
                                    Vector3d vector = new Vector3d();
                                    if (mesh.Edges[edge][0] == vertIndex) { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][1]]; }
                                    else { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][0]]; }
                                    J_data.Add(new MatrixElement(devConstraintCount + edge, colZ, w0 * vector.Z / length));
                                }
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    rowStart = vertToConstraints[effectVert];
                                    if (B.Contains(effectVert))
                                    {
                                        jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 8, false, false, true, w1);
                                        J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                    }
                                    else if (C.Contains(effectVert))
                                    {
                                        jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 4, false, false, true, w1);
                                        J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                    }
                                    else if (E.Contains(effectVert))
                                    {
                                        jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 2, false, false, true, w1);
                                        J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                    }
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
                                    rowStart = vertToConstraints[loopVertices2[0]];
                                    jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, false, false, true,
                                        w2, w3, w4, w5, w6);
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                    J_data.Add(new MatrixElement(rowStart + 1, colZ, jacobi[5]));
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
                                    rowStart = vertToConstraints[loopVertices2[0]];
                                    jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, false, false, true,
                                        w2, w3, w4, w5, w6);
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
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
                                    rowStart = vertToConstraints[loopVertices2[0]];
                                    jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, false, false, true,
                                        w2, w3, w4, w5, w6);
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                    J_data.Add(new MatrixElement(rowStart + 1, colZ, jacobi[5]));
                                    J_data.Add(new MatrixElement(rowStart + 2, colZ, jacobi[8]));
                                }
                            }
                        }
                        else if (zxXYinnerCut.Any(sub => sub[0] == vertGroup[0]))
                        {
                            int rowStart = vertToConstraints[vertGroup[0]];
                            double[] jacobi = VertGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 1, true, false, false,
                                w2, w3, w4, w5, w6);
                            J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedEdges = mesh.GetEdgesForVertex(vertIndex);
                                foreach (int edge in connectedEdges)
                                {
                                    double length = currentLength[edge];
                                    Vector3d vector = new Vector3d();
                                    if (mesh.Edges[edge][0] == vertIndex) { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][1]]; }
                                    else { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][0]]; }
                                    J_data.Add(new MatrixElement(devConstraintCount + edge, colX, w0 * vector.X / length));
                                }
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    rowStart = vertToConstraints[effectVert];
                                    if (B.Contains(effectVert))
                                    {
                                        jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 8, true, false, false, w1);
                                        J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    }
                                    else if (C.Contains(effectVert))
                                    {
                                        jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 4, true, false, false, w1);
                                        J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    }
                                    else if (E.Contains(effectVert))
                                    {
                                        jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 2, true, false, false, w1);
                                        J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    }
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
                                    rowStart = vertToConstraints[loopVertices2[0]];
                                    jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, true, false, false,
                                        w2, w3, w4, w5, w6);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart + 1, colX, jacobi[3]));
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
                                    rowStart = vertToConstraints[loopVertices2[0]];
                                    jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, true, false, false,
                                        w2, w3, w4, w5, w6);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
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
                                    rowStart = vertToConstraints[loopVertices2[0]];
                                    jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, true, false, false,
                                        w2, w3, w4, w5, w6);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart + 1, colX, jacobi[3]));
                                    J_data.Add(new MatrixElement(rowStart + 2, colX, jacobi[6]));
                                }
                            }
                        }
                        else
                        {
                            int rowStart = vertToConstraints[vertGroup[0]];
                            double[] jacobi = VertGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 1, true, true, true,
                                w2, w3, w4, w5, w6);
                            J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                            J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                            J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                            foreach (int vertIndex in vertGroup)
                            {
                                List<int> connectedEdges = mesh.GetEdgesForVertex(vertIndex);
                                foreach (int edge in connectedEdges)
                                {
                                    double length = currentLength[edge];
                                    Vector3d vector = new Vector3d();
                                    if (mesh.Edges[edge][0] == vertIndex) { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][1]]; }
                                    else { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][0]]; }
                                    J_data.Add(new MatrixElement(devConstraintCount + edge, colX, w0 * vector.X / length));
                                    J_data.Add(new MatrixElement(devConstraintCount + edge, colY, w0 * vector.Y / length));
                                    J_data.Add(new MatrixElement(devConstraintCount + edge, colZ, w0 * vector.Z / length));
                                }
                                List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                                foreach (int effectVert in connectedVertices)
                                {
                                    rowStart = vertToConstraints[effectVert];
                                    if (B.Contains(effectVert))
                                    {
                                        jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 8, true, true, true, w1);
                                        J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                        J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                        J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                    }
                                    else if (C.Contains(effectVert))
                                    {
                                        jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 4, true, true, true, w1);
                                        J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                        J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                        J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                    }
                                    else if (E.Contains(effectVert))
                                    {
                                        jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 2, true, true, true, w1);
                                        J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                        J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                        J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                    }
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
                                    rowStart = vertToConstraints[loopVertices2[0]];
                                    jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, true, true, true,
                                        w2, w3, w4, w5, w6);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                    J_data.Add(new MatrixElement(rowStart + 1, colX, jacobi[3]));
                                    J_data.Add(new MatrixElement(rowStart + 1, colY, jacobi[4]));
                                    J_data.Add(new MatrixElement(rowStart + 1, colZ, jacobi[5]));
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
                                    rowStart = vertToConstraints[loopVertices2[0]];
                                    jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, true, true, true,
                                        w2, w3, w4, w5, w6);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
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
                                    rowStart = vertToConstraints[loopVertices2[0]];
                                    jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, true, true, true,
                                        w2, w3, w4, w5, w6);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                    J_data.Add(new MatrixElement(rowStart + 1, colX, jacobi[3]));
                                    J_data.Add(new MatrixElement(rowStart + 1, colY, jacobi[4]));
                                    J_data.Add(new MatrixElement(rowStart + 1, colZ, jacobi[5]));
                                    J_data.Add(new MatrixElement(rowStart + 2, colX, jacobi[6]));
                                    J_data.Add(new MatrixElement(rowStart + 2, colY, jacobi[7]));
                                    J_data.Add(new MatrixElement(rowStart + 2, colZ, jacobi[8]));
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

                        int rowStart = vertToConstraints[vertGroup[0]];
                        double[] jacobi = VertGroupDFConstraintJacobi(mesh, angleSum, loopLength, loopCos, loopSin, vertGroup, loopVertices, 3, true, true, true,
                            w2, w3, w4, w5, w6);
                        J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                        J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                        J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                        J_data.Add(new MatrixElement(rowStart + 1, colX, jacobi[3]));
                        J_data.Add(new MatrixElement(rowStart + 1, colY, jacobi[4]));
                        J_data.Add(new MatrixElement(rowStart + 1, colZ, jacobi[5]));
                        J_data.Add(new MatrixElement(rowStart + 2, colX, jacobi[6]));
                        J_data.Add(new MatrixElement(rowStart + 2, colY, jacobi[7]));
                        J_data.Add(new MatrixElement(rowStart + 2, colZ, jacobi[8]));
                        foreach (int vertIndex in vertGroup)
                        {
                            List<int> connectedEdges = mesh.GetEdgesForVertex(vertIndex);
                            foreach (int edge in connectedEdges)
                            {
                                double length = currentLength[edge];
                                Vector3d vector = new Vector3d();
                                if (mesh.Edges[edge][0] == vertIndex) { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][1]]; }
                                else { vector = mesh.Vertices[vertIndex] - mesh.Vertices[mesh.Edges[edge][0]]; }
                                J_data.Add(new MatrixElement(devConstraintCount + edge, colX, w0 * vector.X / length));
                                J_data.Add(new MatrixElement(devConstraintCount + edge, colY, w0 * vector.Y / length));
                                J_data.Add(new MatrixElement(devConstraintCount + edge, colZ, w0 * vector.Z / length));
                            }
                            List<int> connectedVertices = mesh.GetVerticesForVertex(vertIndex);
                            foreach (int effectVert in connectedVertices)
                            {
                                rowStart = vertToConstraints[effectVert];
                                if (B.Contains(effectVert))
                                {
                                    jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 8, true, true, true, w1);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                }
                                else if (C.Contains(effectVert))
                                {
                                    jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 4, true, true, true, w1);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                }
                                else if (E.Contains(effectVert))
                                {
                                    jacobi = EffectToGroupBCEConstraintJacobi(mesh, vertIndex, effectVert, 2, true, true, true, w1);
                                    J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                    J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                    J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                }
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
                                rowStart = vertToConstraints[loopVertices2[0]];
                                jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 2, true, true, true,
                                    w2, w3, w4, w5, w6);
                                J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                J_data.Add(new MatrixElement(rowStart + 1, colX, jacobi[3]));
                                J_data.Add(new MatrixElement(rowStart + 1, colY, jacobi[4]));
                                J_data.Add(new MatrixElement(rowStart + 1, colZ, jacobi[5]));
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
                                rowStart = vertToConstraints[loopVertices2[0]];
                                jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 1, true, true, true,
                                    w2, w3, w4, w5, w6);
                                J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
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
                                rowStart = vertToConstraints[loopVertices2[0]];
                                jacobi = EffectToGroupDFConstraintJacobi(mesh, angleSum, loopLength2, loopCos2, loopSin2, vertGroup, effectVertices, loopVertices2, 3, true, true, true,
                                    w2, w3, w4, w5, w6);
                                J_data.Add(new MatrixElement(rowStart, colX, jacobi[0]));
                                J_data.Add(new MatrixElement(rowStart, colY, jacobi[1]));
                                J_data.Add(new MatrixElement(rowStart, colZ, jacobi[2]));
                                J_data.Add(new MatrixElement(rowStart + 1, colX, jacobi[3]));
                                J_data.Add(new MatrixElement(rowStart + 1, colY, jacobi[4]));
                                J_data.Add(new MatrixElement(rowStart + 1, colZ, jacobi[5]));
                                J_data.Add(new MatrixElement(rowStart + 2, colX, jacobi[6]));
                                J_data.Add(new MatrixElement(rowStart + 2, colY, jacobi[7]));
                                J_data.Add(new MatrixElement(rowStart + 2, colZ, jacobi[8]));
                            }
                        }

                    }
                }
                /*----------------------------------------------------Jacobiの算出終了-------------------------------------------*/

                //J * input,サイズ n のベクトルを受け取り、サイズ m のベクトルを返す
                Func<double[], double[]> multiplyJ = (input) =>
                {
                    double[] output = new double[m]; // 結果は制約数サイズ

                    // スパース行列の計算: ゼロじゃない要素だけループする
                    foreach (var elem in J_data)
                    {
                        // J * v の定義通り: 行ごとの積和
                        output[elem.Row] += elem.Val * input[elem.Col];
                    }
                    return output;
                };

                //J^T * input,サイズ m のベクトルを受け取り、サイズ n のベクトルを返す
                Func<double[], double[]> multiplyJT = (input) =>
                {
                    double[] output = new double[n]; // 結果は変数数サイズ

                    // 転置行列の計算: RowとColを入れ替えて考える
                    foreach (var elem in J_data)
                    {
                        // J^T * v なので、inputのインデックスは「Row」になる
                        output[elem.Col] += elem.Val * input[elem.Row];
                    }
                    return output;
                };

                double[] deltaX = new double[n]; // 初期値 0

                //制約条件の式で求めた値の-1倍がb
                double[] b = new double[m];
                //まずdevelopableのconstraintsの値の-1倍を求めてbの該当部分に入れる
                double[] devConstraintValue = DevConstraintValue(mesh, category, devConstraintCount, vertToConstraints,
                    angleSum, DoneLength, DoneCos, DoneSin, FblocksLength, FblocksCos, FblocksSin, w1, w2, w3, w4, w5, w6);
                for (int i = 0; i < devConstraintCount; i++)
                {
                    b[i] = -devConstraintValue[i];
                }

                for (int i = 0; i < edgeCount; i++)
                {
                    b[devConstraintCount + i] = -w0 * (currentLength[i] - initialLength[i]);
                }


                //maxItrはデフォルトで100、torelanceはデフォルトで1e-6
                SolveCGLS(m, n, multiplyJ, multiplyJT, b,deltaX);

                //現在の位置にdeltaXを足す
                for (int i = 0; i < dupVertCount; i++)
                {
                    Vector3d deltaVec = new Vector3d(deltaX[3 * i], deltaX[3 * i + 1], deltaX[3 * i + 2]);
                    foreach (int v in duplicatedVertIndices[i])
                    {
                        mesh.Vertices[v] += deltaVec; 
                    }
                } 
            }
            return mesh;
        }

        /* 
         * multiplyJ: J  * v を計算する関数 multiplyJT: J^T * v を計算する関数 
         * J^T J Δx = - J^T C(x)（最小二乗法による解)をCG法で解く。反復回数はmaxIter, 許容誤差1e-6、Jのサイズはm x n、b = -C(x)
         * 制約条件の数がm、設計変数の数がn個
         */
        public static void SolveCGLS(int m, int n,Func<double[], double[]> multiplyJ, Func<double[], double[]> multiplyJT,
            double[] b,double[] x,int maxIter = 100,double tolerance = 1e-6)
        {
            // 1. r0 = b - J * x (初期値x=0なら r=b)
            // ※厳密には正規方程式の残差ではなく、本来の式 Jx=b の残差 r を管理するのが一般的

            double[] Jx = multiplyJ(x);
            double[] r = new double[m];
            for (int i = 0; i < m; i++) r[i] = b[i] - Jx[i];

            // p = J^T * r
            double[] p = multiplyJT(r);

            // s = p (共役方向の初期値)
            double[] s = (double[])p.Clone();

            double normSq_p = MatrixUtils.Multiply(p, p); // ||J^T r||^2
            if (normSq_p < tolerance * tolerance) return; // 最初から解けてる場合

            for (int k = 0; k < maxIter; k++)
            {
                // q = J * s
                double[] q = multiplyJ(s);

                // alpha = ||p||^2 / ||q||^2
                double alpha = normSq_p / MatrixUtils.Multiply(q, q);

                // x = x + alpha * s
                for (int i = 0; i < n; i++) x[i] += alpha * s[i];

                // r = r - alpha * q
                for (int i = 0; i < m; i++) r[i] -= alpha * q[i];

                // p_new = J^T * r
                double[] p_new = multiplyJT(r);
                double normSq_p_new = MatrixUtils.Multiply(p_new, p_new);

                if (normSq_p_new < tolerance * tolerance) break; // 収束判定

                // beta = ||p_new||^2 / ||p||^2
                double beta = normSq_p_new / normSq_p;

                // s = p_new + beta * s
                for (int i = 0; i < n; i++) s[i] = p_new[i] + beta * s[i];

                p = p_new;
                normSq_p = normSq_p_new;
            }
        }


        //developableのconstraintsの関数の値を求める
        public static double[] DevConstraintValue(CutMesh mesh, List<List<List<int>>> category, int devConstraintCount, 
            int[] vertToConstraints, double[] angleSum, List<List<double>> DoneLength, List<List<double>> DoneCos, 
            List<List<double>> DoneSin, List<List<double>> FblocksLength, List<List<double>> FblocksCos, 
            List<List<double>> FblocksSin, double w1,  double w2, double w3, double w4, double w5, double w6)
        {
            /*----------------------------------------------------A～Fの分類-----------------------------------------------------------------------*/
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

            double[] constraintValues = new double[devConstraintCount];

            foreach (int vert in B)
            {
                double angle = angleSum[vert];
                int row = vertToConstraints[vert];
                constraintValues[row] = w1 * (2 * Math.PI - 4 * angle);
            }
            foreach (int vert in C)
            {
                double angle = angleSum[vert];
                int row = vertToConstraints[vert];
                constraintValues[row] = w1 * (2 * Math.PI - 2 * angle);
            }
            foreach (int vert in E)
            {
                double angle = angleSum[vert];
                int row = vertToConstraints[vert];
                constraintValues[row] = w1 * (2 * Math.PI - angle);
            }
            foreach (List<int> loopVert in DtwoMirror)
            {
                int n = loopVert.Count;
                double angle = 0;
                foreach (int vert in loopVert)
                {
                    angle += angleSum[vert];
                }
                int row = vertToConstraints[loopVert[0]];
                constraintValues[row] = (n + 1) * w6 * (n * Math.PI - angle) / n;
            }
            int i = 0;
            foreach (List<int> loopVert in DoneMirror)
            {
                List<double> loopLength = DoneLength[i];
                List<double> loopSin = DoneSin[i];
                int n = loopVert.Count;
                double angle = 0;
                foreach (int vert in loopVert)
                {
                    angle += angleSum[vert];
                }
                int row = vertToConstraints[loopVert[0]];
                constraintValues[row] = (n + 1) * w4 * (n * Math.PI - angle) / n;

                double averageLength = 0;
                for (int ii = 1; ii < loopLength.Count; ii++)
                {
                    averageLength += loopLength[ii];
                }
                averageLength = averageLength / (n - 1);
                double s = 0;
                for (int j = 0; j < n - 1; j++)
                {
                    if (j % 2 == 0) { s += loopLength[j + 1] * loopSin[j]; }
                    else { s -= loopLength[j + 1] * loopSin[j]; }
                }
                constraintValues[row + 1] = w5 * (n + 1) * Math.PI * s / averageLength;
                i += 1;
            }
            i = 0;
            foreach (List<int> loopVert in Fblocks)
            {
                List<double> loopLength = FblocksLength[i];
                List<double> loopSin = FblocksSin[i];
                List<double> loopCos = FblocksCos[i];
                int n = loopVert.Count;
                double angle = 0;
                foreach (int vert in loopVert)
                {
                    angle += angleSum[vert];
                }
                int row = vertToConstraints[loopVert[0]];
                constraintValues[row] = (n + 4) * w2 * ((n + 2) * Math.PI - angle) / (n + 2);


                double averageLength = 0;
                for (int ii = 1; ii < loopLength.Count; ii++)
                {
                    averageLength += loopLength[ii];
                }
                averageLength = averageLength / n;


                double c = loopLength[1];
                for (int j = 1; j < n; j++)
                {
                    if (j % 2 == 0) { c += loopLength[j + 1] * loopCos[j - 1]; }
                    else { c -= loopLength[j + 1] * loopCos[j - 1]; }
                }

                double s = 0;
                for (int j = 1; j < n; j++)
                {
                    if (j % 2 == 0) { s += loopLength[j + 1] * loopSin[j - 1]; }
                    else { s -= loopLength[j + 1] * loopSin[j - 1]; }
                }
                double a = w3 * (n + 4) * Math.PI / averageLength;
                constraintValues[row + 1] = s * a;
                constraintValues[row + 2] = c * a;
                i += 1;
            }
            return constraintValues;
        }

        //constraintsとして入れるときのjacobi、x,y,zの長さ3で返す。Bならratio:8, Cならratio:4, Eならratio:2。この関数合ってそう
        public static double[] EffectToGroupBCEConstraintJacobi(CutMesh cutMesh, int vertIndex, int effectVert, int ratio, bool moveX, bool moveY, bool moveZ, double w1)
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
            double[] Jacobi = new double[3];
            Jacobi[0] = sX * -0.5 * ratio * w1;
            Jacobi[1] = sY * -0.5 * ratio * w1;
            Jacobi[2] = sZ * -0.5 * ratio * w1;
            return Jacobi;
        }

        //constraintsとして入れるときのjacobi、x,y,zの長さ3*1,2,3で返す。
        //loopが１方向ミラーならpattern = 2,2方向ミラーならpattern = 1,Fならpattern = 3
        public static double[] VertGroupDFConstraintJacobi(CutMesh cutMesh, double[] angleSum,
            List<double> loopLength, List<double> loopCos, List<double> loopSin, List<int> vertGroup,
            List<int> loopVertices, int pattern, bool moveX, bool moveY, bool moveZ,
            double w2, double w3, double w4, double w5, double w6)
        {
            List<int> nextVertices = new List<int>();
            foreach (int vertIndex in vertGroup)
            {
                List<int> next = (cutMesh.GetVerticesForVertex(vertIndex).Intersect(loopVertices)).Except(nextVertices).ToList();
                nextVertices.AddRange(next);
            }
            int n = loopVertices.Count;
            List<Point3d> vertices = cutMesh.Vertices;
            double sX = 0;
            double sY = 0;
            double sZ = 0;

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

            List<double> Jacobi = new List<double>();
            double calc = 0;
            if (pattern == 1) { calc = w6 * (n + 1) / n; }
            if (pattern == 2) { calc = w4 * (n + 1) / n; }
            if (pattern == 3) { calc = w2 * (n + 4) / (n + 2); }

            Jacobi.Add(-(sX + sX2) * calc);
            Jacobi.Add(-(sY + sY2) * calc);
            Jacobi.Add(-(sZ + sZ2) * calc);

            //2方向ミラーの場合
            if (pattern == 1) { return Jacobi.ToArray(); }
            //1方向ミラーの場合
            else if (pattern == 2)
            {
                double averageLength = 0;
                for (int ii = 1; ii < loopLength.Count; ii++)
                {
                    averageLength += loopLength[ii];
                }
                averageLength = averageLength / (n - 1);
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
                double tZ2 = 0;
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
                //double value2 = s * count * 4 * Math.PI / (averageLength * averageLength);
                double value2 = w5 * (n + 1) * Math.PI / averageLength;
                Jacobi.Add(value2 * sumX);
                Jacobi.Add(value2 * sumY);
                Jacobi.Add(value2 * sumZ);
            }
            //Ｆの場合
            else
            {
                double averageLength = 0;
                for (int ii = 1; ii < loopLength.Count; ii++)
                {
                    averageLength += loopLength[ii];
                }
                averageLength = averageLength / n;
                int count = loopVertices.Count;

                //まずsinの条件式の方から
                double s = 0;
                for (int j = 1; j < count; j++)
                {
                    if (j % 2 == 0) { s += loopLength[j + 1] * loopSin[j - 1]; }
                    else { s -= loopLength[j + 1] * loopSin[j - 1]; }
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
                //double value2 = 2 * s * count * 4 * Math.PI / (3 * averageLength * averageLength);
                double value2 = w3 * (n + 4) * Math.PI / averageLength;
                Jacobi.Add(value2 * sumX);
                Jacobi.Add(value2 * sumY);
                Jacobi.Add(value2 * sumZ);

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
                Jacobi.Add(value2 * sumX);
                Jacobi.Add(value2 * sumY);
                Jacobi.Add(value2 * sumZ);
            }

            return Jacobi.ToArray();
        }

        //constraintsとして入れるときのjacobi、x,y,zの長さ3*1,2,3で返す。
        //loopが１方向ミラーならpattern = 2,2方向ミラーならpattern = 1,Fならpattern = 3
        public static double[] EffectToGroupDFConstraintJacobi(CutMesh cutMesh, double[] angleSum,
            List<double> loopLength, List<double> loopCos, List<double> loopSin, List<int> vertGroup, List<int> effectVertices,
            List<int> loopVertices, int pattern, bool moveX, bool moveY, bool moveZ,
            double w2, double w3, double w4, double w5, double w6)
        {
            int n = loopVertices.Count;

            List<Point3d> vertices = cutMesh.Vertices;
            double sX = 0;
            double sY = 0;
            double sZ = 0;

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

            List<double> Jacobi = new List<double>();
            double calc = 0;
            if (pattern == 1) { calc = w6 * (n + 1) / n; }
            if (pattern == 2) { calc = w4 * (n + 1) / n; }
            if (pattern == 3) { calc = w2 * (n + 4) / (n + 2); }

            Jacobi.Add(-sX * calc);
            Jacobi.Add(-sY * calc);
            Jacobi.Add(-sZ * calc);
            //2方向ミラーの場合
            if (pattern == 1) { return Jacobi.ToArray(); }
            //1方向ミラーの場合
            else if (pattern == 2)
            {
                double averageLength = 0;
                for (int ii = 1; ii < loopLength.Count; ii++)
                {
                    averageLength += loopLength[ii];
                }
                averageLength = averageLength / (n - 1);
                int count = loopVertices.Count;//つまりnと同じ
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
                //double value2 = s * count * 4 * Math.PI / (averageLength * averageLength);
                double value2 = w5 * (n + 1) * Math.PI / averageLength;
                Jacobi.Add(value2 * sumX);
                Jacobi.Add(value2 * sumY);
                Jacobi.Add(value2 * sumZ);

            }
            //Ｆの場合
            else
            {
                double averageLength = 0;
                for (int ii = 1; ii < loopLength.Count; ii++)
                {
                    averageLength += loopLength[ii];
                }
                averageLength = averageLength / n;
                int count = loopVertices.Count;//つまりnと同じ
                double s = loopLength[1];
                for (int j = 1; j < count; j++)
                {
                    if (j % 2 == 0) { s += loopLength[j + 1] * loopCos[j - 1]; }
                    else { s -= loopLength[j + 1] * loopCos[j - 1]; }
                }

                //cosの条件式の方
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
                    double value = loopLength[j + 1] * loopSin[j - 1];
                    if (moveX) { if (j % 2 == 0) { sumX -= value * tX; } else { sumX += value * tX; } }
                    if (moveY) { if (j % 2 == 0) { sumY -= value * tY; } else { sumY += value * tY; } }
                    if (moveZ) { if (j % 2 == 0) { sumZ -= value * tZ; } else { sumZ += value * tZ; } }
                }
                //double value2 = 2 * s * count * count * Math.PI * Math.PI / (3 * averageLength * averageLength);
                //double value2 = 2 * s * 4 * count * Math.PI / (3 * averageLength * averageLength);
                double value2 = w3 * (n + 4) * Math.PI / averageLength;
                double aX = value2 * sumX;
                double aY = value2 * sumY;
                double aZ = value2 * sumZ;

                //sinの条件式の方
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

                Jacobi.Add(value2 * sumX);
                Jacobi.Add(value2 * sumY);
                Jacobi.Add(value2 * sumZ);
                Jacobi.Add(aX);
                Jacobi.Add(aY);
                Jacobi.Add(aZ);
            }
            return Jacobi.ToArray();
        }

        //DuplicatedVertのi番目の点たちがdevConstraintsの何行目(～何行目)に対応するのか
        public static int[] ConstraintsOrder(List<List<int>> duplicatedVertIndices, List<List<List<int>>> category)
        {
            List<int> B = category[0][1];
            List<int> C = category[0][2];
            List<int> E = category[0][4];
            List<List<int>> DoneMirror = category[1];
            List<List<int>> DtwoMirror = category[5];
            List<List<int>> Fblocks = category[9];

            int dupVertCount = duplicatedVertIndices.Count;
            int[] constraintsOrder = new int[dupVertCount];
            for (int i = 0; i < constraintsOrder.Length; i++)
            {
                constraintsOrder[i] = -1;
            }
            int guide = 0;
            List<int> skipVert = new List<int>();
            int iii = 0;
            foreach (List<int> vertGroup in duplicatedVertIndices)
            {
                if (skipVert.Contains(vertGroup[0])) { iii += 1; continue; }
                if (B.Contains(vertGroup[0])) { constraintsOrder[iii] = guide; guide += 1; iii += 1; continue; }
                if (C.Contains(vertGroup[0])) { constraintsOrder[iii] = guide; guide += 1; iii += 1; continue; }
                if (E.Contains(vertGroup[0])) { constraintsOrder[iii] = guide; guide += 1; iii += 1; continue; }

                List<int> found = DoneMirror.FirstOrDefault(sub => sub.Contains(vertGroup[0]));
                if (found != null)
                {
                    foreach (int f in found)
                    {
                        for (int i = 0; i < dupVertCount; i++)
                        {
                            if (duplicatedVertIndices[i].Contains(f))
                            {
                                constraintsOrder[i] = guide; // f は i 番目のサブリストにある
                                break;
                            }
                        }
                    }

                    guide += 2;
                    skipVert.AddRange(found);
                    iii += 1;
                    continue;
                }

                found = DtwoMirror.FirstOrDefault(sub => sub.Contains(vertGroup[0]));
                if (found != null)
                {
                    foreach (int f in found)
                    {
                        for (int i = 0; i < dupVertCount; i++)
                        {
                            if (duplicatedVertIndices[i].Contains(f))
                            {
                                constraintsOrder[i] = guide; // f は i 番目のサブリストにある
                                break;
                            }
                        }
                    }
                    guide += 1;
                    skipVert.AddRange(found);
                    iii += 1;
                    continue;
                }

                found = Fblocks.FirstOrDefault(sub => sub.Contains(vertGroup[0]));
                if (found != null)
                {
                    foreach (int f in found)
                    {
                        for (int i = 0; i < dupVertCount; i++)
                        {
                            if (duplicatedVertIndices[i].Contains(f))
                            {
                                constraintsOrder[i] = guide; // f は i 番目のサブリストにある
                                break;
                            }
                        }
                    }
                    guide += 3;
                    skipVert.AddRange(found);
                    iii += 1;
                    continue;
                }
                iii += 1;
            }
            return constraintsOrder;
        }

        //verticesのi番目の点がdupVertの何番目のグループの点なのかの対応のリスト
        public static int[] VertOrderInDup(int meshVertCount, List<List<int>> duplicatedVertIndices)
        {
            int[] vertOrderInDup = new int[meshVertCount];
            for (int i = 0; i < vertOrderInDup.Length; i++)
            {
                vertOrderInDup[i] = -1;
            }
            for (int i = 0; i < duplicatedVertIndices.Count; i++)
            {
                foreach (int vertIndex in duplicatedVertIndices[i])
                {
                    vertOrderInDup[vertIndex] = i;
                }
            }
            return vertOrderInDup;
        }

        //-------------------------------------------------------------------------------CGNR法終わり--------------------------------------------

        //エネルギー関数は[0]、その一回微分は[1]～[3]
        public static double[] EffectToGroupBCEEnergyJacobi(CutMesh cutMesh, double[] angleSum, int vertIndex, int effectVert, int ratio, bool moveX, bool moveY, bool moveZ, double w1)
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
            double[] Jacobi = new double[4];

            double E1 = 2 * Math.PI - ratio * angleSum[effectVert] * 0.5;
            double factor = E1 * -0.5 * ratio * w1;
            Jacobi[0] = w1 * 0.5 * E1 * E1;
            Jacobi[1] = sX * factor;
            Jacobi[2] = sY * factor;
            Jacobi[3] = sZ * factor;
            return Jacobi;
        }

        //constraintsとして入れるときのjacobi、x,y,zの長さ3*1,2,3で返す。
        //loopが１方向ミラーならpattern = 2,2方向ミラーならpattern = 1,Fならpattern = 3
        public static double[] VertGroupDFEnergyJacobi(CutMesh cutMesh, double averageLength, double[] angleSum,
            List<double> loopLength, List<double> loopCos, List<double> loopSin, List<int> vertGroup,
            List<int> loopVertices, int pattern, bool moveX, bool moveY, bool moveZ,
            double w2, double w3, double w4, double w5, double w6)
        {
            List<int> nextVertices = new List<int>();
            foreach (int vertIndex in vertGroup)
            {
                List<int> next = (cutMesh.GetVerticesForVertex(vertIndex).Intersect(loopVertices)).Except(nextVertices).ToList();
                nextVertices.AddRange(next);
            }
            int n = loopVertices.Count;
            List<Point3d> vertices = cutMesh.Vertices;
            double sX = 0;
            double sY = 0;
            double sZ = 0;

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

            double[] Jacobi = new double[4];
            double calc = 0;
            double angleSumSum = 0;
            foreach (int v in loopVertices)
            {
                angleSumSum += angleSum[v];
            }
            if (pattern == 1) 
            {
                double E1 = (n * Math.PI - angleSumSum) * (n + 1) / n;
                calc = w6 * E1;
                Jacobi[0] = E1 * calc * 0.5;
            }
            if (pattern == 2)
            {
                double E1 = (n * Math.PI - angleSumSum) * (n + 1) / n;
                calc = w4 * E1;
                Jacobi[0] = E1 * calc * 0.5;
            }
            if (pattern == 3)
            {
                double E1 = ((n + 2) * Math.PI - angleSumSum) * (n + 4) / (n + 2);
                calc = w2 * E1;
                Jacobi[0] = E1 * calc * 0.5;
            }


            Jacobi[1] -= (sX + sX2) * calc;
            Jacobi[2] -= (sY + sY2) * calc;
            Jacobi[3] -= (sZ + sZ2) * calc;

            //2方向ミラーの場合
            if (pattern == 1) { return Jacobi; }
            //1方向ミラーの場合
            else if (pattern == 2)
            {
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
                double tZ2 = 0;
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
                //double value2 = s * count * 4 * Math.PI / (averageLength * averageLength)


                double E1 = (n + 1) * Math.PI * s / averageLength;
                double value2 = w5 * E1;
                Jacobi[0] += value2 * E1 * 0.5;
                Jacobi[1] += value2 * sumX;
                Jacobi[2] += value2 * sumY;
                Jacobi[3] += value2 * sumZ;
            }
            //Ｆの場合
            else
            {
                int count = loopVertices.Count;

                //まずsinの条件式の方から
                double s = 0;
                for (int j = 1; j < count; j++)
                {
                    if (j % 2 == 0) { s += loopLength[j + 1] * loopSin[j - 1]; }
                    else { s -= loopLength[j + 1] * loopSin[j - 1]; }
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
                //double value2 = 2 * s * count * 4 * Math.PI / (3 * averageLength * averageLength);

                double E1 = (n + 4) * Math.PI * s / averageLength;
                double value2 = w3 * E1;
                Jacobi[0] += value2 * E1 * 0.5;
                Jacobi[1] += value2 * sumX;
                Jacobi[2] += value2 * sumY;
                Jacobi[3] += value2 * sumZ;

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

                E1 = (n + 4) * Math.PI * s / averageLength;
                value2 = w3 * E1;
                Jacobi[0] += value2 * E1 * 0.5;
                Jacobi[1] += value2 * sumX;
                Jacobi[2] += value2 * sumY;
                Jacobi[3] += value2 * sumZ;
            }

            return Jacobi;
        }

        //constraintsとして入れるときのjacobi、x,y,zの長さ3*1,2,3で返す。
        //loopが１方向ミラーならpattern = 2,2方向ミラーならpattern = 1,Fならpattern = 3
        public static double[] EffectToGroupDFEnergyJacobi(CutMesh cutMesh, double[] angleSum,
            List<double> loopLength, List<double> loopCos, List<double> loopSin, List<int> vertGroup, List<int> effectVertices,
            List<int> loopVertices, int pattern, bool moveX, bool moveY, bool moveZ,
            double w2, double w3, double w4, double w5, double w6)
        {
            int n = loopVertices.Count;

            List<Point3d> vertices = cutMesh.Vertices;
            double sX = 0;
            double sY = 0;
            double sZ = 0;

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

            List<double> Jacobi = new List<double>();
            double calc = 0;
            if (pattern == 1) { calc = w6 * (n + 1) / n; }
            if (pattern == 2) { calc = w4 * (n + 1) / n; }
            if (pattern == 3) { calc = w2 * (n + 4) / (n + 2); }

            Jacobi.Add(-sX * calc);
            Jacobi.Add(-sY * calc);
            Jacobi.Add(-sZ * calc);
            //2方向ミラーの場合
            if (pattern == 1) { return Jacobi.ToArray(); }
            //1方向ミラーの場合
            else if (pattern == 2)
            {
                double averageLength = 0;
                for (int ii = 1; ii < loopLength.Count; ii++)
                {
                    averageLength += loopLength[ii];
                }
                averageLength = averageLength / (n - 1);
                int count = loopVertices.Count;//つまりnと同じ
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
                //double value2 = s * count * 4 * Math.PI / (averageLength * averageLength);
                double value2 = w5 * (n + 1) * Math.PI / averageLength;
                Jacobi.Add(value2 * sumX);
                Jacobi.Add(value2 * sumY);
                Jacobi.Add(value2 * sumZ);

            }
            //Ｆの場合
            else
            {
                double averageLength = 0;
                for (int ii = 1; ii < loopLength.Count; ii++)
                {
                    averageLength += loopLength[ii];
                }
                averageLength = averageLength / n;
                int count = loopVertices.Count;//つまりnと同じ
                double s = loopLength[1];
                for (int j = 1; j < count; j++)
                {
                    if (j % 2 == 0) { s += loopLength[j + 1] * loopCos[j - 1]; }
                    else { s -= loopLength[j + 1] * loopCos[j - 1]; }
                }

                //cosの条件式の方
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
                    double value = loopLength[j + 1] * loopSin[j - 1];
                    if (moveX) { if (j % 2 == 0) { sumX -= value * tX; } else { sumX += value * tX; } }
                    if (moveY) { if (j % 2 == 0) { sumY -= value * tY; } else { sumY += value * tY; } }
                    if (moveZ) { if (j % 2 == 0) { sumZ -= value * tZ; } else { sumZ += value * tZ; } }
                }
                //double value2 = 2 * s * count * count * Math.PI * Math.PI / (3 * averageLength * averageLength);
                //double value2 = 2 * s * 4 * count * Math.PI / (3 * averageLength * averageLength);
                double value2 = w3 * (n + 4) * Math.PI / averageLength;
                double aX = value2 * sumX;
                double aY = value2 * sumY;
                double aZ = value2 * sumZ;

                //sinの条件式の方
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

                Jacobi.Add(value2 * sumX);
                Jacobi.Add(value2 * sumY);
                Jacobi.Add(value2 * sumZ);
                Jacobi.Add(aX);
                Jacobi.Add(aY);
                Jacobi.Add(aZ);
            }
            return Jacobi.ToArray();
        }


        public static (List<List<List<int>>> effectDoneBlocks, List<List<List<int>>> effectDtwoBlocks, List<List<List<int>>> effectFblocks) EffectDFGrouping
            (CutMesh mesh, List<int> vertGroup, List<List<List<int>>> category, List<int> effectDF)
        {
            List<int> D = category[0][3];
            List<int> F = category[0][5];
            List<List<int>> DoneMirror = category[1];
            List<List<int>> DtwoMirror = category[5];
            List<List<int>> Fblocks = category[9];

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
            return (effectDoneBlocks, effectDtwoBlocks, effectFblocks);
        }






        //----------------------------------------LBFGSのための関数--------------------------------------------------------------------
        //effectVertのx,y,zでの偏微分の結果を計算
        public static double[] CalcDxDSigmaTheta(CutMesh cutMesh, double[] angleSum, int vertIndex, int effectVert, bool[] move)
        {
            if (move.All(m => m == false))
            {
                return new double[] { 0, 0, 0 };
            }


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
                    if (abSinTheta < 1e-9)
                    {
                        // 縮退している場合は勾配を0として返す（暴れるのを防ぐ）
                        continue; // foreach内ならcontinue, または return new double[]{0,0,0};
                    }
                    double dot = a * b;
                    double abaa = 1 - dot / a.SquareLength;
                    double abbb = 1 - dot / b.SquareLength;
                    double wa = abaa + abbb;
                    if (move[0]) { sX += (abaa * p1.X + abbb * p2.X - wa * p3.X) / abSinTheta; }
                    if (move[1]) { sY += (abaa * p1.Y + abbb * p2.Y - wa * p3.Y) / abSinTheta; }
                    if (move[2]) { sZ += (abaa * p1.Z + abbb * p2.Z - wa * p3.Z) / abSinTheta; }

                }
            }

            else
            {
                int edgeIndex = cutMesh.GetEdgeForEndPoints(vertIndex, effectVert);
                if (edgeIndex == -1) { return new double[] { 0, 0, 0 }; }
                List<int> faces = cutMesh.GetFacesForEdge(edgeIndex);
                foreach (int face in faces)
                {
                    int anotherVert = cutMesh.GetVerticesForFace(face).Where(v => v != vertIndex && v != effectVert).ToList()[0];
                    Point3d p1 = vertices[vertIndex];
                    Point3d p2 = vertices[effectVert];
                    Point3d p3 = vertices[anotherVert];
                    Vector3d a = p2 - p1;
                    Vector3d b = p3 - p1;

                    double abaa = (a * b) / a.SquareLength;
                    double abSinTheta = Vector3d.CrossProduct(a, b).Length;
                    if (abSinTheta < 1e-9)
                    {
                        // 縮退している場合は勾配を0として返す（暴れるのを防ぐ）
                        continue; // foreach内ならcontinue, または return new double[]{0,0,0};
                    }
                    Vector3d jacobi = (-b + abaa * a) / abSinTheta;

                    if (move[0]) { sX += jacobi.X; }
                    if (move[1]) { sY += jacobi.Y; }
                    if (move[2]) { sZ += jacobi.Z; }
                }
            }
            return new double[] {sX, sY, sZ};
        }

        //ratioは、B:4, C:2, E:1
        public static void BCEVertTerm(ref double obj, ref double[] g, 
            CutMesh mesh, List<bool[]> moveList, int vert, double[] angleSum, int[] vertOrderInDup, double ratio, double w1)
        {
            bool[] vertMove = moveList[vert];
            double E1 = 2 * Math.PI - ratio * angleSum[vert];
            double factor = E1 * w1;
            obj += 0.5 * E1 * factor;
            if (g == null) { return; }
            double[] vertDiff = CalcDxDSigmaTheta(mesh, angleSum, vert, vert, vertMove);

            int vertInDup = vertOrderInDup[vert];
            int vi = vertInDup * 3;
            double factor2 = -factor * ratio;
            g[vi] += factor2 * vertDiff[0];
            g[vi + 1] += factor2 * vertDiff[1];
            g[vi + 2] += factor2 * vertDiff[2];

            List<int> connectedVertices = mesh.GetVerticesForVertex(vert);
            foreach (int effectVert in connectedVertices)
            {
                int effectInDup = vertOrderInDup[effectVert];
                bool[] effectMove = moveList[effectVert];
                double[] effectDiff = CalcDxDSigmaTheta(mesh, angleSum, vert, effectVert, effectMove);
                int ei = effectInDup * 3;
                g[ei] += factor2 * effectDiff[0];
                g[ei + 1] += factor2 * effectDiff[1];
                g[ei + 2] += factor2 * effectDiff[2];
            }
        }

        //Dならk1 =0, k2 = 1, Fならk1 = 2, k2 = 4 
        public static void DFVertAngleTerm(ref double obj, ref double[] g,
            CutMesh mesh, List<bool[]> moveList, int[] loop, double[] angleSum, int[] vertOrderInDup, int k1, int k2, double w)
        {
            int n = loop.Length;
            double angleSumSum = 0;
            foreach (int vert in loop)
            {
                angleSumSum += angleSum[vert];
            }

            double E1 = (n + k2) * ((n + k1) * Math.PI - angleSumSum) / (n + k1);
            double factor = E1 * w;
            obj += 0.5 * E1 * factor;
            if (g == null) { return; }
            double factor2 = -factor * (n + k2) / (n + k1);

            foreach (int vert in loop)
            {
                bool[] vertMove = moveList[vert];
                int vertInDup = vertOrderInDup[vert];
                double[] vertDiff = CalcDxDSigmaTheta(mesh, angleSum, vert, vert, vertMove);

                int vi = vertInDup * 3;
                g[vi] += factor2 * vertDiff[0];
                g[vi + 1] += factor2 * vertDiff[1];
                g[vi + 2] += factor2 * vertDiff[2];

                List<int> connectedVertices = mesh.GetVerticesForVertex(vert);
                foreach (int effectVert in connectedVertices)
                {
                    int effectInDup = vertOrderInDup[effectVert];
                    bool[] effectMove = moveList[effectVert];
                    double[] effectDiff = CalcDxDSigmaTheta(mesh, angleSum, vert, effectVert, effectMove);
                    int ei = effectInDup * 3;
                    g[ei] += factor2 * effectDiff[0];
                    g[ei + 1] += factor2 * effectDiff[1];
                    g[ei + 2] += factor2 * effectDiff[2];
                }
            }
        }

        public static double CalcDoneSinFactor(List<double> loopLength, List<double> loopSin)
        {
            int n = loopLength.Count;

            double s = 0;
            for (int j = 0; j < n - 1; j++)
            {
                if (j % 2 == 0) { s += loopLength[j + 1] * loopSin[j]; }
                else { s -= loopLength[j + 1] * loopSin[j]; }
            }
            return s;
        }

        public static double CalcFSinFactor(List<double> loopLength, List<double> loopSin)
        {
            int n = loopSin.Count;

            double s = 0;
            for (int j = 1; j < n + 1; j++)
            {
                if (j % 2 == 0) { s += loopLength[j + 1] * loopSin[j - 1]; }
                else { s -= loopLength[j + 1] * loopSin[j - 1]; }
            }
            return s;
        }

        public static double CalcFCosFactor(List<double> loopLength, List<double> loopCos)
        {
            int n = loopCos.Count;

            double s = loopLength[1];
            for (int j = 1; j < n + 1; j++)
            {
                if (j % 2 == 0) { s += loopLength[j + 1] * loopCos[j - 1]; }
                else { s -= loopLength[j + 1] * loopCos[j - 1]; }
            }
            return s;
        }

        public static (List<double> DoneAveLength, List<double> FblocksAveLength) CalcLoopLength(CutMesh mesh, List<List<int>> DoneMirror, List<List<int>> Fblocks)
        {
            List<double> DoneAveLength = new List<double>();
            for (int i = 0; i < DoneMirror.Count; i++)
            {
                List<int> loopVertices = DoneMirror[i];
                double length = 0;
                for (int j = 0; j < loopVertices.Count - 1; j++)
                {
                    length += (mesh.Vertices[loopVertices[j]] - mesh.Vertices[loopVertices[j + 1]]).Length;
                }
                DoneAveLength.Add(length / (loopVertices.Count - 1));
            }

            List<double> FblocksAveLength = new List<double>();
            for (int i = 0; i < Fblocks.Count; i++)
            {
                List<int> loopVertices = Fblocks[i];
                int loopVerticesCount = loopVertices.Count; 
                double length = 0;
                for (int j = 0; j < loopVerticesCount; j++)
                {
                    length += (mesh.Vertices[loopVertices[j]] - mesh.Vertices[loopVertices[(j + 1) % loopVerticesCount]]).Length;
                }
                FblocksAveLength.Add(length / loopVertices.Count);
            }
            return (DoneAveLength, FblocksAveLength);
        }

        //目的関数自体の値を返す。どんな式かはDevelop Mesh2のpdfのP6を参照、その式の2乗の和
        public static double CGNRObjectiveCalc(List<List<int>> duplicatedVertIndices, List<Point3d> vertices, double[] angleSum, List<List<List<int>>> category, double[] w,
            List<double> aveLengthDone, List<double> aveLengthF)
        {
            List<int> B = new List<int> (category[0][1]);
            List<int> C = new List<int> (category[0][2]);
            List<int> E = new List<int>(category[0][4]);
            List<List<int>> DoneMirror = new List<List<int>> (category[1]);
            List<List<int>> DtwoMirror = new List<List<int>>(category[5]);
            List<List<int>> Fblocks = new List<List<int>>(category[9]);

            double w1 = w[0];
            double w2 = w[1];
            double w3 = w[2];
            double w4 = w[3];
            double w5 = w[4];
            double w6 = w[5];

            double obj = 0;
            while (B.Count > 0)
            {
                int vert = B[0];
                double a = 2 * Math.PI - 4 * angleSum[vert];
                obj += 0.5 * w1 * a * a;
                List<int> found = duplicatedVertIndices.FirstOrDefault(sublist => sublist.Contains(vert));
                B.RemoveAt(0);
                if (found != null) { B = B.Except(found).ToList(); }
            }

            while (C.Count > 0)
            {
                int vert = C[0];
                double a = 2 * Math.PI - 2 * angleSum[vert];
                obj += 0.5 * w1 * a * a;
                List<int> found = duplicatedVertIndices.FirstOrDefault(sublist => sublist.Contains(vert));
                C.RemoveAt(0);
                if (found != null) { C = C.Except(found).ToList(); }
            }

            while (E.Count > 0)
            {
                int vert = E[0];
                double a = 2 * Math.PI - angleSum[vert];
                obj += 0.5 * w1 * a * a;
                List<int> found = duplicatedVertIndices.FirstOrDefault(sublist => sublist.Contains(vert));
                E.RemoveAt(0);
                if (found != null) { E = E.Except(found).ToList(); }
            }

            foreach (List<int> Dtwo in DtwoMirror)
            {
                int n = Dtwo.Count;
                double sum = 0;
                foreach (int v in Dtwo)
                {
                    sum += angleSum[v];
                }
                double a = (n * Math.PI - sum) * (n + 1) / n;
                obj += 0.5 * w6 * a * a;
            }
            int i = 0;
            foreach (List<int> Done in DoneMirror)
            {
                
                int n = Done.Count;
                double sum = 0;
                foreach (int v in Done)
                {
                    sum += angleSum[v];
                }
                double a = (n * Math.PI - sum) * (n + 1) / n;
                obj += 0.5 * w4 * a * a;

                List<double> loopLength = new List<double> { Done[0] };
                List<double> loopCos = new List<double>();
                List<double> loopSin = new List<double>();
                double angleSumSum = 0;
                for (int j = 0; j < Done.Count - 1; j++)
                {
                    double length = (vertices[Done[j]] - vertices[Done[j + 1]]).Length;
                    angleSumSum += angleSum[Done[j]];
                    loopLength.Add(length);
                    loopCos.Add(Math.Cos(angleSumSum));
                    loopSin.Add(Math.Sin(angleSumSum));
                }
                int count = Done.Count;
                double averageLength = aveLengthDone[i];
                double s = 0;
                for (int j = 0; j < count - 1; j++)
                {
                    if (j % 2 == 0) { s += loopLength[j + 1] * loopSin[j]; }
                    else { s -= loopLength[j + 1] * loopSin[j]; }
                }
                double aa = (n + 1) * Math.PI * s / averageLength;
                obj += 0.5 * w5 * aa * aa;
                i++;
            }
            i = 0;
            foreach (List<int> Fblock in Fblocks)
            {
                int n = Fblock.Count;
                double sum = 0;
                foreach (int v in Fblock)
                {
                    sum += angleSum[v];
                }
                double a = (n + 4) * ((n + 2) * Math.PI - sum) / (n + 2);
                obj += 0.5 * w2 * a * a;

                List<double> loopLength = new List<double> { Fblock[0] };
                List<double> loopCos = new List<double>();
                List<double> loopSin = new List<double>();
                double angleSumSum = 0;
                int loopVerticesCount = Fblock.Count;
                for (int j = 0; j < loopVerticesCount; j++)
                {
                    double length = (vertices[Fblock[j]] - vertices[Fblock[(j + 1) % loopVerticesCount]]).Length;
                    loopLength.Add(length);
                    if (j == 0) { continue; }
                    angleSumSum += angleSum[Fblock[j]];
                    loopCos.Add(Math.Cos(angleSumSum));
                    loopSin.Add(Math.Sin(angleSumSum));
                }

                int count = Fblock.Count;
                double averageLength = aveLengthF[i];

                double s = 0;
                for (int j = 1; j < count; j++)
                {
                    if (j % 2 == 0) { s += loopLength[j + 1] * loopSin[j - 1]; }
                    else { s -= loopLength[j + 1] * loopSin[j - 1]; }
                }

                double c = loopLength[1];
                for (int j = 1; j < count; j++)
                {
                    if (j % 2 == 0) { c += loopLength[j + 1] * loopCos[j - 1]; }
                    else { c -= loopLength[j + 1] * loopCos[j - 1]; }
                }
                double factor = (n + 4) * Math.PI / averageLength;
                double ss = factor * s;
                double cc = factor * c;
                obj += 0.5 * w3 * (ss * ss + cc * cc);
                i++;
            }

            return obj;
        }


        //ratioは、B:4, C:2, E:1
        public static void BCEVertCGLS(ref ConstraintBuilder builder,
            CutMesh mesh, List<bool[]> moveList, int vert, double[] angleSum, int[] vertOrderInDup, double ratio, double w1)
        {
            int row = builder.AllocateRow();
            //builder.AddDerivative(myRowIndex1, i3 + 0, Weight);
            bool[] vertMove = moveList[vert];
            double E1 = 2 * Math.PI - ratio * angleSum[vert];
            double factor = E1 * w1;
            builder.AddResidual(row, -factor);
            double[] vertDiff = CalcDxDSigmaTheta(mesh, angleSum, vert, vert, vertMove);

            int vertInDup = vertOrderInDup[vert];
            int vi = vertInDup * 3;
            double factor2 = w1 * -ratio;
            if (vertMove[0]) builder.AddDerivative(row, vi + 0, factor2 * vertDiff[0]);
            if (vertMove[1]) builder.AddDerivative(row, vi + 1, factor2 * vertDiff[1]);
            if (vertMove[2]) builder.AddDerivative(row, vi + 2, factor2 * vertDiff[2]);

            List<int> connectedVertices = mesh.GetVerticesForVertex(vert);
            foreach (int effectVert in connectedVertices)
            {
                int effectInDup = vertOrderInDup[effectVert];
                bool[] effectMove = moveList[effectVert];
                double[] effectDiff = CalcDxDSigmaTheta(mesh, angleSum, vert, effectVert, effectMove);
                int ei = effectInDup * 3;
                if (effectMove[0]) builder.AddDerivative(row, ei + 0, factor2 * effectDiff[0]);
                if (effectMove[1]) builder.AddDerivative(row, ei + 1, factor2 * effectDiff[1]);
                if (effectMove[2]) builder.AddDerivative(row, ei + 2, factor2 * effectDiff[2]);
            }
        }

        //Dならk1 =0, k2 = 1, Fならk1 = 2, k2 = 4 
        public static void DFVertAngleCGLS(ref ConstraintBuilder builder,
            CutMesh mesh, List<bool[]> moveList, int[] loop, double[] angleSum, int[] vertOrderInDup, int k1, int k2, double w)
        {
            int n = loop.Length;
            double angleSumSum = 0;
            foreach (int vert in loop)
            {
                angleSumSum += angleSum[vert];
            }

            double E1 = (n + k2) * ((n + k1) * Math.PI - angleSumSum) / (n + k1);
            double factor = E1 * w;
            builder.AddResidual(-factor);

            double factor2 = -w * (n + k2) / (n + k1);

            foreach (int vert in loop)
            {
                bool[] vertMove = moveList[vert];
                int vertInDup = vertOrderInDup[vert];
                double[] vertDiff = CalcDxDSigmaTheta(mesh, angleSum, vert, vert, vertMove);

                int vi = vertInDup * 3;
                if (vertMove[0]) builder.AddDerivative(vi + 0, factor2 * vertDiff[0]);
                if (vertMove[1]) builder.AddDerivative(vi + 1, factor2 * vertDiff[1]);
                if (vertMove[2]) builder.AddDerivative(vi + 2, factor2 * vertDiff[2]);

                List<int> connectedVertices = mesh.GetVerticesForVertex(vert);
                foreach (int effectVert in connectedVertices)
                {
                    int effectInDup = vertOrderInDup[effectVert];
                    bool[] effectMove = moveList[effectVert];
                    double[] effectDiff = CalcDxDSigmaTheta(mesh, angleSum, vert, effectVert, effectMove);
                    int ei = effectInDup * 3;
                    if (effectMove[0]) builder.AddDerivative(ei + 0, factor2 * effectDiff[0]);
                    if (effectMove[1]) builder.AddDerivative(ei + 1, factor2 * effectDiff[1]);
                    if (effectMove[2]) builder.AddDerivative(ei + 2, factor2 * effectDiff[2]);
                }
            }
            builder.NextRow();
        }

        public static void DevelopCGLS(CutMesh cutMesh, List<int> sortedOuterVertIndices, int outerIter, int innerIter)
        {
            List<List<int>> duplicatedVertIndices = cutMesh.DuplicatedVertIndices;
            int dupVertCount = duplicatedVertIndices.Count;
            int variableCount = dupVertCount * 3;

            int[] vertOrderInDup = cutMesh.VertOrderInDup();
            List<List<int>> dupConnectedVertIndices = cutMesh.DupConnectedVertIndices();
            //List<int> sortedFixVertIndices = cutMesh.BoundaryVertIndices();
            List<int> sortedFixVertIndices = sortedOuterVertIndices;
            List<List<List<int>>> category = CategolizeCutMesh(cutMesh, false, false, false, sortedOuterVertIndices, sortedFixVertIndices);
            List<bool[]> moveList = CategoryToMoveList(cutMesh.Vertices.Count, category, sortedFixVertIndices);
            HashSet<int> unMoveSet = MoveListToUnMoveSet(moveList, vertOrderInDup);

            HashSet<int> unMoveSetSmooth = new HashSet<int>(unMoveSet);
            /*
            foreach (int i in sortedUnSmoothVertIndices)
            {
                int id = vertOrderInDup[i] * 3;
                unMoveSetSmooth.Add(id);
                unMoveSetSmooth.Add(id + 1);
                unMoveSetSmooth.Add(id + 2);
            }
            */
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

            //optimizer.AddTerm(new MinMaxLengthCGLS3D(lenRangeW, cullDupEdges, vertOrderInDup, initialLength, unMoveSet));

            optimizer.AddTerm(new EdgeLengthCGLS3D(10, cullDupEdges, vertOrderInDup, initialLength, unMoveSet));

            var devTerm = new DevelopableCGLS(100, cutMesh, category, aveLengthDone, aveLengthF, w, moveList, DoneMaps, FMaps);
            optimizer.AddTerm(devTerm);

            optimizer.AddTerm(new SmoothingCGLS(10, dupConnectedVertIndices, unMoveSetSmooth));

            optimizer.AddTerm(new VertMoveCGLS(1, dupInitialPosition, unMoveSet));
            // ----不等式制約の追加（別に等式と特段変わりはない、その制約のクラス内で条件満たすかどうかでの処理を書くかどうかの違い）---

            optimizer.AddTerm(new MinMaxLengthCGLS3D(10, cullDupEdges, vertOrderInDup, initialLength, unMoveSet));


            //計算実行
            optimizer.Step(outerIter: outerIter, innerIter: innerIter);
            //optimizer.Step(outerIter: 1, innerIter: 100);

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
        }
    }
}