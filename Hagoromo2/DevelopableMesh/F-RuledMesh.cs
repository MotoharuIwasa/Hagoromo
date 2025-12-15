using Grasshopper.Kernel;
using Grasshopper.Kernel.Geometry.Delaunay;
using Hagoromo.GeometryTools;
using Hagoromo.MathTools;
using MathNet.Numerics.Distributions;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra.Double.Solvers;
using MathNet.Numerics.LinearAlgebra.Factorization;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Collections;
using Rhino.Render.ChangeQueue;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Windows.Forms.VisualStyles;

namespace Hagoromo.DevelopableMesh
{
    public static class RulingTools
    {
        public static double[,] vertLengthEach(CutMesh mesh2d)
        {
            List<Point3d> vert = mesh2d.Vertices;
            double[,] length = new double[vert.Count, vert.Count];
            for (int i = 0; i < vert.Count; i++)
            {
                for (int j = i; j < vert.Count; j++)
                {
                    double distance = vert[i].DistanceTo(vert[j]);
                    length[i, j] = distance;
                    length[j, i] = distance;
                }
            }
            return length;
        }

        //mesh3dはソートされていること前提とする、endVertは境界または折線上の節点のインデックス、internalの点にのみqMaxを与える
        public static int[] qMaxVertIndex(CutMesh mesh2d, CutMesh mesh3d, List<int> endVert)
        {
            double[,] length = vertLengthEach(mesh2d);
            //0.8倍の円と交わるだと計算多いので、0.7～0.9倍の範囲にある点を候補とする
            double[] closestEndDisMin = new double[mesh2d.Vertices.Count];
            double[] closestEndDisMax = new double[mesh2d.Vertices.Count];
            for (int i = 0; i < mesh2d.Vertices.Count; i++)
            {
                if (endVert.Contains(i)) { continue; }
                for (int j = 0; j < mesh2d.Vertices.Count; j++)
                {
                    if (endVert.Contains(j))
                    {
                        if (closestEndDisMin[i] == 0 || closestEndDisMin[i] > length[i, j] * 0.8)
                        {
                            closestEndDisMin[i] = length[i, j] * 0.7;
                            closestEndDisMax[i] = length[i, j] * 0.9;
                        }

                    }
                }
            }
            List<int> boundaryVert = mesh2d.BoundaryVertIndices();
            List<int> fullSet = Enumerable.Range(0, mesh2d.Vertices.Count).ToList();
            List<int> internalVert = fullSet.Except(boundaryVert).ToList();
            Vector3d[] vertNormal = CutMeshCalcTools.VertexNormal(mesh3d);
            int[] internalVertQmaxIndex = new int[internalVert.Count];
            for (int i = 0; i < internalVert.Count; i++)
            {
                internalVertQmaxIndex[i] = -1;
            }

            for (int ii = 0; ii < internalVert.Count; ii++)
            {
                int i = internalVert[ii];
                double sigma = double.MinValue;
                Vector3d pNormal = vertNormal[i];
                Point3d pPosition = mesh3d.Vertices[i];
                for (int j = 0; j < mesh2d.Vertices.Count; j++)
                {
                    double dist2d = length[i, j];
                    if (closestEndDisMin[i] < dist2d && dist2d < closestEndDisMax[i])
                    {
                        double sigmaNew = pNormal * vertNormal[j] + 0.1 * pPosition.DistanceTo(mesh3d.Vertices[j]) / dist2d;
                        if (sigma < sigmaNew)
                        {
                            sigma = sigmaNew;
                            internalVertQmaxIndex[ii] = j;
                        }
                    }
                }
            }
            return internalVertQmaxIndex;
        }

        /*論文では元のメッシュが全然可展面でないとして3次元メッシュ上で測地線をたどっているが今回はほとんど可展面を扱うので
        2dメッシュ上で測地線をたどって工程1-2を行う、ルーリングのスコアが小さい順に始点終点の点座標を、その点の乗っている
        エッジと内分比で返す。*/
        public static (List<int[]> OnEdge, List<double[]> Ratio, List<Vector3d[]> Normal) PossibleRulings(CutMesh mesh2d, CutMesh mesh3d, int[] qMax)
        {
            List<int> boundaryVert = mesh2d.BoundaryVertIndices();
            List<int> fullSet = Enumerable.Range(0, mesh2d.Vertices.Count).ToList();
            List<int> internalVert = fullSet.Except(boundaryVert).ToList();
            List<int[]> OnEdge = new List<int[]>();
            List<double[]> Ratio = new List<double[]>();
            List<Vector3d[]> Normal = new List<Vector3d[]>();
            List<double> score = new List<double>();

            Vector3d[] vertNormals = CutMeshCalcTools.VertexNormal(mesh3d);
            for (int i = 0; i < internalVert.Count; i++)
            {
                if (qMax[i] == -1) { continue; }
                Vector3d qNormalStart = new Vector3d(0, 0, 0);
                Vector3d qNormalEnd = new Vector3d(0, 0, 0);
                Vector3d pNormal = vertNormals[internalVert[i]];
                Point3d p1 = mesh2d.Vertices[internalVert[i]];
                Point3d p2 = mesh2d.Vertices[qMax[i]];
                Line line = new Line (p1, p2);
                Line line2 = new Line(p1, 2*p1-p2);
                List<int> arroundFaces = mesh2d.GetFacesForVertex(internalVert[i]);
                List<int> edgeList = new List<int>();
                List<int>arroundEdges = mesh2d.GetEdgesForVertex(internalVert[i]);
                foreach (int face in arroundFaces)
                {
                    int[] faceEdges = mesh2d.GetEdgesForFace(face);
                    for (int j = 0; j < faceEdges.Length; j++)
                    {
                        if (!arroundEdges.Contains(faceEdges[j])) { edgeList.Add(faceEdges[j]); break; }
                    }
                }

                int currentStartEdgeIndex = -1;
                double currentStartRatio = -1;
                int currentEndEdgeIndex = -1;
                double currentEndRatio = -1;
                int nextStartFace = -1;
                int nextEndFace = -1;
                double startFlag = 1;
                double endFlag = 1;

                double totalNormal = 0;
                //初期設定
                for (int j = 0;j < edgeList.Count; j++)
                {
                    int edgeIndex = edgeList[j];
                    Line edge = mesh2d.GetEdgeLine(edgeList[j]);
                    if (Rhino.Geometry.Intersect.Intersection.LineLine(edge, line, out double a, out double b) && 0 <= a && a <= 1 && 0 <= b && b <= 1)
                    {
                        currentStartEdgeIndex = edgeIndex;
                        currentStartRatio = a;
                        List<int> edgeFaces = mesh2d.GetFacesForEdge(edgeIndex);
                        nextStartFace = edgeFaces.Except(arroundFaces).FirstOrDefault();
                        qNormalStart = (1 - a) * vertNormals[mesh2d.Edges[edgeIndex][0]] + a * vertNormals[mesh2d.Edges[edgeIndex][1]];
                        totalNormal += pNormal * qNormalStart;
                        break;
                    }
                }

                for (int j = 0; j < edgeList.Count; j++)
                {
                    int edgeIndex = edgeList[j];
                    Line edge = mesh2d.GetEdgeLine(edgeIndex);
                    if (Rhino.Geometry.Intersect.Intersection.LineLine(edge, line2, out double a, out double b) && 0 <= a && a <= 1 && 0 <= b && b <= 1)
                    {
                        currentEndEdgeIndex = edgeIndex;
                        currentEndRatio = a;
                        List<int> edgeFaces = mesh2d.GetFacesForEdge(edgeIndex);
                        nextEndFace = edgeFaces.Except(arroundFaces).FirstOrDefault();
                        qNormalEnd = (1 - a) * vertNormals[mesh2d.Edges[edgeIndex][0]] + a * vertNormals[mesh2d.Edges[edgeIndex][1]];
                        totalNormal += pNormal * qNormalEnd;
                        break;
                    }
                }


                int vertCount = 2;
                //順番に近い順で交点を調べていく
                double flagValue = Math.Cos(Math.PI / 12);

                while (startFlag > flagValue)
                {
                    if (nextStartFace == -1) { startFlag = -1; break; }
                    int[] possibleEdges = mesh2d.GetEdgesForFace(nextStartFace).Except(new[] { currentStartEdgeIndex }).ToArray();
                    for (int j = 0; j < 2; j++)
                    {
                        int edgeIndex = possibleEdges[j];
                        Line edge = mesh2d.GetEdgeLine(edgeIndex);
                        if (Rhino.Geometry.Intersect.Intersection.LineLine(edge, line, out double a, out double b) && 0 < a && a < 1 && 0 < b && b < 1)
                        {
                            qNormalStart = (1 - a) * vertNormals[mesh2d.Edges[edgeIndex][0]] + a * vertNormals[mesh2d.Edges[edgeIndex][1]];
                            startFlag = pNormal * qNormalStart;
                            if (startFlag <= flagValue) { break; }
                            vertCount++;
                            totalNormal += startFlag;
                            currentStartEdgeIndex = edgeIndex;
                            currentStartRatio = a;
                            List<int> edgeFaces = mesh2d.GetFacesForEdge(edgeIndex);
                            if (edgeFaces.Count == 1) { startFlag = -1; }
                            else { nextStartFace = edgeFaces.Except(new[] { nextStartFace }).FirstOrDefault(); }
                            break;
                        }
                        startFlag = -1;
                    }
                }

                while (endFlag > flagValue)
                {
                    if (nextEndFace == -1) { endFlag = -1; break; }
                    int[] possibleEdges = mesh2d.GetEdgesForFace(nextEndFace).Except(new[] { currentEndEdgeIndex }).ToArray();
                    for (int j = 0; j < 2; j++)
                    {
                        int edgeIndex = possibleEdges[j];
                        Line edge = mesh2d.GetEdgeLine(edgeIndex);
                        if (Rhino.Geometry.Intersect.Intersection.LineLine(edge, line, out double a, out double b) && 0 < a && a < 1 && 0 < b && b < 1)
                        {
                            qNormalEnd = (1 - a) * vertNormals[mesh2d.Edges[edgeIndex][0]] + a * vertNormals[mesh2d.Edges[edgeIndex][1]];
                            endFlag = pNormal * qNormalEnd;
                            if (endFlag <= flagValue) { break; }
                            vertCount++;
                            totalNormal += startFlag;
                            currentEndEdgeIndex = edgeIndex;
                            currentEndRatio = a;
                            List<int> edgeFaces = mesh2d.GetFacesForEdge(edgeIndex);
                            if (edgeFaces.Count == 1) { startFlag = -1; }
                            else { nextEndFace = edgeFaces.Except(new[] { nextEndFace }).FirstOrDefault(); }
                            break;
                        }
                        endFlag = -1;
                    }
                }
                OnEdge.Add(new int[] { currentStartEdgeIndex, currentEndEdgeIndex });
                Ratio.Add(new double[] { currentStartRatio, currentEndRatio });
                Normal.Add(new Vector3d[] { qNormalStart,qNormalEnd });
                score.Add(totalNormal / vertCount);
            }
            List<int> sortedIndices = score.Select((s, i) => new { Score = s, Index = i }).OrderByDescending(x => x.Score).Select(x => x.Index).ToList();

            // scoreの高い順に並び替える
            List<int[]> sortedOnEdge = sortedIndices.Select(i => OnEdge[i]).ToList();
            List<double[]> sortedRatio = sortedIndices.Select(i => Ratio[i]).ToList();
            List<Vector3d[]> sortedNormal = sortedIndices.Select(i => Normal[i]).ToList();

            /*
            OnEdge = sortedOnEdge;
            Ratio = sortedRatio;
            Normal = sortedNormal;
            */

            return (OnEdge, Ratio, Normal);
        }

        //質のいいルーリングを厳選する、長方形幅2d、直線を分割したようなきれいなメッシュだとうまく回らないがそれ以外はいい感じ
        public static (List<int[]> OnEdge, List<double[]> Ratio) RulingChoice(CutMesh mesh3d, List<int[]> onEdge, List<double[]> ratio, List<Vector3d[]> normal, double d)
        {
            List<int[]> newOnEdge = new List<int[]>();
            List<double[]> newRatio = new List<double[]>();
            while (onEdge.Count > 0)
            {
                int[] currentEdge = onEdge[0];
                double[] currentRatio = ratio[0];
                Vector3d[] currentNormal = normal[0];
                onEdge.RemoveAt(0);
                ratio.RemoveAt(0);
                normal.RemoveAt(0);
                newOnEdge.Add(currentEdge);
                newRatio.Add(currentRatio);

                Vector3d midNormal = (currentNormal[0] + currentNormal[1]) / 2;
                midNormal.Unitize();
                Point3d p1 = (1 - currentRatio[0]) * mesh3d.Vertices[mesh3d.Edges[currentEdge[0]][0]] + currentRatio[0] * mesh3d.Vertices[mesh3d.Edges[currentEdge[0]][1]];
                Point3d p2 = (1 - currentRatio[1]) * mesh3d.Vertices[mesh3d.Edges[currentEdge[1]][0]] + currentRatio[1] * mesh3d.Vertices[mesh3d.Edges[currentEdge[1]][1]];
                Point3d p3 = (p1 + p2) / 2;
                Transform zTransform =Transform.Rotation(midNormal, Vector3d.ZAxis, p3);
                p1.Transform(zTransform);
                p2.Transform(zTransform);
                Vector3d p1p2 = p2 - p1;
                p1p2.Unitize();
                Transform xyTransform = Transform.Rotation(p1p2, Vector3d.XAxis, p3);
                p1.Transform(xyTransform);//x1,y1,z1に対応
                p2.Transform(xyTransform);//x2,y2,z2に対応
                double maxX = Math.Max(p1.X, p2.X);
                double minX = Math.Min(p1.X, p2.X);

                for (int i = onEdge.Count - 1; i >= 0; i--)
                {
                    Point3d a = (1 - ratio[i][0]) * mesh3d.Vertices[mesh3d.Edges[onEdge[i][0]][0]] + ratio[i][0] * mesh3d.Vertices[mesh3d.Edges[onEdge[i][0]][1]];
                    Point3d b = (1 - ratio[i][1]) * mesh3d.Vertices[mesh3d.Edges[onEdge[i][1]][0]] + ratio[i][1] * mesh3d.Vertices[mesh3d.Edges[onEdge[i][1]][1]];
                    a.Transform(zTransform);
                    a.Transform(xyTransform);
                    b.Transform(zTransform);
                    b.Transform(xyTransform);
                    double z = (p1.Z - a.Z) / (b.Z - a.Z);
                    double x = z * (b.X - a.X) + a.X;
                    double y = z * (b.Y - a.Y) + a.Y;
                    if ((0 < z && z < 1) && (0.75 * minX + 0.25 *maxX < x && x < 0.75 * maxX + 0.25 * minX) && (p1.Y -d < y && y < p1.Y + d))
                    {
                        onEdge.RemoveAt(i);
                        ratio.RemoveAt(i);
                        normal.RemoveAt(i);
                    }
                }
            }
            return (newOnEdge, newRatio);
        }
        
        //スナップして長方形を作る

        
    }
}
