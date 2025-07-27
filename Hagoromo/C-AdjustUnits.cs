using System;
using System.Collections.Generic;
using System.Drawing;

using Grasshopper.Kernel;
using Rhino.Geometry;
using Hagoromo.DataStructure;
using Hagoromo.MathTools;
using Hagoromo.StiffnessTools;


namespace Hagoromo.Solver
{

    public class AdjustUnits : GH_Component
    {
        public Alldata previousalldata = new Alldata();
        double[] U = new double[] { };
        public Vector3d previousgravity_vec = new Vector3d { };

        public AdjustUnits()
          : base("Adjust Units", "Adjust Units",
              "adjust input unit digits.入力の力、強制変位、モーメントは10^Fp N,10^Lp mm, 10^(Fp+Lp) Nmmの単位で入力する。このコンポーネントの出力のE,A,I,V,gはこの単位でないことに注意(x 10^±Epだけずれている)",
              "Hagoromo", "Linear")
        {
        }


        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddNumberParameter("Young's modulus", "E", "x10^Ed kN/m2", GH_ParamAccess.item, 0);
            pManager.AddIntegerParameter("E digit", "Ed", "Young's modulus digit", GH_ParamAccess.item, 0);
            pManager.AddNumberParameter("Density", "D", "x10^Dd kg/m3", GH_ParamAccess.item, 0);
            pManager.AddIntegerParameter("D digit", "Dd", "Density digit", GH_ParamAccess.item, 0);
            pManager.AddNumberParameter("Area", "A", "x10^Ad mm2", GH_ParamAccess.item, 0);
            pManager.AddIntegerParameter("A digit", "Ad", "Area digit", GH_ParamAccess.item, 0);
            pManager.AddNumberParameter("Inertia_x", "Ix", "x10^Ixd mm4", GH_ParamAccess.item, 0);
            pManager.AddIntegerParameter("Ix digit", "Ixd", "Inertia_x digit", GH_ParamAccess.item, 0);
            pManager.AddNumberParameter("Inertia_y", "Iy", "x10^Iyd mm4", GH_ParamAccess.item, 0);
            pManager.AddIntegerParameter("Iy digit", "Iyd", "Inertia_y digit", GH_ParamAccess.item, 0);
            pManager.AddNumberParameter("Saint-Venant", "V", "x10^Vd mm4", GH_ParamAccess.item, 0);
            pManager.AddIntegerParameter("V digit", "Vd", "Saint-Venant digit", GH_ParamAccess.item, 0);
            pManager.AddNumberParameter("Length", "L", "x10^Ld mm |average element length|", GH_ParamAccess.item, 0);
            pManager.AddIntegerParameter("L digit", "Ld", "Length digit", GH_ParamAccess.item, 0);
            pManager.AddVectorParameter("体積力の加速度ベクトルの和", "g", "体積力(重力)加速度ベクトル m/s2", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Force unit parameter", "Fp", "Force unit is 10^Fp N, 3:kN, 4:almmost tf", GH_ParamAccess.item, 0);
            pManager.AddIntegerParameter("Length unit parameter", "Lp", "Length unit is 10^Lp mm, 3:m", GH_ParamAccess.item, 0);
            pManager.AddIntegerParameter("E unit parameter", "Ep", "EA,EI,GVの値は変えないが、E,A,I,Vの値は変えて桁落ちを避ける。構造物でFp,Lp,Epの値は同じにしておく。", GH_ParamAccess.item, 0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Young's modulus", "E", "modified E", GH_ParamAccess.item);
            pManager.AddNumberParameter("Density", "D", "modified D", GH_ParamAccess.item);
            pManager.AddNumberParameter("Area", "A", "modified A", GH_ParamAccess.item);
            pManager.AddNumberParameter("Inertia_x", "Ix", "modified Ix", GH_ParamAccess.item);
            pManager.AddNumberParameter("Inertia_y", "Iy", "modified Iy", GH_ParamAccess.item);
            pManager.AddNumberParameter("Saint-Venant", "V", "modified V", GH_ParamAccess.item);
            pManager.AddNumberParameter("Length", "L", "modified L", GH_ParamAccess.item);
            pManager.AddVectorParameter("gravity", "g", "modified g", GH_ParamAccess.item);
            pManager.AddNumberParameter("Model Scale", "S", "model should be scaled by this value.", GH_ParamAccess.item);
            pManager.AddTextParameter("showing each value as text", "Txt", "Contents of combined data", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // Young's modulus
            double E = 0;
            int Ed = 0;
            DA.GetData(0, ref E);
            DA.GetData(1, ref Ed);

            // Density
            double D = 0;
            int Dd = 0;
            DA.GetData(2, ref D);
            DA.GetData(3, ref Dd);

            // Area
            double A = 0;
            int Ad = 0;
            DA.GetData(4, ref A);
            DA.GetData(5, ref Ad);

            // Inertia_x
            double Ix = 0;
            int Ixd = 0;
            DA.GetData(6, ref Ix);
            DA.GetData(7, ref Ixd);

            // Inertia_y
            double Iy = 0;
            int Iyd = 0;
            DA.GetData(8, ref Iy);
            DA.GetData(9, ref Iyd);

            // Saint-Venant torsional constant
            double V = 0;
            int Vd = 0;
            DA.GetData(10, ref V);
            DA.GetData(11, ref Vd);

            // Length
            double L = 0;
            int Ld = 0;
            DA.GetData(12, ref L);
            DA.GetData(13, ref Ld);

            Vector3d g = Vector3d.Zero;
            DA.GetData(14, ref g);

            // Fp, Lp
            int Fp = 0;
            int Lp = 0;
            int Ep = 0;
            DA.GetData(15, ref Fp);
            DA.GetData(16, ref Lp);
            DA.GetData(17, ref Ep);

            //実際の計算用で使う値を出力
            double E_actual = E * Math.Pow(10, Ed-3+2*Lp-Fp-Ep);
            double D_actual = D * Math.Pow(10, Dd-Fp-9+3*Lp);
            double A_actual = A * Math.Pow(10, Ad-2*Lp+Ep);
            double Ix_actual = Ix * Math.Pow(10, Ixd-4*Lp+Ep);
            double Iy_actual = Iy * Math.Pow(10, Iyd-4*Lp+Ep);
            double V_actual = V * Math.Pow(10, Vd-4*Lp+Ep);
            double L_actual = L * Math.Pow(10, Ld-Lp);
            Vector3d g_actual = g * Math.Pow(10, -Ep);
            double scale = Math.Pow(10, Ld - Lp);

            // テキスト形式で結果を整理
            List<string> textOutput = new List<string>
            {
                $"E = {E_actual.ToString("F3")}",
                $"D = {D_actual.ToString("F3")}",
                $"A = {A_actual.ToString("F3")}",
                $"Ix = {Ix_actual.ToString("F3")}",
                $"Iy = {Iy_actual.ToString("F3")}",
                $"V = {V_actual.ToString("F3")}",
                $"L = {L_actual.ToString("F3")}",
                $"g = ({g_actual.X.ToString("F3")}, {g_actual.Y.ToString("F3")}, {g_actual.Z.ToString("F3")})",
                $"Scale = {scale.ToString("F3")}",
                $"Length unit = 10^{Lp} mm",
                $"Force unit = 10^{Fp} N",
                $"ただし、Ep分だけ,A,I,Vは大きくなり、Eは小さくなる"
            };


            DA.SetData(0, E_actual);
            DA.SetData(1, D_actual);
            DA.SetData(2, A_actual);
            DA.SetData(3, Ix_actual);
            DA.SetData(4, Iy_actual);
            DA.SetData(5, V_actual);
            DA.SetData(6, L_actual);
            DA.SetData(7, g_actual);
            DA.SetData(8, scale);
            DA.SetDataList(9, textOutput);
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
            get { return new Guid("6DF1A9A4-0DEA-4B44-BFC7-798B38B9AADE"); }
        }
    }
}