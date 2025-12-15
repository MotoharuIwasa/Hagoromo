using GH_IO.Serialization;
using Grasshopper.GUI;
using Grasshopper.GUI.Canvas;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Attributes;
using Grasshopper.Kernel.Types;
using Rhino.Collections;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using static Rhino.DocObjects.HatchPattern;

namespace Hagoromo.DataStructure
{
    public class BeamElem : GH_Component
    {
        internal bool Toggle_X { get; set; }
        internal bool Toggle_Y { get; set; }
        internal bool Toggle_Z { get; set; }
        internal bool Toggle_RX { get; set; }
        internal bool Toggle_RY { get; set; }
        internal bool Toggle_RZ { get; set; }
        internal bool Toggle_X2 { get; set; }
        internal bool Toggle_Y2 { get; set; }
        internal bool Toggle_Z2 { get; set; }
        internal bool Toggle_RX2 { get; set; }
        internal bool Toggle_RY2 { get; set; }
        internal bool Toggle_RZ2 { get; set; }
        internal bool[] mouseHover;

        internal const string logText_X = "Condition X Changed.";
        internal const string logText_Y = "Condition Y Changed.";
        internal const string logText_Z = "Condition Z Changed.";
        internal const string logText_RX = "Condition TX Changed.";
        internal const string logText_RY = "Condition TY Changed.";
        internal const string logText_RZ = "Condition TZ Changed.";
        internal const string logText_X2 = "Condition X2 Changed.";
        internal const string logText_Y2 = "Condition Y2 Changed.";
        internal const string logText_Z2 = "Condition Z2 Changed.";
        internal const string logText_RX2 = "Condition TX2 Changed.";
        internal const string logText_RY2 = "Condition TY2 Changed.";
        internal const string logText_RZ2 = "Condition TZ2 Changed.";

        public BeamElem()
          : base("Beam Element Info", "BeamElem",
              "Gather beam element information,部材端の自由度は固定がTrue(=1)",
              "Hagoromo", "Input")
        {
            Toggle_X = true;
            Toggle_Y = true;
            Toggle_Z = true;
            Toggle_RX = true;
            Toggle_RY = true;
            Toggle_RZ = true;
            Toggle_X2 = true;
            Toggle_Y2 = true;
            Toggle_Z2 = true;
            Toggle_RX2 = true;
            Toggle_RY2 = true;
            Toggle_RZ2 = true;
            mouseHover = new bool[12];
        }

        public class Elemdata
        {
            public int SectId { get; set; }
            public List<bool> Constraint { get; set; }
            public List<double> CMQ { get; set; }
            public List<object> RotInfo { get; set; }
            public List<Line> Lines { get; set; }

            public Elemdata(int sectId, List<bool> constraint, List<double> cmq, List<object> rotInfo, List<Line> lines)
            {
                SectId = sectId;
                Constraint = constraint;
                CMQ = cmq;
                RotInfo = rotInfo;
                Lines = lines;
            }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            var defaultConstraints = new List<bool>(System.Linq.Enumerable.Repeat(true, 12));
            var defaultCMQ = new List<double>(System.Linq.Enumerable.Repeat(0.0, 6));

            pManager.AddLineParameter("Lines", "Lines", "The lines to calculate nodes", GH_ParamAccess.list);
            pManager.AddIntegerParameter("SectId", "SId", "Section Id of the element", GH_ParamAccess.item, 1);
            pManager.AddNumberParameter("CoordAngle", "Ang", "Coord angle Value(degree)", GH_ParamAccess.item, 0.0);
            pManager.AddNumberParameter("CMQ(STA)", "CMQ(STA)", "CMQ values of start node", GH_ParamAccess.list, defaultCMQ);
            pManager.AddNumberParameter("CMQ(END)", "CMQ(END)", "CMQ values of end node", GH_ParamAccess.list, defaultCMQ);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Elemdata", "e", "Exported element data", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Line> lines = new List<Line>();
            int sectId = 0;
            double coordAngle = 0.0;
            List<double> cmq1 = new List<double>();
            List<double> cmq2 = new List<double>();

            if (!DA.GetDataList(0, lines)) return;
            if (!DA.GetData(1, ref sectId)) return;
            if (!DA.GetData(2, ref coordAngle)) return;
            if (!DA.GetDataList(3, cmq1)) return;
            if (!DA.GetDataList(4, cmq2)) return;
            List<double> cmq = new List<double>();
            cmq.AddRange(cmq1);
            cmq.AddRange(cmq2);

            bool[] constraints = new bool[12];
            if (Toggle_X) constraints[0] = true; else constraints[0] = false;
            if (Toggle_Y) constraints[1] = true; else constraints[1] = false;
            if (Toggle_Z) constraints[2] = true; else constraints[2] = false;
            if (Toggle_RX) constraints[3] = true; else constraints[3] = false;
            if (Toggle_RY) constraints[4] = true; else constraints[4] = false;
            if (Toggle_RZ) constraints[5] = true; else constraints[5] = false;
            if (Toggle_X2) constraints[6] = true; else constraints[6] = false;
            if (Toggle_Y2) constraints[7] = true; else constraints[7] = false;
            if (Toggle_Z2) constraints[8] = true; else constraints[8] = false;
            if (Toggle_RX2) constraints[9] = true; else constraints[9] = false;
            if (Toggle_RY2) constraints[10] = true; else constraints[10] = false;
            if (Toggle_RZ2) constraints[11] = true; else constraints[11] = false;

            // Use a list of lists to store rotInfo for each line
            List<object> rotInfo = new List<object>();

            for (int i = 0; i < lines.Count; i++)
            {
                Line line = lines[i];
                double dx = line.To.X - line.From.X;
                double dy = line.To.Y - line.From.Y;
                double dz = line.To.Z - line.From.Z;
                double length = line.Length;
                double angle = coordAngle;

                List<double> lineRotInfo = new List<double> { angle, length, dx, dy, dz };

                rotInfo.Add(lineRotInfo);
            }

            //その部材が与えられた部材端条件で安定かどうかチェック
            if(!(Toggle_RZ) && !(Toggle_RZ2))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "rotation around Z axis should be fixed");
            };

            if (!(Toggle_X) && !(Toggle_X2))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "X direction movement should be fixed");
            };

            if (!(Toggle_Y) && !(Toggle_Y2))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Y direction movement should be fixed");
            };

            if (!(Toggle_Z) && !(Toggle_Z2))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Z direction movement should be fixed");
            };

            if (!(Toggle_RX) && !(Toggle_RX2))
            {
                if (!(Toggle_Y) || !(Toggle_Y2))
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "rotation around X axis should be fixed");
                };                
            }

            if (!(Toggle_RY) && !(Toggle_RY2))
            {
                if (!(Toggle_X) || !(Toggle_X2))
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "rotation around Y axis should be fixed");
                };
            }

            List<bool> constraints_list = new List<bool>(constraints);
            Elemdata elemdata = new Elemdata(sectId, constraints_list, cmq, rotInfo, lines);

            DA.SetData(0, new GH_Elemdata(elemdata));
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
            get { return new Guid("A19A028B-65AD-4F65-9C6D-4F2724A211A0"); }
        }


        public override bool Write(GH_IWriter writer)
        {
            writer.SetBoolean(logText_X, this.Toggle_X);
            writer.SetBoolean(logText_Y, this.Toggle_Y);
            writer.SetBoolean(logText_Z, this.Toggle_Z);
            writer.SetBoolean(logText_RX, this.Toggle_RX);
            writer.SetBoolean(logText_RY, this.Toggle_RY);
            writer.SetBoolean(logText_RZ, this.Toggle_RZ);
            writer.SetBoolean(logText_X2, this.Toggle_X);
            writer.SetBoolean(logText_Y2, this.Toggle_Y);
            writer.SetBoolean(logText_Z2, this.Toggle_Z);
            writer.SetBoolean(logText_RX2, this.Toggle_RX);
            writer.SetBoolean(logText_RY2, this.Toggle_RY);
            writer.SetBoolean(logText_RZ2, this.Toggle_RZ);
            return base.Write(writer);
        }
        public override bool Read(GH_IReader reader)
        {
            Toggle_X = reader.GetBoolean(logText_X);
            Toggle_Y = reader.GetBoolean(logText_Y);
            Toggle_Z = reader.GetBoolean(logText_Z);
            Toggle_RX = reader.GetBoolean(logText_RX);
            Toggle_RY = reader.GetBoolean(logText_RY);
            Toggle_RZ = reader.GetBoolean(logText_RZ);
            Toggle_X2 = reader.GetBoolean(logText_X2);
            Toggle_Y2 = reader.GetBoolean(logText_Y2);
            Toggle_Z2 = reader.GetBoolean(logText_Z2);
            Toggle_RX2 = reader.GetBoolean(logText_RX2);
            Toggle_RY2 = reader.GetBoolean(logText_RY2);
            Toggle_RZ2 = reader.GetBoolean(logText_RZ2);
            return base.Read(reader);
        }

        public override void CreateAttributes()
        {
            m_attributes = new Attributes_ICON(this);
        }

        public class Attributes_ICON : Grasshopper.Kernel.Attributes.GH_ComponentAttributes
        {
            public Attributes_ICON(BeamElem owner) : base(owner) { }

            protected override void Layout()
            {
                base.Layout();
                Rectangle[] recs1 = new Rectangle[6];
                Rectangle[] dots1 = new Rectangle[6];
                Rectangle[] texts1 = new Rectangle[6];

                Rectangle[] recs2 = new Rectangle[6];
                Rectangle[] dots2 = new Rectangle[6];
                Rectangle[] texts2 = new Rectangle[6];

                Rectangle rec0 = GH_Convert.ToRectangle(Bounds);
                rec0.Height += 90;
                //rec0.Width = 100;
                int center = ((int)Bounds.Left + (int)Bounds.Right) / 2;
                int circlesize = 14;

                Rectangle rec1 = rec0;
                rec1.Y = rec1.Bottom - 90;
                rec1.Height = 20;
                rec1.Inflate(-2, -2);

                Rectangle rec2 = rec0;
                rec2.Y = rec2.Bottom - 72;
                rec2.Height = 28;
                rec2.Inflate(-2, -2);

                Rectangle rec3 = rec0;
                rec3.Y = rec3.Bottom - 46;
                rec3.Height = 20;
                rec3.Inflate(-2, -2);

                Rectangle rec4 = rec0;
                rec4.Y = rec4.Bottom - 28;
                rec4.Height = 28;
                rec4.Inflate(-2, -2);

                for (int i = 0; i < 6; i++)
                {
                    texts1[i].Y = rec0.Bottom - 24 - 44;
                    texts1[i].Height = 16;
                    texts1[i].Width = 16;
                    texts1[i].X = center - 3 * circlesize + i * circlesize + 4;

                    recs1[i].Y = rec0.Bottom - circlesize - 2 - 44;
                    recs1[i].Height = circlesize;
                    recs1[i].Width = circlesize;
                    recs1[i].X = center - 3 * circlesize + i * circlesize;
                    recs1[i].Inflate(-2, -2);

                    dots1[i].Y = rec0.Bottom - circlesize - 2 - 44;
                    dots1[i].Height = circlesize;
                    dots1[i].Width = circlesize;
                    dots1[i].X = center - 3 * circlesize + i * circlesize;
                    dots1[i].Inflate(-4, -4);
                }

                for (int i = 0; i < 6; i++)
                {
                    texts2[i].Y = rec0.Bottom - 24;
                    texts2[i].Height = 16;
                    texts2[i].Width = 16;
                    texts2[i].X = center - 3 * circlesize + i * circlesize + 4;

                    recs2[i].Y = rec0.Bottom - circlesize - 2;
                    recs2[i].Height = circlesize;
                    recs2[i].Width = circlesize;
                    recs2[i].X = center - 3 * circlesize + i * circlesize;
                    recs2[i].Inflate(-2, -2);

                    dots2[i].Y = rec0.Bottom - circlesize - 2;
                    dots2[i].Height = circlesize;
                    dots2[i].Width = circlesize;
                    dots2[i].X = center - 3 * circlesize + i * circlesize;
                    dots2[i].Inflate(-4, -4);
                }


                Bounds = rec0;
                ButtonBounds1 = rec1;
                ButtonBounds2 = rec2;
                ButtonBounds3 = rec3;
                ButtonBounds4 = rec4;
                ButtonBoundses1 = recs1;
                DotBoundses1 = dots1;
                TextBoundses1 = texts1;
                ButtonBoundses2 = recs2;
                DotBoundses2 = dots2;
                TextBoundses2 = texts2;
            }
            private Rectangle ButtonBounds1 { get; set; }
            private Rectangle ButtonBounds2 { get; set; }
            private Rectangle ButtonBounds3 { get; set; }
            private Rectangle ButtonBounds4 { get; set; }
            private Rectangle[] ButtonBoundses1 { get; set; }
            private Rectangle[] DotBoundses1 { get; set; }
            private Rectangle[] TextBoundses1 { get; set; }
            private Rectangle[] ButtonBoundses2 { get; set; }
            private Rectangle[] DotBoundses2 { get; set; }
            private Rectangle[] TextBoundses2 { get; set; }

            protected override void Render(GH_Canvas canvas, System.Drawing.Graphics graphics, GH_CanvasChannel channel)
            {
                BeamElem comp = Owner as BeamElem;

                List<SolidBrush> brushs_dot1 = new List<SolidBrush> { new SolidBrush(Color.White), new SolidBrush(Color.White), new SolidBrush(Color.White), new SolidBrush(Color.White), new SolidBrush(Color.White), new SolidBrush(Color.White) };
                List<SolidBrush> brushs_dot2 = new List<SolidBrush> { new SolidBrush(Color.White), new SolidBrush(Color.White), new SolidBrush(Color.White), new SolidBrush(Color.White), new SolidBrush(Color.White), new SolidBrush(Color.White) };

                List<SolidBrush> brushs_background1 = new List<SolidBrush>(brushs_dot2);
                List<SolidBrush> brushs_background2 = new List<SolidBrush>(brushs_dot2);
                List<string> strings = new List<string> { "X", "Y", "Z", "Rx", "Ry", "Rz" };
                SolidBrush mybrush_brank = new SolidBrush(Color.White);
                SolidBrush mybrush_x = new SolidBrush(Color.Transparent);
                SolidBrush mybrush_selected = new SolidBrush(Color.Black);
                SolidBrush mybrush_hoverd = new SolidBrush(Color.LightBlue);
                Pen mypen0 = new Pen(mybrush_selected, 1);
                Font myfont0 = new Font("MS UI Gothic", 3);

                if (comp.Toggle_X) brushs_dot1[0] = mybrush_selected; else brushs_dot1[0] = mybrush_x;
                if (comp.Toggle_Y) brushs_dot1[1] = mybrush_selected; else brushs_dot1[1] = mybrush_x;
                if (comp.Toggle_Z) brushs_dot1[2] = mybrush_selected; else brushs_dot1[2] = mybrush_x;
                if (comp.Toggle_RX) brushs_dot1[3] = mybrush_selected; else brushs_dot1[3] = mybrush_x;
                if (comp.Toggle_RY) brushs_dot1[4] = mybrush_selected; else brushs_dot1[4] = mybrush_x;
                if (comp.Toggle_RZ) brushs_dot1[5] = mybrush_selected; else brushs_dot1[5] = mybrush_x;

                if (comp.Toggle_X2) brushs_dot2[0] = mybrush_selected; else brushs_dot2[0] = mybrush_x;
                if (comp.Toggle_Y2) brushs_dot2[1] = mybrush_selected; else brushs_dot2[1] = mybrush_x;
                if (comp.Toggle_Z2) brushs_dot2[2] = mybrush_selected; else brushs_dot2[2] = mybrush_x;
                if (comp.Toggle_RX2) brushs_dot2[3] = mybrush_selected; else brushs_dot2[3] = mybrush_x;
                if (comp.Toggle_RY2) brushs_dot2[4] = mybrush_selected; else brushs_dot2[4] = mybrush_x;
                if (comp.Toggle_RZ2) brushs_dot2[5] = mybrush_selected; else brushs_dot2[5] = mybrush_x;

                for (int i = 0; i < 6; i++)
                {
                    if (comp.mouseHover[i]) brushs_background1[i] = mybrush_hoverd;
                    else brushs_background1[i] = mybrush_brank;
                }

                for (int i = 0; i < 6; i++)
                {
                    if (comp.mouseHover[i + 6]) brushs_background2[i] = mybrush_hoverd;
                    else brushs_background2[i] = mybrush_brank;
                }

                base.Render(canvas, graphics, channel);

                if (channel == GH_CanvasChannel.Objects)
                {
                    GH_Capsule name1 = GH_Capsule.CreateTextCapsule(ButtonBounds1, ButtonBounds1, GH_Palette.Black, "Constraints(STA)", 2, 0);
                    GH_Capsule background1 = GH_Capsule.CreateCapsule(ButtonBounds2, GH_Palette.Transparent);
                    GH_Capsule name2 = GH_Capsule.CreateTextCapsule(ButtonBounds3, ButtonBounds3, GH_Palette.Black, "Constraints(END)", 2, 0);
                    GH_Capsule background2 = GH_Capsule.CreateCapsule(ButtonBounds4, GH_Palette.Transparent);

                    name1.Render(graphics, Selected, Owner.Locked, false);
                    name1.Dispose();
                    background1.Render(graphics, Selected, Owner.Locked, false);
                    background1.Dispose();
                    name2.Render(graphics, Selected, Owner.Locked, false);
                    name2.Dispose();
                    background2.Render(graphics, Selected, Owner.Locked, false);
                    background2.Dispose();
                    for (int i = 0; i < 6; i++)
                    {
                        graphics.FillEllipse(brushs_background1[i], ButtonBoundses1[i]);
                        graphics.FillEllipse(brushs_dot1[i], DotBoundses1[i]);
                        graphics.DrawEllipse(mypen0, ButtonBoundses1[i]);
                        graphics.DrawString(strings[i], myfont0, mybrush_selected, TextBoundses1[i]);

                        graphics.FillEllipse(brushs_background2[i], ButtonBoundses2[i]);
                        graphics.FillEllipse(brushs_dot2[i], DotBoundses2[i]);
                        graphics.DrawEllipse(mypen0, ButtonBoundses2[i]);
                        graphics.DrawString(strings[i], myfont0, mybrush_selected, TextBoundses2[i]);
                    }

                }
            }

            public override GH_ObjectResponse RespondToMouseMove(GH_Canvas sender, GH_CanvasMouseEvent e)
            {
                BeamElem comp = Owner as BeamElem;
                for (int i = 0; i < 6; i++)
                {
                    if (((RectangleF)ButtonBoundses1[i]).Contains(e.CanvasLocation)
                        && !comp.mouseHover[i])
                    {
                        comp.mouseHover[i] = true;
                        Owner.ExpireSolution(true);
                        return GH_ObjectResponse.Handled;
                    }
                    else if (!((RectangleF)ButtonBoundses1[i]).Contains(e.CanvasLocation)
                        && comp.mouseHover[i])
                    {
                        comp.mouseHover[i] = false;
                        Owner.ExpireSolution(true);
                        return GH_ObjectResponse.Handled;
                    }
                }

                for (int i = 0; i < 6; i++)
                {
                    if (((RectangleF)ButtonBoundses2[i]).Contains(e.CanvasLocation)
                        && !comp.mouseHover[i + 6])
                    {
                        comp.mouseHover[i + 6] = true;
                        Owner.ExpireSolution(true);
                        return GH_ObjectResponse.Handled;
                    }
                    else if (!((RectangleF)ButtonBoundses2[i]).Contains(e.CanvasLocation)
                        && comp.mouseHover[i + 6])
                    {
                        comp.mouseHover[i + 6] = false;
                        Owner.ExpireSolution(true);
                        return GH_ObjectResponse.Handled;
                    }
                }

                return base.RespondToMouseMove(sender, e);
            }
            public override GH_ObjectResponse RespondToMouseDown(GH_Canvas sender, GH_CanvasMouseEvent e)
            {

                BeamElem comp = Owner as BeamElem;
                if (e.Button == System.Windows.Forms.MouseButtons.Left)
                {

                    if (((RectangleF)ButtonBoundses1[0]).Contains(e.CanvasLocation))
                    {
                        comp.RecordUndoEvent(logText_X);
                        if (comp.Toggle_X) comp.Toggle_X = false; else comp.Toggle_X = true;
                        Owner.ExpireSolution(true);
                        return GH_ObjectResponse.Handled;
                    }


                    if (((RectangleF)ButtonBoundses1[1]).Contains(e.CanvasLocation))
                    {
                        comp.RecordUndoEvent(logText_Y);
                        if (comp.Toggle_Y) comp.Toggle_Y = false; else comp.Toggle_Y = true;
                        Owner.ExpireSolution(true);
                        return GH_ObjectResponse.Handled;
                    }


                    if (((RectangleF)ButtonBoundses1[2]).Contains(e.CanvasLocation))
                    {
                        comp.RecordUndoEvent(logText_Z);
                        if (comp.Toggle_Z) comp.Toggle_Z = false; else comp.Toggle_Z = true;
                        Owner.ExpireSolution(true);
                        return GH_ObjectResponse.Handled;
                    }


                    if (((RectangleF)ButtonBoundses1[3]).Contains(e.CanvasLocation))
                    {
                        comp.RecordUndoEvent(logText_RX);
                        if (comp.Toggle_RX) comp.Toggle_RX = false; else comp.Toggle_RX = true;
                        Owner.ExpireSolution(true);
                        return GH_ObjectResponse.Handled;
                    }


                    if (((RectangleF)ButtonBoundses1[4]).Contains(e.CanvasLocation))
                    {
                        comp.RecordUndoEvent(logText_RY);
                        if (comp.Toggle_RY) comp.Toggle_RY = false; else comp.Toggle_RY = true;
                        Owner.ExpireSolution(true);
                        return GH_ObjectResponse.Handled;
                    }


                    if (((RectangleF)ButtonBoundses1[5]).Contains(e.CanvasLocation))
                    {
                        comp.RecordUndoEvent(logText_RZ);
                        if (comp.Toggle_RZ) comp.Toggle_RZ = false; else comp.Toggle_RZ = true;
                        Owner.ExpireSolution(true);
                        return GH_ObjectResponse.Handled;
                    }


                    if (((RectangleF)ButtonBoundses2[0]).Contains(e.CanvasLocation))
                    {
                        comp.RecordUndoEvent(logText_X2);
                        if (comp.Toggle_X2) comp.Toggle_X2 = false; else comp.Toggle_X2 = true;
                        Owner.ExpireSolution(true);
                        return GH_ObjectResponse.Handled;
                    }


                    if (((RectangleF)ButtonBoundses2[1]).Contains(e.CanvasLocation))
                    {
                        comp.RecordUndoEvent(logText_Y2);
                        if (comp.Toggle_Y2) comp.Toggle_Y2 = false; else comp.Toggle_Y2 = true;
                        Owner.ExpireSolution(true);
                        return GH_ObjectResponse.Handled;
                    }


                    if (((RectangleF)ButtonBoundses2[2]).Contains(e.CanvasLocation))
                    {
                        comp.RecordUndoEvent(logText_Z2);
                        if (comp.Toggle_Z2) comp.Toggle_Z2 = false; else comp.Toggle_Z2 = true;
                        Owner.ExpireSolution(true);
                        return GH_ObjectResponse.Handled;
                    }


                    if (((RectangleF)ButtonBoundses2[3]).Contains(e.CanvasLocation))
                    {
                        comp.RecordUndoEvent(logText_RX2);
                        if (comp.Toggle_RX2) comp.Toggle_RX2 = false; else comp.Toggle_RX2 = true;
                        Owner.ExpireSolution(true);
                        return GH_ObjectResponse.Handled;
                    }


                    if (((RectangleF)ButtonBoundses2[4]).Contains(e.CanvasLocation))
                    {
                        comp.RecordUndoEvent(logText_RY2);
                        if (comp.Toggle_RY2) comp.Toggle_RY2 = false; else comp.Toggle_RY2 = true;
                        Owner.ExpireSolution(true);
                        return GH_ObjectResponse.Handled;
                    }


                    if (((RectangleF)ButtonBoundses2[5]).Contains(e.CanvasLocation))
                    {
                        comp.RecordUndoEvent(logText_RZ2);
                        if (comp.Toggle_RZ2) comp.Toggle_RZ2 = false; else comp.Toggle_RZ2 = true;
                        Owner.ExpireSolution(true);
                        return GH_ObjectResponse.Handled;
                    }
                }

                return base.RespondToMouseDown(sender, e);
            }
        }
    }
}

