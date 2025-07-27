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
    public class ICon_2 : GH_Component
    {
        internal bool Toggle_X { get; set; }
        internal bool Toggle_Y { get; set; }
        internal bool Toggle_Z { get; set; }
        internal bool Toggle_RX { get; set; }
        internal bool Toggle_RY { get; set; }
        internal bool Toggle_RZ { get; set; }
        internal bool[] mouseHover;

        internal const string logText_X = "Condition X Changed.";
        internal const string logText_Y = "Condition Y Changed.";
        internal const string logText_Z = "Condition Z Changed.";
        internal const string logText_RX = "Condition TX Changed.";
        internal const string logText_RY = "Condition TY Changed.";
        internal const string logText_RZ = "Condition TZ Changed.";

        internal int unit;
        public int Unit
        {
            get { return this.unit; }
            internal set
            {
                var doc = Rhino.RhinoDoc.ActiveDoc;
                this.unit = (int)doc.ModelUnitSystem;
            }
        }

        public ICon_2()
          : base("Icon2 Info", "Icon2",
              "Gather Icon2 information,True(=1)が固定、同じ固定条件のpointはまとめてpointsに入れてOK",
              "Hagoromo", "Linear")
        {
            Toggle_X = true;
            Toggle_Y = true;
            Toggle_Z = true;
            Toggle_RX = true;
            Toggle_RY = true;
            Toggle_RZ = true;
            mouseHover = new bool[6];
            Unit = 0;
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddPointParameter("Points", "Points", "Points defining the Icon", GH_ParamAccess.list);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Icondata", "ICon", "Exported Icon data", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Point3d> points = new List<Point3d>();
            bool[] constraints = new bool[6];
            if (Toggle_X) constraints[0] = true; else constraints[0] = false;
            if (Toggle_Y) constraints[1] = true; else constraints[1] = false;
            if (Toggle_Z) constraints[2] = true; else constraints[2] = false;
            if (Toggle_RX) constraints[3] = true; else constraints[3] = false;
            if (Toggle_RY) constraints[4] = true; else constraints[4] = false;
            if (Toggle_RZ) constraints[5] = true; else constraints[5] = false;

            List<bool> constraints_list = new List<bool>(constraints);
            // Retrieve inputs
            if (!DA.GetDataList(0, points)) return;

            // Icondata 配列を作成
            Icondata[] icondataArray = new Icondata[points.Count];
            for (int i = 0; i < points.Count; i++)
            {
                List<Point3d> singlePointList = new List<Point3d> { points[i] };
                icondataArray[i] = new Icondata(singlePointList, constraints_list);
            }

            List<GH_Icondata> ghIcondataList = new List<GH_Icondata>();
            for (int i = 0; i < icondataArray.Length; i++)
            {
                ghIcondataList.Add(new GH_Icondata(icondataArray[i]));
            }

            DA.SetDataList(0, ghIcondataList);

            //Message = "UNIT:[10^Lp mm]";
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
            get { return new Guid("58D86FE2-8C89-4801-A6E2-7B34611F30A5"); }
        }



        public override bool Write(GH_IWriter writer)
        {
            writer.SetBoolean(logText_X, this.Toggle_X);
            writer.SetBoolean(logText_Y, this.Toggle_Y);
            writer.SetBoolean(logText_Z, this.Toggle_Z);
            writer.SetBoolean(logText_RX, this.Toggle_RX);
            writer.SetBoolean(logText_RY, this.Toggle_RY);
            writer.SetBoolean(logText_RZ, this.Toggle_RZ);
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
            return base.Read(reader);
        }

        public override void CreateAttributes()
        {
            m_attributes = new Attributes_ICON(this);
        }

        public class Attributes_ICON : Grasshopper.Kernel.Attributes.GH_ComponentAttributes
        {
            public Attributes_ICON(ICon_2 owner) : base(owner) { }

            protected override void Layout()
            {
                base.Layout();
                Rectangle[] recs = new Rectangle[6];
                Rectangle[] dots = new Rectangle[6];
                Rectangle[] texts = new Rectangle[6];
                Rectangle rec0 = GH_Convert.ToRectangle(Bounds);
                rec0.Height += 44;
                rec0.Width = 102;
                int center = ((int)Bounds.Left + (int)Bounds.Right) / 2;
                int circlesize = 14;
                Rectangle rec1 = rec0;
                rec1.Y = rec1.Bottom - 46;
                rec1.Height = 20;
                rec1.Inflate(-2, -2);
                Rectangle rec2 = rec0;
                rec2.Y = rec0.Bottom - 28;
                rec2.Height = 28;
                rec2.Inflate(-2, -2);
                for (int i = 0; i < 6; i++)
                {
                    texts[i].Y = rec0.Bottom - 24;
                    texts[i].Height = 16;
                    texts[i].Width = 16;
                    texts[i].X = center - 3 * circlesize + i * circlesize + 4;
                    recs[i].Y = rec0.Bottom - circlesize - 2;
                    recs[i].Height = circlesize;
                    recs[i].Width = circlesize;
                    recs[i].X = center - 3 * circlesize + i * circlesize;
                    recs[i].Inflate(-2, -2);
                    dots[i].Y = rec0.Bottom - circlesize - 2;
                    dots[i].Height = circlesize;
                    dots[i].Width = circlesize;
                    dots[i].X = center - 3 * circlesize + i * circlesize;
                    dots[i].Inflate(-4, -4);
                }

                Bounds = rec0;
                ButtonBounds1 = rec1;
                ButtonBounds2 = rec2;
                ButtonBoundses = recs;
                DotBoundses = dots;
                TextBoundses = texts;
            }
            private Rectangle ButtonBounds1 { get; set; }
            private Rectangle ButtonBounds2 { get; set; }
            private Rectangle[] ButtonBoundses { get; set; }
            private Rectangle[] DotBoundses { get; set; }
            private Rectangle[] TextBoundses { get; set; }

            protected override void Render(GH_Canvas canvas, System.Drawing.Graphics graphics, GH_CanvasChannel channel)
            {
                ICon_2 comp = Owner as ICon_2;

                List<SolidBrush> brushs_dot = new List<SolidBrush> { new SolidBrush(Color.White), new SolidBrush(Color.White), new SolidBrush(Color.White), new SolidBrush(Color.White), new SolidBrush(Color.White), new SolidBrush(Color.White) };
                List<SolidBrush> brushs_background = new List<SolidBrush>(brushs_dot);
                List<string> strings = new List<string> { "X", "Y", "Z", "Rx", "Ry", "Rz" };
                SolidBrush mybrush_brank = new SolidBrush(Color.White);
                SolidBrush mybrush_x = new SolidBrush(Color.Transparent);
                SolidBrush mybrush_selected = new SolidBrush(Color.Black);
                SolidBrush mybrush_hoverd = new SolidBrush(Color.LightBlue);
                Pen mypen0 = new Pen(mybrush_selected, 1);
                Font myfont0 = new Font("MS UI Gothic", 3);

                if (comp.Toggle_X) brushs_dot[0] = mybrush_selected; else brushs_dot[0] = mybrush_x;
                if (comp.Toggle_Y) brushs_dot[1] = mybrush_selected; else brushs_dot[1] = mybrush_x;
                if (comp.Toggle_Z) brushs_dot[2] = mybrush_selected; else brushs_dot[2] = mybrush_x;
                if (comp.Toggle_RX) brushs_dot[3] = mybrush_selected; else brushs_dot[3] = mybrush_x;
                if (comp.Toggle_RY) brushs_dot[4] = mybrush_selected; else brushs_dot[4] = mybrush_x;
                if (comp.Toggle_RZ) brushs_dot[5] = mybrush_selected; else brushs_dot[5] = mybrush_x;

                for (int i = 0; i < 6; i++)
                {
                    if (comp.mouseHover[i]) brushs_background[i] = mybrush_hoverd;
                    else brushs_background[i] = mybrush_brank;
                }

                base.Render(canvas, graphics, channel);

                if (channel == GH_CanvasChannel.Objects)
                {
                    GH_Capsule name = GH_Capsule.CreateTextCapsule(ButtonBounds1, ButtonBounds1, GH_Palette.Black, "CONDITION", 2, 0);
                    GH_Capsule background = GH_Capsule.CreateCapsule(ButtonBounds2, GH_Palette.Transparent);
                    name.Render(graphics, Selected, Owner.Locked, false);
                    name.Dispose();
                    background.Render(graphics, Selected, Owner.Locked, false);
                    background.Dispose();
                    for (int i = 0; i < 6; i++)
                    {
                        graphics.FillEllipse(brushs_background[i], ButtonBoundses[i]);
                        graphics.FillEllipse(brushs_dot[i], DotBoundses[i]);
                        graphics.DrawEllipse(mypen0, ButtonBoundses[i]);
                        graphics.DrawString(strings[i], myfont0, mybrush_selected, TextBoundses[i]);
                    }
                }
            }

            public override GH_ObjectResponse RespondToMouseMove(GH_Canvas sender, GH_CanvasMouseEvent e)
            {
                ICon_2 comp = Owner as ICon_2;
                for (int i = 0; i < 6; i++)
                {
                    if (((RectangleF)ButtonBoundses[i]).Contains(e.CanvasLocation)
                        && !comp.mouseHover[i])
                    {
                        comp.mouseHover[i] = true;
                        Owner.ExpireSolution(true);
                        return GH_ObjectResponse.Handled;
                    }
                    else if (!((RectangleF)ButtonBoundses[i]).Contains(e.CanvasLocation)
                        && comp.mouseHover[i])
                    {
                        comp.mouseHover[i] = false;
                        Owner.ExpireSolution(true);
                        return GH_ObjectResponse.Handled;
                    }
                }

                return base.RespondToMouseMove(sender, e);
            }
            public override GH_ObjectResponse RespondToMouseDown(GH_Canvas sender, GH_CanvasMouseEvent e)
            {

                ICon_2 comp = Owner as ICon_2;
                if (e.Button == System.Windows.Forms.MouseButtons.Left)
                {

                    if (((RectangleF)ButtonBoundses[0]).Contains(e.CanvasLocation))
                    {
                        comp.RecordUndoEvent(logText_X);
                        if (comp.Toggle_X) comp.Toggle_X = false; else comp.Toggle_X = true;
                        Owner.ExpireSolution(true);
                        return GH_ObjectResponse.Handled;
                    }


                    if (((RectangleF)ButtonBoundses[1]).Contains(e.CanvasLocation))
                    {
                        comp.RecordUndoEvent(logText_Y);
                        if (comp.Toggle_Y) comp.Toggle_Y = false; else comp.Toggle_Y = true;
                        Owner.ExpireSolution(true);
                        return GH_ObjectResponse.Handled;
                    }


                    if (((RectangleF)ButtonBoundses[2]).Contains(e.CanvasLocation))
                    {
                        comp.RecordUndoEvent(logText_Z);
                        if (comp.Toggle_Z) comp.Toggle_Z = false; else comp.Toggle_Z = true;
                        Owner.ExpireSolution(true);
                        return GH_ObjectResponse.Handled;
                    }


                    if (((RectangleF)ButtonBoundses[3]).Contains(e.CanvasLocation))
                    {
                        comp.RecordUndoEvent(logText_RX);
                        if (comp.Toggle_RX) comp.Toggle_RX = false; else comp.Toggle_RX = true;
                        Owner.ExpireSolution(true);
                        return GH_ObjectResponse.Handled;
                    }


                    if (((RectangleF)ButtonBoundses[4]).Contains(e.CanvasLocation))
                    {
                        comp.RecordUndoEvent(logText_RY);
                        if (comp.Toggle_RY) comp.Toggle_RY = false; else comp.Toggle_RY = true;
                        Owner.ExpireSolution(true);
                        return GH_ObjectResponse.Handled;
                    }


                    if (((RectangleF)ButtonBoundses[5]).Contains(e.CanvasLocation))
                    {
                        comp.RecordUndoEvent(logText_RZ);
                        if (comp.Toggle_RZ) comp.Toggle_RZ = false; else comp.Toggle_RZ = true;
                        Owner.ExpireSolution(true);
                        return GH_ObjectResponse.Handled;
                    }
                }

                return base.RespondToMouseDown(sender, e);
            }
        }
    }
}