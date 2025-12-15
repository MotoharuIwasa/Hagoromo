using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;

using Rhino.Geometry;
using System;
using System.Collections.Generic;

using Grasshopper.Kernel.Types;

namespace Hagoromo.GeometryTools
{
    public class CheckMesh : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public CheckMesh()
          : base("CheckMesh", "CheckMesh",
              "CheckMesh",
              "Hagoromo", "Geometry")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Triangulated Mesh", "M", "Mesh", GH_ParamAccess.item);
            pManager.AddIntegerParameter("vertex index", "v", "v", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddIntegerParameter("faces", "f", "faces", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Rhino.Geometry.Mesh mesh = new Rhino.Geometry.Mesh();
            int v = 0;
            DA.GetData(0, ref mesh);
            DA.GetData(1, ref v);
            List<int> faces = MeshDataTools.GetOrderedFacesAroundVertex(mesh, mesh.TopologyVertices.TopologyVertexIndex(v));
            DA.SetDataList(0,faces);
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

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("5408B3A5-8B9F-4352-A205-CDA74E4B5602"); }
        }
    }
}