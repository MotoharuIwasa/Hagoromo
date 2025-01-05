using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using Cloo;
using System.IO;

namespace Hagoromo
{
    public class Add_GPU : GH_Component
    {
        public Add_GPU()
          : base("G-Add", "Add_GPU",
            "Adds two numbers using GPU",
            "Hagoromo", "test")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddNumberParameter("A", "a", "first number", GH_ParamAccess.item);
            pManager.AddNumberParameter("B", "b", "second number", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("C", "c", "Result", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            double a = 0.0;
            double b = 0.0;
            DA.GetData("A", ref a);
            DA.GetData("B", ref b);

            // Initialize OpenCL
            ComputePlatform platform = ComputePlatform.Platforms[0];
            ComputeContext context = new ComputeContext(
                ComputeDeviceTypes.Gpu,
                new ComputeContextPropertyList(platform),
                null, IntPtr.Zero
            );

            // Create command queue
            ComputeCommandQueue queue = new ComputeCommandQueue(context, context.Devices[0], ComputeCommandQueueFlags.None);

            // OpenCL kernel source
            string kernelSource = @"
                __kernel void Add(__global double* a, __global double* b, __global double* result)
                {
                    int i = get_global_id(0);
                    result[i] = a[i] + b[i];
                }
            ";

            // Compile the kernel
            ComputeProgram program = new ComputeProgram(context, kernelSource);
            program.Build(null, null, null, IntPtr.Zero);
            ComputeKernel kernel = program.CreateKernel("Add");

            // Create buffers
            ComputeBuffer<double> bufferA = new ComputeBuffer<double>(context, ComputeMemoryFlags.ReadOnly | ComputeMemoryFlags.CopyHostPointer, new double[] { a });
            ComputeBuffer<double> bufferB = new ComputeBuffer<double>(context, ComputeMemoryFlags.ReadOnly | ComputeMemoryFlags.CopyHostPointer, new double[] { b });
            ComputeBuffer<double> bufferResult = new ComputeBuffer<double>(context, ComputeMemoryFlags.WriteOnly, 1);

            // Set kernel arguments
            kernel.SetMemoryArgument(0, bufferA);
            kernel.SetMemoryArgument(1, bufferB);
            kernel.SetMemoryArgument(2, bufferResult);

            // Execute the kernel
            queue.Execute(kernel, null, new long[] { 1 }, null, null);

            // Read back the result
            double[] result = new double[1];
            queue.ReadFromBuffer(bufferResult, ref result, true, null);

            // Set output
            DA.SetData("C", result[0]);

            // Clean up
            queue.Dispose();
            bufferA.Dispose();
            bufferB.Dispose();
            bufferResult.Dispose();
            kernel.Dispose();
            program.Dispose();
            context.Dispose();
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid
        {
            get { return new Guid("0092AFEB-35DD-4CAD-BD9D-28C8A4A88FDC"); }
        }
    }
}