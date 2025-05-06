extern "C" {
    __declspec(dllexport) double ComputeY(double x) {
        return x * x + 1.0;
    }
}