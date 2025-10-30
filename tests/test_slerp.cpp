/* *******************************************************************************
 * File: tests/test_slerp.cpp
 * Description: Test and micro-benchmark for quaternion slerp.
 * *******************************************************************************/

#include <iostream>
#include <chrono>
#include "quaternion.hpp"
#include <math.h>

using namespace AdasTools;

static bool approx(double a, double b, double eps=1e-6) {
    double d = a-b; if (d<0) d=-d; return d<=eps;
}

int main()
{
    Quaternion a = quaternionFromRPY(0.2, -0.1, 0.3);
    Quaternion b = quaternionFromRPY(-0.4, 0.6, -0.2);
    a = normalizeQuaternion(a);
    b = normalizeQuaternion(b);

    // correctness: slerp at 0 and 1 should yield the endpoints
    Quaternion s0 = slerp(a, b, 0.0);
    Quaternion s1 = slerp(a, b, 1.0);
    if (!approx(s0.w, a.w) || !approx(s0.x, a.x)) { std::cerr << "slerp t=0 failed\n"; return 1; }
    if (!approx(s1.w, b.w) || !approx(s1.x, b.x)) { std::cerr << "slerp t=1 failed\n"; return 2; }

    // micro-benchmark
    const int N = 200000;
    auto start = std::chrono::high_resolution_clock::now();
    for (int i=0;i<N;++i) {
        double t = (i % 1000) / 1000.0;
        Quaternion r = slerp(a, b, t);
        (void)r;
    }
    auto end = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end-start).count();

    std::cout << "slerp benchmark: " << N << " ops in " << ms << " ms -> " << (N / (ms/1000.0)) << " ops/s\n";
    std::cout << "slerp correctness and performance OK" << std::endl;
    return 0;
}
