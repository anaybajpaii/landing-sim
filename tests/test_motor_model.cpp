#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>

#include "quad/motor_model.hpp"

static int failures = 0;

static void check(bool ok, const char* name) {
    if (!ok) {
        std::printf("FAIL: %s\n", name);
        ++failures;
    }
}

static bool approx(double a, double b, double tol) {
    return std::fabs(a - b) <= tol;
}

int main() {
    QuadParams p;

    {
        MotorModel m(p);
        check(m.omega(0) == 0.0 && m.thrust(0) == 0.0 && m.torque(0) == 0.0, "starts at rest");
    }

    {
        MotorModel m(p);
        m.reset(p.omegaMax);
        check(approx(m.thrust(0), 8.0, 1e-9), "max thrust is 8 N");
        check(approx(m.torque(0), 0.128, 1e-12), "max torque is 0.128 N m");
    }

    {
        const double wHover = std::sqrt(p.mass * p.gravity / (4.0 * p.kThrust));
        MotorModel m(p);
        std::array<double, 4> cmd{wHover, wHover, wHover, wHover};
        for (int k = 0; k < 1000; ++k) m.step(cmd, 0.001);
        double total = 0.0;
        for (std::size_t i = 0; i < 4; ++i) total += m.thrust(i);
        check(approx(wHover, 553.7, 0.1), "hover speed about 553.7 rad/s");
        check(approx(total, p.mass * p.gravity, 1e-6), "hover thrust equals weight");
    }

    {
        MotorModel m(p);
        std::array<double, 4> cmd{400.0, 400.0, 400.0, 400.0};
        for (int k = 0; k < 30; ++k) m.step(cmd, 0.001);
        const double expected = 400.0 * (1.0 - std::exp(-1.0));
        check(approx(m.omega(0), expected, 1e-9), "first-order response at t = tau");
    }

    {
        MotorModel m(p);
        std::array<double, 4> cmd{5000.0, -10.0, 500.0, 500.0};
        for (int k = 0; k < 2000; ++k) m.step(cmd, 0.001);
        check(approx(m.omega(0), p.omegaMax, 1e-6), "command above max clamps to max");
        check(approx(m.omega(1), 0.0, 1e-9), "negative command clamps to zero");
        check(approx(m.omega(2), 500.0, 1e-6), "in-range command reached");
    }

    {
        MotorModel m(p);
        m.reset(300.0);
        check(approx(m.omega(3), 300.0, 1e-12), "reset sets all motors");
    }

    if (failures == 0) std::printf("all motor model tests passed\n");
    return failures == 0 ? 0 : 1;
}