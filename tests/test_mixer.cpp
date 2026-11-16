#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>

#include "quad/mixer.hpp"
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
    Mixer mixer(p);
    const double weight = p.mass * p.gravity;
    const double yawRatio = p.kTorque / p.kThrust;

    {
        Wrench w;
        w.thrust = weight;
        const std::array<double, 4> t = mixer.thrusts(w);
        const std::array<double, 4> om = mixer.omegaCommands(w);
        const double wHover = std::sqrt(weight / (4.0 * p.kThrust));
        for (std::size_t i = 0; i < 4; ++i) {
            check(approx(t[i], weight / 4.0, 1e-12), "hover thrust split equally");
            check(approx(om[i], wHover, 1e-9), "hover omega command");
        }
    }

    {
        Wrench w;
        w.thrust = 12.0;
        w.tx = 0.05;
        w.ty = -0.03;
        w.tz = 0.01;
        const Wrench back = mixer.wrench(mixer.thrusts(w));
        check(approx(back.thrust, w.thrust, 1e-12), "round trip thrust");
        check(approx(back.tx, w.tx, 1e-12), "round trip roll torque");
        check(approx(back.ty, w.ty, 1e-12), "round trip pitch torque");
        check(approx(back.tz, w.tz, 1e-12), "round trip yaw torque");
    }

    {
        Wrench w;
        w.thrust = weight;
        w.tz = yawRatio;
        const std::array<double, 4> t = mixer.thrusts(w);
        check(approx(t[0], t[2], 1e-12), "yaw: CCW motors equal");
        check(approx(t[1], t[3], 1e-12), "yaw: CW motors equal");
        check(t[1] > t[0], "positive yaw torque speeds up CW motors");
        check(approx(t[0], 0.25 * (weight - 1.0), 1e-12), "yaw: CCW motor value");
        check(approx(t[1], 0.25 * (weight + 1.0), 1e-12), "yaw: CW motor value");
    }

    {
        Wrench w;
        w.thrust = weight;
        w.tx = 0.1;
        const std::array<double, 4> t = mixer.thrusts(w);
        check(t[1] > t[0] && t[2] > t[3], "positive roll torque speeds up left motors");
        check(approx(t[1], t[2], 1e-12) && approx(t[0], t[3], 1e-12), "roll symmetry");
    }

    {
        Wrench w;
        w.thrust = weight;
        w.ty = 0.1;
        const std::array<double, 4> t = mixer.thrusts(w);
        check(t[2] > t[0] && t[3] > t[1], "positive pitch torque speeds up rear motors");
        check(approx(t[2], t[3], 1e-12) && approx(t[0], t[1], 1e-12), "pitch symmetry");
    }

    {
        const double maxT = p.kThrust * p.omegaMax * p.omegaMax;
        Wrench hi;
        hi.thrust = 100.0;
        const std::array<double, 4> th = mixer.thrusts(hi);
        const std::array<double, 4> oh = mixer.omegaCommands(hi);
        Wrench lo;
        lo.thrust = -5.0;
        const std::array<double, 4> tl = mixer.thrusts(lo);
        const std::array<double, 4> ol = mixer.omegaCommands(lo);
        for (std::size_t i = 0; i < 4; ++i) {
            check(approx(th[i], maxT, 1e-12), "over-demand clamps to max thrust");
            check(approx(oh[i], p.omegaMax, 1e-9), "over-demand omega is omegaMax");
            check(tl[i] == 0.0, "negative demand clamps to zero thrust");
            check(ol[i] == 0.0 && !std::isnan(ol[i]), "zero thrust gives zero omega, no NaN");
        }
    }

    {
        Wrench w;
        w.thrust = 11.0;
        w.tx = 0.04;
        w.ty = 0.02;
        w.tz = -0.008;
        MotorModel motors(p);
        const std::array<double, 4> cmd = mixer.omegaCommands(w);
        for (int k = 0; k < 2000; ++k) motors.step(cmd, 0.001);
        std::array<double, 4> actual{};
        for (std::size_t i = 0; i < 4; ++i) actual[i] = motors.thrust(i);
        const Wrench got = mixer.wrench(actual);
        check(approx(got.thrust, w.thrust, 1e-6), "mixer + motors: thrust");
        check(approx(got.tx, w.tx, 1e-6), "mixer + motors: roll torque");
        check(approx(got.ty, w.ty, 1e-6), "mixer + motors: pitch torque");
        check(approx(got.tz, w.tz, 1e-6), "mixer + motors: yaw torque");
    }

    if (failures == 0) std::printf("all mixer tests passed\n");
    return failures == 0 ? 0 : 1;
}