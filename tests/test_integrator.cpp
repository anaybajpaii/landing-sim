#include <cmath>

#include "check.hpp"
#include "math/quat.hpp"
#include "math/vec3.hpp"
#include "plant/integrator.hpp"
#include "plant/rigid_body.hpp"
#include "plant/vehicle_params.hpp"

using core::Quatd;
using core::Vec3d;
using sim::State;
using sim::VehicleParams;
using sim::Wrench;

namespace {

VehicleParams test_params() {
    VehicleParams p;
    p.mass_kg = 2.0;
    p.inertia_kgm2 = Vec3d{0.01, 0.02, 0.03};
    p.gravity_mps2 = 10.0;
    return p;
}

State run(State s, const Wrench& w, const VehicleParams& p, double dt, int steps) {
    for (int i = 0; i < steps; ++i) {
        s = sim::rk4_step(s, w, p, dt);
    }
    return s;
}

double kinetic_energy(const State& s, const VehicleParams& p) {
    const Vec3d& J = p.inertia_kgm2;
    return 0.5 * (J.x * s.omega.x * s.omega.x + J.y * s.omega.y * s.omega.y + J.z * s.omega.z * s.omega.z);
}

Vec3d angular_momentum_world(const State& s, const VehicleParams& p) {
    const Vec3d& J = p.inertia_kgm2;
    const Vec3d h_body{J.x * s.omega.x, J.y * s.omega.y, J.z * s.omega.z};
    return core::rotate(s.attitude, h_body);
}

void check_vec(const Vec3d& v, double x, double y, double z, double tol) {
    CHECK_NEAR(v.x, x, tol);
    CHECK_NEAR(v.y, y, tol);
    CHECK_NEAR(v.z, z, tol);
}

void test_free_fall_matches_analytic() {
    State s;
    s.velocity = Vec3d{3.0, -1.0, -2.0};
    const State r = run(s, Wrench{}, test_params(), 1e-3, 1000);
    check_vec(r.position, 3.0, -1.0, -2.0 + 5.0, 1e-9);
    check_vec(r.velocity, 3.0, -1.0, 8.0, 1e-9);
}

void test_hover_stays_put() {
    const VehicleParams p = test_params();
    Wrench w;
    w.thrust_n = p.mass_kg * p.gravity_mps2;
    const State r = run(State{}, w, p, 1e-3, 10000);
    check_vec(r.position, 0.0, 0.0, 0.0, 1e-9);
    check_vec(r.velocity, 0.0, 0.0, 0.0, 1e-9);
    CHECK_NEAR(r.attitude.w, 1.0, 1e-12);
}

void test_tilted_hover_accelerates_sideways() {
    const VehicleParams p = test_params();
    const double tilt = 0.4;
    State s;
    s.attitude = core::from_axis_angle(Vec3d{1.0, 0.0, 0.0}, tilt);
    Wrench w;
    w.thrust_n = p.mass_kg * p.gravity_mps2 / std::cos(tilt);
    const State r = run(s, w, p, 1e-3, 1000);
    const double a = p.gravity_mps2 * std::tan(tilt);
    check_vec(r.velocity, 0.0, a, 0.0, 1e-9);
    check_vec(r.position, 0.0, 0.5 * a, 0.0, 1e-9);
}

void test_spinning_thrust_matches_analytic() {
    const VehicleParams p = test_params();
    const double spin = 2.0;
    const double thrust_accel = 12.0;
    const double t_end = 3.0;
    State s;
    s.omega = Vec3d{spin, 0.0, 0.0};
    Wrench w;
    w.thrust_n = thrust_accel * p.mass_kg;
    const State r = run(s, w, p, 1e-3, 3000);

    const double theta = spin * t_end;
    const double vy = thrust_accel * (1.0 - std::cos(theta)) / spin;
    const double vz = p.gravity_mps2 * t_end - thrust_accel * std::sin(theta) / spin;
    const double y = thrust_accel * (t_end - std::sin(theta) / spin) / spin;
    const double z = 0.5 * p.gravity_mps2 * t_end * t_end - thrust_accel * (1.0 - std::cos(theta)) / (spin * spin);
    check_vec(r.velocity, 0.0, vy, vz, 1e-8);
    check_vec(r.position, 0.0, y, z, 1e-8);
}

void test_constant_torque_spin_up() {
    const VehicleParams p = test_params();
    Wrench w;
    w.torque_nm = Vec3d{0.0, 0.0, 0.006};
    const State r = run(State{}, w, p, 1e-3, 2000);
    CHECK_NEAR(r.omega.z, 0.4, 1e-9);
    const Quatd expected = core::from_axis_angle(Vec3d{0.0, 0.0, 1.0}, 0.4);
    CHECK_NEAR(r.attitude.w, expected.w, 1e-9);
    CHECK_NEAR(r.attitude.x, expected.x, 1e-9);
    CHECK_NEAR(r.attitude.y, expected.y, 1e-9);
    CHECK_NEAR(r.attitude.z, expected.z, 1e-9);
}

void test_torque_free_conserves_energy_and_momentum() {
    const VehicleParams p = test_params();
    State s;
    s.attitude = core::normalized(Quatd{0.3, -0.5, 0.7, 0.2});
    s.omega = Vec3d{2.0, 1.0, 3.0};
    const double e0 = kinetic_energy(s, p);
    const Vec3d h0 = angular_momentum_world(s, p);

    const State r = run(s, Wrench{}, p, 1e-3, 10000);
    CHECK_NEAR(kinetic_energy(r, p) / e0, 1.0, 1e-8);
    const Vec3d h1 = angular_momentum_world(r, p);
    check_vec(h1, h0.x, h0.y, h0.z, 1e-8);
    CHECK(std::fabs(r.omega.x - 2.0) > 1e-3 || std::fabs(r.omega.y - 1.0) > 1e-3);
}

void test_quaternion_stays_unit() {
    const VehicleParams p = test_params();
    State s;
    s.omega = Vec3d{4.0, -3.0, 5.0};
    Wrench w;
    w.thrust_n = 25.0;
    w.torque_nm = Vec3d{0.01, -0.02, 0.005};
    const State r = run(s, w, p, 1e-3, 20000);
    CHECK_NEAR(core::norm(r.attitude), 1.0, 1e-12);
}

void test_fourth_order_convergence() {
    const VehicleParams p = test_params();
    State s;
    s.attitude = core::normalized(Quatd{0.3, -0.5, 0.7, 0.2});
    s.omega = Vec3d{6.0, 2.0, 9.0};

    const State ref = run(s, Wrench{}, p, 1e-4, 10000);
    const State coarse = run(s, Wrench{}, p, 0.05, 20);
    const State fine = run(s, Wrench{}, p, 0.025, 40);
    const double e_coarse = core::norm(coarse.omega - ref.omega);
    const double e_fine = core::norm(fine.omega - ref.omega);
    CHECK(e_fine > 1e-12);
    CHECK(e_coarse > 1e-9);
    const double ratio = e_coarse / e_fine;
    CHECK(ratio > 12.0);
    CHECK(ratio < 20.0);
}

}

int main() {
    test_free_fall_matches_analytic();
    test_hover_stays_put();
    test_tilted_hover_accelerates_sideways();
    test_spinning_thrust_matches_analytic();
    test_constant_torque_spin_up();
    test_torque_free_conserves_energy_and_momentum();
    test_quaternion_stays_unit();
    test_fourth_order_convergence();
    return check::report();
}