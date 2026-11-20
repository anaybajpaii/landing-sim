#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>
#include <string>

#include "control/altitude_controller.hpp"
#include "control/attitude_controller.hpp"
#include "control/position_controller.hpp"
#include "logger/csv_logger.hpp"
#include "math/vec3.hpp"
#include "plant/platform.hpp"
#include "plant/quad.hpp"
#include "quad/quad_params.hpp"
#include "sensors/camera.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

void run_smoke(const std::string& out_path) {
    sim::CsvLogger log(out_path, {"t", "sine", "cosine"});
    const double dt = 0.01;
    for (int i = 0; i <= 200; ++i) {
        const double t = i * dt;
        log.row({t, std::sin(2.0 * kPi * t), std::cos(2.0 * kPi * t)});
    }
}

void run_roll_step(const std::string& out_path) {
    const QuadParams params;
    sim::Quad quad(params);
    core::AttitudeController controller;

    const double hover_omega = std::sqrt(params.mass * params.gravity / (4.0 * params.kThrust));
    quad.reset(sim::State{}, hover_omega);

    sim::CsvLogger log(out_path,
                       {"t", "roll_sp", "roll", "pitch", "yaw", "omega_x", "torque_x"});

    const double dt = 1e-3;
    const double step_time = 0.5;
    const double step_size = 0.2;
    const int steps = 4000;
    const int log_every = 10;

    for (int i = 0; i <= steps; ++i) {
        const double t = i * dt;
        const double roll_sp = (t >= step_time) ? step_size : 0.0;

        const sim::State& s = quad.state();
        const core::Vec3d torque =
            controller.update(s.attitude, s.omega, core::Vec3d{roll_sp, 0.0, 0.0}, dt);

        if (i % log_every == 0) {
            const core::Vec3d e = core::euler_zyx(s.attitude);
            log.row({t, roll_sp, e.x, e.y, e.z, s.omega.x, torque.x});
        }

        quad.step(params.mass * params.gravity, torque, dt);
    }
}

void run_altitude_step(const std::string& out_path) {
    const QuadParams params;
    sim::Quad quad(params);
    core::AttitudeController attitude;

    core::AltitudeConfig alt_cfg;
    alt_cfg.mass = params.mass;
    alt_cfg.gravity = params.gravity;
    alt_cfg.max_thrust = 4.0 * params.kThrust * params.omegaMax * params.omegaMax;
    core::AltitudeController altitude(alt_cfg);

    const double hover_omega = std::sqrt(params.mass * params.gravity / (4.0 * params.kThrust));
    quad.reset(sim::State{}, hover_omega);

    sim::CsvLogger log(out_path, {"t", "alt_sp", "alt", "climb_sp", "climb", "thrust"});

    const double dt = 1e-3;
    const int steps = 9000;
    const int log_every = 10;

    for (int i = 0; i <= steps; ++i) {
        const double t = i * dt;
        double alt_sp = 0.0;
        if (t >= 0.5) {
            alt_sp = 2.0;
        }
        if (t >= 5.0) {
            alt_sp = 1.0;
        }

        const sim::State& s = quad.state();
        const double thrust = altitude.update(alt_sp, s.position, s.velocity, s.attitude, dt);
        const core::Vec3d torque =
            attitude.update(s.attitude, s.omega, core::Vec3d{0.0, 0.0, 0.0}, dt);

        if (i % log_every == 0) {
            log.row({t, alt_sp, -s.position.z, altitude.climb_rate_setpoint(), -s.velocity.z,
                     thrust});
        }

        quad.step(thrust, torque, dt);
    }
}

void run_waypoint(const std::string& out_path) {
    const QuadParams params;
    sim::Quad quad(params);
    core::AttitudeController attitude;
    core::PositionController position;

    core::AltitudeConfig alt_cfg;
    alt_cfg.mass = params.mass;
    alt_cfg.gravity = params.gravity;
    alt_cfg.max_thrust = 4.0 * params.kThrust * params.omegaMax * params.omegaMax;
    core::AltitudeController altitude(alt_cfg);

    const double hover_omega = std::sqrt(params.mass * params.gravity / (4.0 * params.kThrust));
    sim::State start;
    start.position.z = -1.0;
    quad.reset(start, hover_omega);

    sim::CsvLogger log(out_path,
                       {"t", "x_sp", "x", "y_sp", "y", "alt", "roll", "pitch", "speed"});

    const double dt = 1e-3;
    const int steps = 17000;
    const int log_every = 10;

    for (int i = 0; i <= steps; ++i) {
        const double t = i * dt;
        double x_sp = 0.0;
        double y_sp = 0.0;
        if (t >= 1.0 && t < 9.0) {
            x_sp = 4.0;
            y_sp = 2.0;
        }

        const sim::State& s = quad.state();
        const double thrust = altitude.update(1.0, s.position, s.velocity, s.attitude, dt);
        const core::Vec3d tilt = position.update(core::Vec3d{x_sp, y_sp, 0.0}, core::Vec3d{},
                                                 s.position, s.velocity, s.attitude, dt);
        const core::Vec3d torque =
            attitude.update(s.attitude, s.omega, core::Vec3d{tilt.x, tilt.y, 0.0}, dt);

        if (i % log_every == 0) {
            const core::Vec3d e = core::euler_zyx(s.attitude);
            log.row({t, x_sp, s.position.x, y_sp, s.position.y, -s.position.z, e.x, e.y,
                     std::hypot(s.velocity.x, s.velocity.y)});
        }

        quad.step(thrust, torque, dt);
    }
}

void run_platform_track(const std::string& out_path) {
    const QuadParams params;
    sim::Quad quad(params);
    core::AttitudeController attitude;
    core::PositionController position;

    core::AltitudeConfig alt_cfg;
    alt_cfg.mass = params.mass;
    alt_cfg.gravity = params.gravity;
    alt_cfg.max_thrust = 4.0 * params.kThrust * params.omegaMax * params.omegaMax;
    core::AltitudeController altitude(alt_cfg);

    sim::Platform platform;
    platform.position = core::Vec3d{3.0, 0.0, 0.0};
    platform.velocity = core::Vec3d{0.5, 0.0, 0.0};

    const double hover_omega = std::sqrt(params.mass * params.gravity / (4.0 * params.kThrust));
    sim::State start;
    start.position.z = -1.0;
    quad.reset(start, hover_omega);

    sim::CsvLogger log(out_path,
                       {"t", "plat_x", "x", "plat_y", "y", "err_x", "err_y", "alt"});

    const double dt = 1e-3;
    const int steps = 20000;
    const int log_every = 10;

    for (int i = 0; i <= steps; ++i) {
        const double t = i * dt;

        const sim::State& s = quad.state();
        const double thrust = altitude.update(1.0, s.position, s.velocity, s.attitude, dt);
        const core::Vec3d tilt = position.update(platform.position, platform.velocity,
                                                 s.position, s.velocity, s.attitude, dt);
        const core::Vec3d torque =
            attitude.update(s.attitude, s.omega, core::Vec3d{tilt.x, tilt.y, 0.0}, dt);

        if (i % log_every == 0) {
            log.row({t, platform.position.x, s.position.x, platform.position.y, s.position.y,
                     platform.position.x - s.position.x, platform.position.y - s.position.y,
                     -s.position.z});
        }

        quad.step(thrust, torque, dt);
        platform.step(dt);
    }
}

void run_camera_demo(const std::string& out_path) {
    const QuadParams params;
    sim::Quad quad(params);
    core::AttitudeController attitude;
    core::PositionController position;

    core::AltitudeConfig alt_cfg;
    alt_cfg.mass = params.mass;
    alt_cfg.gravity = params.gravity;
    alt_cfg.max_thrust = 4.0 * params.kThrust * params.omegaMax * params.omegaMax;
    core::AltitudeController altitude(alt_cfg);

    sim::Platform platform;
    platform.position = core::Vec3d{3.0, 0.0, 0.0};
    platform.velocity = core::Vec3d{0.5, 0.0, 0.0};

    sim::Camera camera;
    core::Vec3d held{};
    double held_time = 0.0;

    const double hover_omega = std::sqrt(params.mass * params.gravity / (4.0 * params.kThrust));
    sim::State start;
    start.position.z = -1.0;
    quad.reset(start, hover_omega);

    sim::CsvLogger log(out_path, {"t", "err_x_true", "err_x_meas", "age", "got"});

    const double dt = 1e-3;
    const int steps = 12000;
    const int log_every = 5;

    for (int i = 0; i <= steps; ++i) {
        const double t = i * dt;
        const sim::State& s = quad.state();

        const core::Vec3d true_rel{platform.position.x - s.position.x,
                                   platform.position.y - s.position.y, 0.0};
        camera.capture(t, true_rel, -s.position.z);
        double got = 0.0;
        sim::CameraMeasurement m;
        while (camera.poll(t, m)) {
            held = m.rel_pos;
            held_time = m.capture_time;
            got = 1.0;
        }

        const double thrust = altitude.update(1.0, s.position, s.velocity, s.attitude, dt);
        const core::Vec3d tilt = position.update(platform.position, platform.velocity,
                                                 s.position, s.velocity, s.attitude, dt);
        const core::Vec3d torque =
            attitude.update(s.attitude, s.omega, core::Vec3d{tilt.x, tilt.y, 0.0}, dt);

        if (i % log_every == 0) {
            log.row({t, true_rel.x, held.x, t - held_time, got});
        }

        quad.step(thrust, torque, dt);
        platform.step(dt);
    }
}

}

int main(int argc, char** argv) {
    std::string scenario = "smoke";
    std::string out_path;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--scenario") == 0 && i + 1 < argc) {
            scenario = argv[++i];
        } else if (std::strcmp(argv[i], "--out") == 0 && i + 1 < argc) {
            out_path = argv[++i];
        } else {
            std::fprintf(stderr,
                         "usage: landing_sim [--scenario "
                         "smoke|roll_step|altitude_step|waypoint|platform_track|camera_demo] "
                         "[--out path.csv]\n");
            return 1;
        }
    }

    if (out_path.empty()) {
        out_path = "logs/" + scenario + ".csv";
    }

    try {
        if (scenario == "smoke") {
            run_smoke(out_path);
        } else if (scenario == "roll_step") {
            run_roll_step(out_path);
        } else if (scenario == "altitude_step") {
            run_altitude_step(out_path);
        } else if (scenario == "waypoint") {
            run_waypoint(out_path);
        } else if (scenario == "platform_track") {
            run_platform_track(out_path);
        } else if (scenario == "camera_demo") {
            run_camera_demo(out_path);
        } else {
            std::fprintf(stderr, "unknown scenario: %s\n", scenario.c_str());
            return 1;
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }

    std::printf("wrote %s\n", out_path.c_str());
    return 0;
}