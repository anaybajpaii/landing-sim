#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>
#include <string>

#include "logger/csv_logger.hpp"

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

}

int main(int argc, char** argv) {
    std::string scenario = "smoke";
    std::string out_path = "logs/smoke.csv";

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--scenario") == 0 && i + 1 < argc) {
            scenario = argv[++i];
        } else if (std::strcmp(argv[i], "--out") == 0 && i + 1 < argc) {
            out_path = argv[++i];
        } else {
            std::fprintf(stderr, "usage: landing_sim [--scenario smoke] [--out path.csv]\n");
            return 1;
        }
    }

    try {
        if (scenario == "smoke") {
            run_smoke(out_path);
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