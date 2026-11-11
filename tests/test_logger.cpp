#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "check.hpp"
#include "logger/csv_logger.hpp"

namespace {

std::vector<std::string> read_lines(const std::string& path) {
    std::ifstream in(path);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(in, line)) {
        lines.push_back(line);
    }
    return lines;
}

std::vector<double> parse_row(const std::string& line) {
    std::vector<double> values;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
        values.push_back(std::stod(cell));
    }
    return values;
}

}

int main() {
    const std::filesystem::path dir = std::filesystem::temp_directory_path() / "landing_sim_test_logger";
    const std::string path = (dir / "log.csv").string();

    {
        sim::CsvLogger log(path, {"t", "x_m"});
        log.row({0.0, 1.5});
        log.row({0.01, -2.25});
    }

    const std::vector<std::string> lines = read_lines(path);
    CHECK(lines.size() == 3);
    if (lines.size() == 3) {
        CHECK(lines[0] == "t,x_m");
        const std::vector<double> r1 = parse_row(lines[1]);
        const std::vector<double> r2 = parse_row(lines[2]);
        CHECK(r1.size() == 2);
        CHECK(r2.size() == 2);
        if (r1.size() == 2 && r2.size() == 2) {
            CHECK_NEAR(r1[0], 0.0, 1e-12);
            CHECK_NEAR(r1[1], 1.5, 1e-12);
            CHECK_NEAR(r2[0], 0.01, 1e-12);
            CHECK_NEAR(r2[1], -2.25, 1e-12);
        }
    }

    bool threw = false;
    try {
        sim::CsvLogger log(path, {"t", "x_m"});
        log.row({1.0});
    } catch (const std::runtime_error&) {
        threw = true;
    }
    CHECK(threw);

    std::filesystem::remove_all(dir);
    return check::report();
}