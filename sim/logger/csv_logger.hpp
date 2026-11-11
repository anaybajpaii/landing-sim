#pragma once

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <vector>

namespace sim {

class CsvLogger {
public:
    CsvLogger(const std::string& path, const std::vector<std::string>& columns)
        : column_count_(columns.size()) {
        const std::filesystem::path file_path(path);
        if (file_path.has_parent_path()) {
            std::filesystem::create_directories(file_path.parent_path());
        }
        out_.open(path);
        if (!out_) {
            throw std::runtime_error("cannot open log file: " + path);
        }
        out_.precision(10);
        for (std::size_t i = 0; i < columns.size(); ++i) {
            if (i > 0) {
                out_ << ',';
            }
            out_ << columns[i];
        }
        out_ << '\n';
    }

    void row(std::initializer_list<double> values) {
        if (values.size() != column_count_) {
            throw std::runtime_error("row size does not match column count");
        }
        bool first = true;
        for (const double v : values) {
            if (!first) {
                out_ << ',';
            }
            out_ << v;
            first = false;
        }
        out_ << '\n';
    }

private:
    std::ofstream out_;
    std::size_t column_count_;
};

}