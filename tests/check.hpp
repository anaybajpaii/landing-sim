#pragma once

#include <cmath>
#include <cstdio>

namespace check {

inline int& total() {
    static int n = 0;
    return n;
}

inline int& failures() {
    static int n = 0;
    return n;
}

inline int report() {
    std::printf("%d checks, %d failed\n", total(), failures());
    return failures() > 0 ? 1 : 0;
}

}

#define CHECK(cond)                                                                  \
    do {                                                                             \
        ++check::total();                                                            \
        if (!(cond)) {                                                               \
            ++check::failures();                                                     \
            std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
        }                                                                            \
    } while (0)

#define CHECK_NEAR(actual, expected, tol)                                            \
    do {                                                                             \
        ++check::total();                                                            \
        const double check_actual = (actual);                                        \
        const double check_expected = (expected);                                    \
        if (!(std::fabs(check_actual - check_expected) <= (tol))) {                  \
            ++check::failures();                                                     \
            std::fprintf(stderr, "%s:%d: CHECK_NEAR failed: %s = %.12g, expected %.12g\n", \
                         __FILE__, __LINE__, #actual, check_actual, check_expected); \
        }                                                                            \
    } while (0)