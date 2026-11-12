cmake_minimum_required(VERSION 3.20)
project(landing_sim VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)

add_executable(landing_sim sim/main.cpp)
target_include_directories(landing_sim PRIVATE sim)

enable_testing()

function(add_unit_test name)
    add_executable(${name} tests/${name}.cpp)
    target_include_directories(${name} PRIVATE sim core tests)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_unit_test(test_logger)
add_unit_test(test_vec3)
add_unit_test(test_quat)