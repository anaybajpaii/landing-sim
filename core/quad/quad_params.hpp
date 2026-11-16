#pragma once

struct QuadParams {
    double mass = 1.0;
    double armLength = 0.2;
    double ixx = 0.01;
    double iyy = 0.01;
    double izz = 0.02;
    double kThrust = 8.0e-6;
    double kTorque = 1.28e-7;
    double omegaMax = 1000.0;
    double motorTau = 0.03;
    double gravity = 9.81;
};