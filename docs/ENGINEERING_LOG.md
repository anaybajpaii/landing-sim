# Engineering log

## 2026-11-16
Built the motor model, mixer and Quad plant. The mixer first assumed a z-up body frame, but the rigid body is z-down (gravity along +z, thrust along body -z). Found it by reading the integrator tests. Flipped the pitch and yaw signs and rewrote the mixer tests. The roll, pitch and yaw checks in test_quad now run against the real dynamics.

## 2026-11-19
Attitude controller (angle loop into rate loop). Roll step of 0.2 rad settles about 0.7 s after the step with no visible overshoot. Slower than my hand estimate, so the loop is more damped than I calculated. Pitch and yaw stay at floating-point noise, so the axes are decoupled. Altitude controller: climb rate only peaks near 2.2 m/s even though the cap is 3 m/s, because the climb-rate setpoint collapses as the altitude error shrinks. Left it as is; alt_kp is the knob if I want faster climbs.

## 2026-11-20
Position controller (position into velocity into tilt). [Add what the waypoint test and plot showed, and anything you changed.]