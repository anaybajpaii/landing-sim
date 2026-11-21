# Log
## 1. Problem: integrator dependencies were undefined

**Date:** Nov 13

**Result:** The integrator implementation failed to compile because `State`, `StateDerivative`, `Wrench`, `VehicleParams` and `derivative()` were undefined.

**Attempt:** Traced the missing declarations back to their respective headers and checked whether the required types and functions had actually been implemented.

**Fix:** Implemented the missing definitions in `rigid_body.hpp` and `vehicle_params.hpp`. The issue was not with the integrator itself; its dependencies were still empty.

---

## 2. Problem: quaternion API names did not match the tests

**Date:** Nov 12

**Result:** `test_attitude.cpp` reported that `core::euler_zyx` and `core::from_euler_zyx` did not exist.

**Attempt:** Checked the quaternion header and compared the function names exposed by the implementation with those expected by the tests.

**Fix:** Renamed `to_euler` to `euler_zyx` and `from_euler` to `from_euler_zyx` in `core/math/quat.hpp`. The implementation was already present; the problem was an API naming mismatch.

---

## 3. Problem: RK4 tests were not catching incorrect integration methods

**Date:** Nov 13

**Result:** The initial integrator tests passed even when the velocity integration was replaced with Euler integration. Constant acceleration made the incorrect implementation difficult to distinguish from RK4.

**Attempt:** Tested several deliberately incorrect implementations, including Euler integration, midpoint integration, equal RK4 weights and an incorrect fourth stage. Investigated why the existing tests failed to detect them.

**Fix:** Added a rotating-thrust test with an analytical reference solution. This introduced changing acceleration and exposed errors that the constant-acceleration tests missed. Also checked quaternion normalisation after integration.

---

## 4. Problem: mixer signs wrong for the vehicle's frame

**Date:** Nov 16

**Result:** The mixer's tests passed, but its pitch and yaw signs assumed a z-up body frame while the rigid body uses z-down (gravity along +z, hover thrust along body -z).

**Attempt:** Read `test_integrator` to check which direction gravity and thrust pointed.

**Fix:** Flipped the pitch and yaw signs and rewrote the mixer tests so the roll, pitch and yaw checks in `test_quad` ran against the actual rigid-body dynamics.

---

## 5. Problem: platform test failed because the probe sat on the zone boundary

**Date:** Nov 20

**Result:** `test_platform` failed on `test_zone_follows_the_platform` (`p.contains_xy(probe)` at line 55). The other 13 tests passed.

**Attempt:** Worked through the numbers in the test. The platform moves at 0.5 m/s for 3000 steps of 1 ms, so it ends near x = 1.5 m. The probe is at x = 2.0 and the zone half-size is 0.5, so the probe was exactly on the zone edge. After 3000 floating-point additions the platform position is not exactly 1.5, so the check could land on either side of the boundary.

**Fix:** Changed the test to run 3500 steps. The platform ends near x = 1.75 m, so the probe is 0.25 m inside the zone and the result no longer depends on rounding. The platform code was correct; the test was written too tight.

## 6. Problem: new scenario missing because main.cpp was still the old version

**Date:** Nov 21

**Result:** Plotting `logs\track_descend.csv` failed with `FileNotFoundError`, and `cmake --build build` printed `ninja: no work to do`.

**Attempt:** Checked which of the commit 24 changes were actually on disk. `CMakeLists.txt`, `tracking_guidance.hpp` and `test_tracking_guidance.cpp` were all present, but searching `sim/main.cpp` for `track_descend` found nothing.

**Fix:** `main.cpp` was still the commit 23 version, so the executable had no `track_descend` scenario and wrote no CSV. Replaced `main.cpp` with the full updated file, after which it compiled and the scenario produced its plot. When the build says there's no work to do right after an edit, the first thing to check is that the edit is on disk.