/**
 * Created: Feb 14, 2026
 * Author(s): Aly Ashour
 *
 * Assumes a car powered by an electric motor.
 * Simulates drag and traction but no air resistance or other info (collisions etc.)
 */

#ifndef AP1_SIM_CAR
#define AP1_SIM_CAR

#include "math.h"
#include <cmath>

// probably a good idea that car doesn't reasonably know it's "defaults"
// should be removed in favor of defaults being somewhere else.

// == DEFAULTS
// STATIC
constexpr float DFLT_MASS_KG = 1200.f;
constexpr float DFLT_WHEELBASE_M = 2.5f;

// DYNAMIC
constexpr float DFLT_PEAK_MTR_PWR_W = 90000.f; // 90 kW
constexpr float DFLT_MAX_TRACTION_N = 12000.f; // 12 kN
constexpr float DFLT_MAX_BRAKE_N = 11000.f; // 11 kN

namespace ap1::sim {
    class Car {
    public:
        // constants
        const float peak_motor_power_w = DFLT_PEAK_MTR_PWR_W;
        const float max_traction_n = DFLT_MAX_TRACTION_N;
        const float max_braking_n = DFLT_MAX_BRAKE_N;
        const float mass_kg = DFLT_MASS_KG;
        const float wheelbase_m = DFLT_WHEELBASE_M;

        // state
        float speed_mps = 0.f;

        // position
        float x = 0.f, y = 0.f, z = 0.f;
        float yaw = 0.f;
        
        // Default Constructor
        Car();

        // Alt constructor
        Car(const float x, const float y, const float z, const float yaw);
    };
} // namespace ap1::sim

#endif // AP1_SIM_CAR
