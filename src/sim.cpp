#include "ap1/pnc_sim/sim.hpp"

#include "ap1_msgs/msg/lane_boundaries.hpp"
#include "ap1_msgs/msg/entity_state.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "ap1/pnc_sim/car.hpp"
#include "ap1_msgs/msg/entity_state_array.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

using geometry_msgs::msg::Point;
using ap1_msgs::msg::LaneBoundaries;
using ap1_msgs::msg::EntityState;
using ap1_msgs::msg::EntityStateArray;

ap1::sim::Sim::Sim(
    const EntityStateArray &entities, 
    const LaneBoundaries &lane, 
    const EntityState &car_state
):  car(car_state.x, car_state.y, car_state.z, car_state.gamma), 
    entities(entities), lane(lane) {}

void ap1::sim::Sim::update(
    const double dt, 
    const float steer_angle, // rads
    const float throttle, const float brake // [0, 1]
) {
    // # Longitudinal force
    // calculate max engine force possible
    float max_engine_force_n = this->car.peak_motor_power_w / std::max(car.speed_mps, EPSILON);
    if (std::isnan(max_engine_force_n)) {
        throw std::runtime_error("MAX ENGINE FORCE IS NAN");
    }

    // calculate the force cmd
    float force_cmd_n = throttle * max_engine_force_n;

    // limit drive by traction
    float drive_force_n = std::min(force_cmd_n, car.max_traction_n);

    // add brake force
    float brake_force_n = brake * car.max_braking_n;

    // total longitudinal force (brake + engine)
    float total_force_lon_n = drive_force_n - brake_force_n;

    // clamp to traction limit
    total_force_lon_n = std::clamp(total_force_lon_n, -car.max_traction_n, car.max_traction_n); 

    // # Calc Acceleration
    float acc_ms2 = total_force_lon_n / car.mass_kg;

    // # Update Speed
    car.speed_mps += acc_ms2 * dt;

    // # Steering
    // yaw rate
    float theta_per_second = (car.speed_mps / car.wheelbase_m) * tan(steer_angle);

    // update yaw
    car.yaw += theta_per_second * dt;

    // # Calculate Displacement components and total
    float dx = car.speed_mps * cos(car.yaw) * dt;
    float dy = car.speed_mps * sin(car.yaw) * dt;
    float ds = std::sqrt(dx*dx + dy*dy);

    // # Update data
    car.x += dx;
    car.y += dy;
    car.distance_covered += ds;
}