/**
 * Created: Jan 22, 2026
 * Author(s): Aly Ashour
 *
 * This sim simulates a world around a car.
 * The world contains some kind of road made up of nodes forming it's boundaries and a list of entities of various types.
 *
 * Internal coordinate system is cartesian and absolute. Output coordinate system is relative to the car (pos and rotation).
 */

#ifndef AP1_SIM_HPP
#define AP1_SIM_HPP

#include "ap1_msgs/msg/lane_boundaries.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "ap1/pnc_sim/car.hpp"
#include "ap1_msgs/msg/entity_state_array.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

#define EPSILON 0.1f

using geometry_msgs::msg::Point;
using ap1_msgs::msg::LaneBoundaries;
using ap1_msgs::msg::EntityStateArray;

namespace ap1::sim
{
class Sim
{
  public:
    Car car = Car();
    EntityStateArray entities;
    LaneBoundaries lane;

    Sim(const EntityStateArray &entities, const LaneBoundaries &lane): entities(entities), lane(lane) {}

    /**
     * @brief Update the sim's details based on some car details.
     * 
     * @param dt The simulation timestep. Should be as high as possible.
     * @param steer_angle The steering wheel's turn angle. Assumes the steering is ideal.
     * @param throttle Throttle [0, 1]. The car CANNOT reverse yet.
     * @param brake Break [0, 1]. 
     */
    void update(
      const double dt, 
      const float steer_angle, // rads
      const float throttle, const float brake // [0, 1]
    ) {
      // # Longitudinal force
      // calculate max engine force possible
      float max_engine_force_n = this->car.peak_motor_power_w / std::max(car.speed_mps, EPSILON);

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

      // # Update Position
      car.x += car.speed_mps * cos(car.yaw) * dt;
      car.y += car.speed_mps * sin(car.yaw) * dt;
    }
};
} // namespace ap1::sim

#endif // AP1_SIM_HPP
