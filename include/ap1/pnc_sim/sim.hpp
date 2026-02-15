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
#include "ap1_msgs/msg/entity_state.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "ap1/pnc_sim/car.hpp"
#include "ap1_msgs/msg/entity_state_array.hpp"
#include <cmath>

#define EPSILON 0.1f

using geometry_msgs::msg::Point;
using ap1_msgs::msg::LaneBoundaries;
using ap1_msgs::msg::EntityState;
using ap1_msgs::msg::EntityStateArray;

namespace ap1::sim
{
class Sim
{
  public:
    Car car = Car();
    EntityStateArray entities;
    LaneBoundaries lane;

    Sim(
      const EntityStateArray &entities,
      const LaneBoundaries &lane,
      const EntityState &car_state
    );

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
    );
};
} // namespace ap1::sim

#endif // AP1_SIM_HPP
