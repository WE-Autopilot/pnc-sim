/**
 * Created: Nov. 9, 2025
 * Author(s): Obaid
 *
 * Publishes:
 * - the world's lanes and entities relative to the car,
 * - Car speed
 * 
 * Subscribes to:
 * - Target motor power (from Control)
 * - Target turn angle (from Control)
 */

#ifndef AP1_SIM_NODE_HPP
#define AP1_SIM_NODE_HPP

#include <cmath>
#include <rclcpp/node_options.hpp>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"

#include "ap1/pnc_sim/sim.hpp"
#include "ap1_msgs/msg/entity_state.hpp"
#include "ap1_msgs/msg/float_stamped.hpp"
#include "ap1_msgs/msg/entity_state_array.hpp"
#include "ap1_msgs/msg/lane_boundaries.hpp"

using ap1_msgs::msg::EntityState;
using ap1_msgs::msg::FloatStamped;
using ap1_msgs::msg::EntityStateArray;
using ap1_msgs::msg::LaneBoundaries;

namespace ap1::sim {
class SimNode : public rclcpp::Node {
public:
  // latest mem
  float last_throttle_cmd = 0.f;
  float last_brake_cmd = 0.f;
  float last_steer_cmd_rad = 0.f;

  // Default Constructor
  SimNode(const ap1::sim::Sim &sim);

  /**
   * @brief Publish the sim's details to the output topics.
   * Transforms the absolute coordinate space of the sim into relative car coords.
   */
  void publish();

private:
  const ap1::sim::Sim &sim;
  // Callbacks
  void on_throttle(const FloatStamped::SharedPtr msg);
  void on_steer(const FloatStamped::SharedPtr msg);
  void on_brake(const FloatStamped::SharedPtr msg);
  
  // ROS subs/pubs
  rclcpp::Subscription<FloatStamped>::SharedPtr throttle_sub_;
  rclcpp::Subscription<FloatStamped>::SharedPtr steer_sub_;
  rclcpp::Subscription<FloatStamped>::SharedPtr brake_sub_;

  rclcpp::Publisher<FloatStamped>::SharedPtr speed_pub_;
  rclcpp::Publisher<EntityStateArray>::SharedPtr entities_pub_;
  rclcpp::Publisher<LaneBoundaries>::SharedPtr lanes_pub_;
};

} // namespace ap1::sim

#endif
