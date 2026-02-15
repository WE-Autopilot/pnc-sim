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
#include <stdexcept>
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

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
  SimNode(const ap1::sim::Sim &sim)
      : Node("sim_node"), sim(sim) {
    // Subscriptions
    throttle_sub_ = this->create_subscription<FloatStamped>(
      "/ap1/control/motor_power", 1,
      [this](const FloatStamped::SharedPtr f) {
        this->on_throttle(f);
      }
    );

    steer_sub_ = this->create_subscription<FloatStamped>(
      "/ap1/control/turn_angle", 1,
      [this](const FloatStamped::SharedPtr f) {
        this->on_steer(f);
      }
    );

    brake_sub_ = this->create_subscription<FloatStamped>(
      "/ap1/control/brake", 1,
      [this](const FloatStamped::SharedPtr f) {
        this->on_brake(f);
      }
    );

    // Publishers
    lanes_pub_ = this->create_publisher<LaneBoundaries>("/mapping/lanes", 1);
    entities_pub_ = this->create_publisher<EntityStateArray>("/mapping/entities", 1);
    

    RCLCPP_INFO(this->get_logger(), "Simulation ROS Node Started.");
  }

  void publish() {
    // Publish Car Speed
    FloatStamped speed_msg;
    speed_msg.value = this->sim.car.speed_mps;
    speed_pub_->publish(speed_msg);

    // Publish entities
    entities_pub_->publish(this->sim.entities);

    // Publish lanes
    lanes_pub_->publish(this->sim.lane);
  }
private:
  const ap1::sim::Sim &sim;
  // Callbacks
  void on_throttle(const FloatStamped::SharedPtr msg) {
    // check value first and crash otherwise
    if (msg->value < 0.f || msg->value > 1.f) {
      throw std::out_of_range("Throttle value out of range.");
    }

    // Set value
    this->last_throttle_cmd = msg->value;
  }

  void on_steer(const FloatStamped::SharedPtr msg) {
    // check value first and crash otherwise
    if (msg->value < -1.f || msg->value > 1.f) {
      throw std::out_of_range("Steer value out of range.");
    }

    // Set value
    this->last_steer_cmd_rad = msg->value;
  }

  void on_brake(const FloatStamped::SharedPtr msg) {
    // check value first and crash otherwise
    if (msg->value < 0.f || msg->value > 1.f) {
      throw std::out_of_range("Brake value out of range.");
    }

    // Set value
    this->last_brake_cmd = msg->value;
  }

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
