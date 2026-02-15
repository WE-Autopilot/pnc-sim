#include "ap1/pnc_sim/sim_node.hpp"

#include <cmath>
#include <rclcpp/node_options.hpp>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"

#include "ap1/pnc_sim/sim.hpp"
#include "ap1_msgs/msg/entity_state.hpp"
#include "ap1_msgs/msg/float_stamped.hpp"
#include "ap1_msgs/msg/entity_state_array.hpp"
#include "ap1_msgs/msg/lane_boundaries.hpp"

template<typename T>
T to_car_frame(
  const T& p,
  float car_x,
  float car_y,
  float car_yaw
) {
  T out;

  float dx = p.x - car_x;
  float dy = p.y - car_y;

  float c = std::cos(car_yaw);
  float s = std::sin(car_yaw);

  out.x = c * dx + s * dy;
  out.y = -s * dx + c * dy;
  out.z = p.z;

  return out;
}

// Default Constructor
ap1::sim::SimNode::SimNode(const ap1::sim::Sim &sim)
    : Node("sim_node"), sim(sim) 
{
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
    speed_pub_ = this->create_publisher<FloatStamped>("/ap1/actuation/speed", 1);
    lanes_pub_ = this->create_publisher<LaneBoundaries>("/ap1/mapping/lanes", 1);
    entities_pub_ = this->create_publisher<EntityStateArray>("/ap1/mapping/entities", 1);

    // Log
    RCLCPP_INFO(this->get_logger(), "Simulation ROS Node Started.");
}

void ap1::sim::SimNode::publish() {
    float car_x = sim.car.x;
    float car_y = sim.car.y;
    float car_yaw = sim.car.yaw;

    // Speed
    FloatStamped speed_msg;
    speed_msg.value = this->sim.car.speed_mps;
    speed_pub_->publish(speed_msg);

    // Entities
    auto entities_msg = sim.entities; // copy
    for (auto& e : entities_msg.entities) {
        e = to_car_frame(e, car_x, car_y, car_yaw);
        e.gamma -= car_yaw; // rotate orientation into car frame
    }
    entities_pub_->publish(entities_msg);

    // Lane
    auto lane_msg = sim.lane; // copy
    for (auto& p : lane_msg.left) {
        p = to_car_frame(p, car_x, car_y, car_yaw);
    }
    for (auto& p : lane_msg.right) {
        p = to_car_frame(p, car_x, car_y, car_yaw);
    }
    lanes_pub_->publish(lane_msg);
}

// Callbacks
void ap1::sim::SimNode::on_throttle(const FloatStamped::SharedPtr msg) {
    // check value first and crash otherwise
    if (msg->value < 0.f || msg->value > 1.f) {
        throw std::out_of_range("Throttle value out of range.");
    }

    // Set value
    this->last_throttle_cmd = msg->value;
}

void ap1::sim::SimNode::on_steer(const FloatStamped::SharedPtr msg) {
    // check value first and crash otherwise
    if (msg->value < -1.f || msg->value > 1.f) {
        throw std::out_of_range("Steer value out of range.");
    }

    // Set value
    this->last_steer_cmd_rad = msg->value;
}

void ap1::sim::SimNode::on_brake(const FloatStamped::SharedPtr msg) {
    // check value first and crash otherwise
    if (msg->value < 0.f || msg->value > 1.f) {
        throw std::out_of_range("Brake value out of range.");
    }

    // Set value
    this->last_brake_cmd = msg->value;
}
