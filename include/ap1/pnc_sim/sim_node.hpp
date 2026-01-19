/**
 * Created: Nov. 9, 2025
 * Author(s): Obaid
 */

#ifndef AP1_SIM_NODE_HPP
#define AP1_SIM_NODE_HPP

#include <chrono>
#include <cmath>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "ap1_msgs/msg/motor_power_stamped.hpp"
#include "ap1_msgs/msg/turn_angle_stamped.hpp"
#include "ap1_msgs/msg/vehicle_speed_stamped.hpp"
#include "ap1_msgs/msg/float_stamped.hpp"

namespace ap1::sim
{

class SimpleVehicleSimNode : public rclcpp::Node
{
  public:
    SimpleVehicleSimNode()
        : Node("simple_vehicle_sim_node"), current_throttle_(0.0f), current_steer_rad_(0.0f)
    {
        cfg_.mass_kg = 1200.0f;     // car
        cfg_.max_force_n = 4000.0f; // so called "engine" force at full throttle
        cfg_.drag_coeff = 0.5f;     // simple linear drag
        cfg_.wheelbase_m = 2.5f;
        cfg_.max_steer_rad = 0.5f;

        // Initial state
        state_.x_m = 0.0f;
        state_.y_m = 0.0f;
        state_.yaw = 0.0f;
        state_.v_mps = 0.0f;

        last_update_ = this->now();

        // Subscriptions from Control
        throttle_sub_ = this->create_subscription<ap1_msgs::msg::FloatStamped>(
            "/ap1/control/motor_power", 10,
            std::bind(&SimpleVehicleSimNode::on_throttle, this, std::placeholders::_1));

        steer_sub_ = this->create_subscription<ap1_msgs::msg::FloatStamped>(
            "/ap1/control/turn_angle", 10,
            std::bind(&SimpleVehicleSimNode::on_steer, this, std::placeholders::_1));

        // Publishers to Planning
        speed_pub_ = this->create_publisher<ap1_msgs::msg::FloatStamped>(
            "/ap1/actuation/speed_actual", 10);
        position_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
            "/ap1/actuation/turn_angle_actual", 10);

        // Timer to step the simulation at fixed rate
        using namespace std::chrono_literals;
        sim_timer_ = this->create_wall_timer(20ms, // 50 Hz
                                             std::bind(&SimpleVehicleSimNode::on_timer, this));

        RCLCPP_INFO(this->get_logger(), "Simple Vehicle Sim Node initialized");
    }

    /**

    mass_kg	= inertia which makes acceleration slower, car feels heavier
    max_force_n	= engine power, raises acceleration and top speed
    drag_coeff = air/friction resistance, reduces top speed and causes quicker slow-down when
    throttle drops wheelbase_m = distance between axles, influences turning radius — longer = wider
    turns max_steer_rad = steering limit, allows tighter turns if larger

    */
  private:
    // sample configuration for the car
    struct CarConfig
    {
        float mass_kg;
        float max_force_n;
        float drag_coeff;
        float wheelbase_m;
        float max_steer_rad;
    };

    // Car state in 2D
    struct CarState
    {
        float x_m;
        float y_m;
        float yaw;   // radians
        float v_mps; // forward speed
    };

    // Callbacks
    void on_throttle(const ap1_msgs::msg::FloatStamped::SharedPtr msg)
    {
        // Expecting throttle in range [-1, 1] or [0, 1]
        current_throttle_ = msg->value;
        // apply to [0, 1] for now
        if (current_throttle_ < 0.0f)
            current_throttle_ = 0.0f;
        if (current_throttle_ > 1.0f)
            current_throttle_ = 1.0f;
    }

    void on_steer(const ap1_msgs::msg::FloatStamped::SharedPtr msg)
    {
        current_steer_rad_ = msg->value;

        // Clamp steering to physical limits
        if (current_steer_rad_ > cfg_.max_steer_rad)
            current_steer_rad_ = cfg_.max_steer_rad;
        if (current_steer_rad_ < -cfg_.max_steer_rad)
            current_steer_rad_ = -cfg_.max_steer_rad;
    }

    void on_timer()
    {
        // Compute dt since last update
        rclcpp::Time now = this->now();
        double dt = (now - last_update_).seconds();
        if (dt <= 0.0)
        {
            dt = 0.02; // fallback
        }
        last_update_ = now;

        step_dynamics(static_cast<float>(dt));
        publish_state();
    }

    // I hope this works but very very questionable

    // Basic longitudinal + kinematic bicycle dynamics
    void step_dynamics(float dt)
    {
        // Longitudinal force from engine
        float engine_force = current_throttle_ * cfg_.max_force_n;

        // Very crude drag, proportional to speed
        float drag_force = cfg_.drag_coeff * state_.v_mps;

        // Net force and acceleration
        float net_force = engine_force - drag_force;
        float accel = net_force / cfg_.mass_kg;

        // Integrate velocity
        state_.v_mps += accel * dt;
        if (state_.v_mps < 0.0f)
        {
            state_.v_mps = 0.0f;
        }

        // Kinematic turning
        float beta = std::tan(current_steer_rad_);
        float yaw_rate = (state_.v_mps / cfg_.wheelbase_m) * beta;

        state_.yaw += yaw_rate * dt;

        // Integrate position
        state_.x_m += state_.v_mps * std::cos(state_.yaw) * dt;
        state_.y_m += state_.v_mps * std::sin(state_.yaw) * dt;
    }

    void publish_state()
    {
        // Speed to Planner
        ap1_msgs::msg::FloatStamped speed_msg;
        speed_msg.value = state_.v_mps;
        speed_pub_->publish(speed_msg);

        // Position to whoever wants it (need to add a "subscriber" in Planner, currently connected
        // to control)
        geometry_msgs::msg::Point pos_msg;
        pos_msg.x = state_.x_m;
        pos_msg.y = state_.y_m;
        pos_msg.z = 0.0;
        position_pub_->publish(pos_msg);
    }

    // Config and state
    CarConfig cfg_;
    CarState state_;

    // Inputs from Control
    float current_throttle_;
    float current_steer_rad_;

    // ROS stuff
    rclcpp::Subscription<ap1_msgs::msg::FloatStamped>::SharedPtr throttle_sub_;
    rclcpp::Subscription<ap1_msgs::msg::FloatStamped>::SharedPtr steer_sub_;

    rclcpp::Publisher<ap1_msgs::msg::FloatStamped>::SharedPtr speed_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr position_pub_;

    rclcpp::TimerBase::SharedPtr sim_timer_;
    rclcpp::Time last_update_;
};

} // namespace ap1::sim

#endif
