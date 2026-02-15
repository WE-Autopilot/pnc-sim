#include <chrono>
#include <thread>
#include "ap1_msgs/msg/entity_state.hpp"
#include "ap1_msgs/msg/entity_state_array.hpp"
#include "ap1_msgs/msg/lane_boundaries.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "ap1/pnc_sim/sim_node.hpp"
#include "ap1/pnc_sim/sim.hpp"

#define UPDATE_RATE 100 // hz

using ap1_msgs::msg::EntityState;
using ap1_msgs::msg::EntityStateArray;
using ap1_msgs::msg::LaneBoundaries;
using geometry_msgs::msg::Point;

constexpr auto SIM_DT = std::chrono::duration<double>(1.0 / UPDATE_RATE);

int main(int argc, char ** argv)
{
    // TODO: add loading lane and entities from file
    EntityStateArray entities{};
    LaneBoundaries lane{};

    // make sim
    auto sim = ap1::sim::Sim(entities, lane);

    // init ROS
    rclcpp::init(argc, argv);

    // make ros node
    auto ros_node = std::make_shared<ap1::sim::SimNode>(sim);

    // update the sim at UPDATE_RATE updates per second
    while (rclcpp::ok()) {
        auto frame_start = std::chrono::steady_clock::now();

        // allow the node to process
        rclcpp::spin_some(ros_node);

        // update physics
        sim.update(
            SIM_DT.count(),
            ros_node->last_steer_cmd_rad,
            ros_node->last_throttle_cmd,
            ros_node->last_brake_cmd
        );
        // publish state
        ros_node->publish();

        // check frame times
        auto frame_end = std::chrono::steady_clock::now();
        auto frame_duration = frame_end - frame_start;
        if (frame_duration > SIM_DT) {
            double ms = std::chrono::duration<double, std::milli>(frame_duration).count();
            double target_ms = std::chrono::duration<double, std::milli>(SIM_DT).count();

            RCLCPP_WARN_THROTTLE(
                ros_node->get_logger(),
                *ros_node->get_clock(),
                2000, // ms (throttle to update every 2 seconds)
                "Frame overrun: %.3f ms (target %.3f ms)",
                ms,
                target_ms
            );
        }

        std::this_thread::sleep_until(frame_start + SIM_DT);
    }

    rclcpp::shutdown();

    return 0;
}
