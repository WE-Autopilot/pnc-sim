#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>
#include <yaml-cpp/exceptions.h>

#include "yaml-cpp/yaml.h"
#include "yaml-cpp/node/parse.h"

#include "geometry_msgs/msg/point.hpp"

#include "ap1_msgs/msg/entity_state.hpp"
#include "ap1_msgs/msg/entity_state_array.hpp"
#include "ap1_msgs/msg/lane_boundaries.hpp"

#include "ap1/pnc_sim/sim_node.hpp"
#include "ap1/pnc_sim/sim.hpp"

#define UPDATE_RATE 100 // hz

using ap1_msgs::msg::EntityState;
using ap1_msgs::msg::EntityStateArray;
using ap1_msgs::msg::LaneBoundaries;
using geometry_msgs::msg::Point;

constexpr auto SIM_DT = std::chrono::duration<double>(1.0 / UPDATE_RATE);

void load_from_file(std::string path, EntityStateArray &entities, LaneBoundaries &lane, EntityState & car_state) {
    // read file
    YAML::Node config = YAML::LoadFile(path);

    // pull lanes
    lane.left.clear();
    for (const auto &pt : config["lane_boundary"]["left"]) {
        Point p;
        p.x = pt["x"].as<float>();
        p.y = pt["y"].as<float>();
        p.z = pt["z"].as<float>();
        lane.left.push_back(p); // TODO: emplace back?
    }
    lane.right.clear();
    for (const auto &pt : config["lane_boundary"]["right"]) {
        Point p;
        p.x = pt["x"].as<float>();
        p.y = pt["y"].as<float>();
        p.z = pt["z"].as<float>();
        lane.right.push_back(p);
    }

    // pull entities
    entities.entities.clear();
    for (const auto &pt : config["entities"]) {
        EntityState e;
        e.x = pt["x"].as<float>();
        e.y = pt["y"].as<float>();
        e.z = pt["z"].as<float>();
        e.gamma = pt["gamma"].as<float>();
        entities.entities.push_back(e);
    }

    // pull car state
    const auto &car_pt = config["car_pos"];
    car_state.x = car_pt["x"].as<float>();
    car_state.y = car_pt["y"].as<float>();
    car_state.z = car_pt["z"].as<float>();
    car_state.gamma = car_pt["gamma"].as<float>();

    // DEBUG:
    printf("Car state: {%f %f %f %f}\n", car_state.x, car_state.y, car_state.z, car_state.gamma);
}

int main(int argc, char ** argv)
{
    // init ROS
    rclcpp::init(argc, argv);
    
    // get file path
    if (argc < 2) throw std::runtime_error("Usage: sim </path/to/world.yaml>");
    std::string path = argv[1];

    // Load entities and lane from file
    EntityState car_state{};
    LaneBoundaries lane{};
    EntityStateArray entities{};
    try {
        load_from_file(path, entities, lane, car_state);
    } catch (const YAML::Exception& err) {
        throw std::runtime_error(
            "Failed to parse YAML! File: " + path + "\n" +
            "Line: " + std::to_string(err.mark.line + 1) + ", " +
            "Column: " + std::to_string(err.mark.column + 1) + "\n" +
            "Error: " + err.what() + "\n"
        );
    }

    // make sim
    auto sim = ap1::sim::Sim(entities, lane, car_state);

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

    // teardown
    rclcpp::shutdown();

    return 0;
}
