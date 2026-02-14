#include <chrono>
#include <thread>
#include "ap1/pnc_sim/sim_node.hpp"
#include "ap1/pnc_sim/sim.hpp"

#define UPDATE_RATE 100 // hz

constexpr auto SIM_DT = std::chrono::duration<double>(1.0 / UPDATE_RATE);

int main(int argc, char ** argv)
{
    // make sim
    auto sim = ap1::sim::Sim();

    // init ROS
    rclcpp::init(argc, argv);

    // make ros node
    auto ros_node = std::make_shared<ap1::sim::SimpleVehicleSimNode>();

    // spin ros node in separate thread
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(ros_node);

    // update the sim at UPDATE_RATE updates per second
    bool running = true;
    while (running && rclcpp::ok()) {
        auto frame_start = std::chrono::steady_clock::now();

        // update
        sim.update(SIM_DT.count());     // physics
        ros_node->publish(sim);         // publish state

        std::this_thread::sleep_until(frame_start + SIM_DT);
    }

    rclcpp::shutdown();

    return 0;
}
