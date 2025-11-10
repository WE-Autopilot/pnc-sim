
# AP1 Basic Vehicle Simulation

This package simulates a small car model that connects to the Control Node.  
It’s used to test basic motion without needing the full planning stack. YET! More to come with that later...


## Overview

The **Control Node** publishes:
- `/motor_power` → throttle command (0–1)
- `/turn_angle` → steering command (radians)

The **Sim Node** (`SimpleVehicleSimNode`) subscribes to those and publishes:
- `/vehicle_speed` → current speed (m/s)
- `/vehicle_position` → current position (x, y, z)


## Key Parameters in `sim_node.hpp`

| Field | Meaning | Effect if Increased |
|--------|----------|---------------------|
| `mass_kg` | Vehicle mass | Slower acceleration |
| `max_force_n` | Engine power | Faster acceleration, higher top speed |
| `drag_coeff` | Air/rolling resistance | Lower top speed, stronger deceleration |
| `wheelbase_m` | Distance between axles | Wider turns |
| `max_steer_rad` | Max steering angle (radians) | Tighter turns |


## Changes Made for Control Connection

### In `ControlNode`
- Added publishers:
  ```cpp
  turning_angle_pub_ = this->create_publisher<std_msgs::msg::Float32>("turn_angle", 10);
  motor_power_pub_ = this->create_publisher<std_msgs::msg::Float32>("motor_power", 10);


* Added a test timer that publishes throttle=0.5, steer=0.0 every 100ms:

  ```cpp
  test_timer_ = this->create_wall_timer(
      100ms,
      std::bind(&ControlNode::on_test_timer, this)
  );
  ```

### In `Sim_Node`

* Subscribed to:

  ```cpp
  "motor_power"  // throttle
  "turn_angle"   // steering
  ```
* Publishes simulated state on:

  ```cpp
  "vehicle_speed"
  "vehicle_position"
  ```



## How to Build and Run

```bash
cd ~/Repo/ap1
colcon build --packages-select ap1_control ap1_pnc_sim
source install/setup.bash
```

### Terminal 1 – Run Control Node

```bash
ros2 run ap1_control control_node
```

### Terminal 2 – Run Simulation Node

```bash
ros2 run ap1_pnc_sim pnc_sim_node
```

### Terminal 3 – Check Output

```bash
ros2 topic echo /vehicle_speed
ros2 topic echo /vehicle_position
```

You should see the car moving forward in the sim (x increasing, y≈0).

