# OP3 P0 ROS 2 Description

English | [简体中文](README.md)

`omnipicker_3_description` provides the OP3 P0 gripper model, single-slider closed-loop kinematic control, and RViz2 visualization. It supports ROS 2 Jazzy / Humble and uses `ament_cmake` and `colcon`.

## Package Contents

| Path | Contents |
| --- | --- |
| `urdf/OP3_P0_URDF_EC.urdf` | Main model: geometry, joints, and inertial parameters |
| `urdf/xacro/op3_p0_urdf_ec.xacro` | Xacro entry point |
| `meshes/` | STL meshes |
| `rviz/` | RViz2 configuration |
| `launch/` | ROS 2 launch file |
| `scripts/gripper_control.py` | Single-slider and closed-loop state publisher |

The `urdf/`, `meshes/` and `rviz/` directories are installed under `share/omnipicker_3_description/`, and the control script is installed under `lib/omnipicker_3_description/`.

## Installation and Launch

Install ROS 2 first. The following commands use Jazzy; replace `jazzy` with `humble` for Humble. Replace `/path/to/OP3_P0_URDF_EC` with the delivered folder path.

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install python3-colcon-common-extensions \
  ros-jazzy-ament-cmake ros-jazzy-ament-index-python \
  ros-jazzy-launch ros-jazzy-launch-ros ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher ros-jazzy-rclpy ros-jazzy-sensor-msgs \
  ros-jazzy-std-msgs ros-jazzy-python-qt-binding ros-jazzy-rviz2

mkdir -p ~/op3_ws/src
cp -r /path/to/OP3_P0_URDF_EC ~/op3_ws/src/omnipicker_3_description
cd ~/op3_ws
colcon build --packages-select omnipicker_3_description
source install/setup.bash
ros2 launch omnipicker_3_description op3_p0_urdf_ec_launch.py
```

In every new terminal, source the ROS 2 environment and `~/op3_ws/install/setup.bash`.

## Gripper Control

- The window contains only the `joint1` slider: `0–20 mm`, with `0.01 mm` increments. Zero stroke is the closed end; increasing the stroke opens the gripper.
- The node solves both closed-loop linkages and publishes `/joint_states` for all 13 non-fixed joints. Passive joints require no separate adjustment.
- External commands use `/joint1_command`, type `std_msgs/msg/Float64`, in meters within `0–0.020`.

For example, move to `10 mm`:

```bash
ros2 topic pub --once /joint1_command std_msgs/msg/Float64 "{data: 0.010}"
```

Out-of-range, non-finite, or infeasible commands are rejected, retaining the last valid pose. The GUI reflects external commands. This interface is for kinematic preview only; it does not drive hardware or perform velocity planning.

## Launch Arguments

| Argument | Default | Description |
| --- | --- | --- |
| `gui` | `true` | Show the single-slider window |
| `rviz` | `true` | Start RViz2 |
| `publish_joint_states` | `true` | Enable this package's closed-loop state publisher |
| `use_sim_time` | `false` | Use the simulator's `/clock` |
| `initial_position` | `0.0` | Initial stroke in meters |

Launch without a GUI at an initial stroke of `10 mm`:

```bash
ros2 launch omnipicker_3_description op3_p0_urdf_ec_launch.py \
  gui:=false rviz:=false initial_position:=0.010
```

Set `publish_joint_states:=false` to use external joint states. This disables both the slider and the closed-loop node; an external node must publish complete `/joint_states`. Sending only `/joint1_command` has no effect. With `use_sim_time:=true`, provide a continuous `/clock`.

## Model and Integration

- The model has 16 links and 15 joints, with a physical mass total of `0.5618 kg` and root frame `base_link`.
- Units are m, kg, rad, and kg·m². Inertia is defined at each center of mass, with axes aligned to the link frame.
- `link13`, `link23`, `link18`, and `link28` are connection reference nodes without independent mass or meshes.
- Mesh URIs use `package://omnipicker_3_description/meshes/...`. External importers must resolve them to the installed `share/omnipicker_3_description/` directory or the corresponding source assets.
- Base component STL files use millimeters and a URDF scale of `0.001`; full-link STL files use meters. Respect each mesh's `scale` and do not scale twice.

Expand Xacro from the installed package:

```bash
package_share="$(ros2 pkg prefix omnipicker_3_description)/share/omnipicker_3_description"
ros2 run xacro xacro "$package_share/urdf/xacro/op3_p0_urdf_ec.xacro" \
  -o /tmp/op3_p0_model.urdf
```

Set the RViz2 fixed frame to `base_link`. RobotModel uses `/robot_description` with Reliable / Transient Local QoS. If the model is missing, check the package environment, mesh paths, `/joint_states`, and TF publishers.

This package does not include a dynamics simulation project. Gazebo, Isaac Sim, and similar tools require separate actuator, controller, world, and closed-loop constraint configuration. Joint `effort` and `velocity` limits are currently zero and must be set for the actual hardware.
