# OP3 P0 ROS 2 Description

[English](README_EN.md) | 简体中文

`omnipicker_3_description` 提供 OP3 P0 夹爪模型、单滑条闭环运动学控制及 RViz2 可视化。适用于 ROS 2 Jazzy / Humble，使用 `ament_cmake` 和 `colcon` 构建。

## 包内容

| 路径 | 内容 |
| --- | --- |
| `urdf/OP3_P0_URDF_EC.urdf` | 主模型：几何、关节及惯性参数 |
| `urdf/xacro/op3_p0_urdf_ec.xacro` | Xacro 入口 |
| `meshes/` | STL 网格 |
| `rviz/` | RViz2 配置 |
| `launch/` | ROS 2 启动文件 |
| `scripts/gripper_control.py` | 单滑条及闭环状态发布节点 |

构建后，`urdf/`、`meshes/`、`rviz/` 下的资源安装到 `share/omnipicker_3_description/`，控制脚本安装到 `lib/omnipicker_3_description/`。

## 安装与启动

先安装 ROS 2。以下以 Jazzy 为例，Humble 环境将 `jazzy` 替换为 `humble`；将 `/path/to/OP3_P0_URDF_EC` 替换为交付文件夹路径。

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

每个新终端均需加载 ROS 2 环境及 `~/op3_ws/install/setup.bash`。

## 开合控制

- 窗口仅有 `joint1` 一根滑条：`0～20 mm`，步进 `0.01 mm`；零行程对应闭合端，增大行程打开夹爪。
- 节点自动求解左右闭环，发布全部 13 个非固定关节的 `/joint_states`；无需单独调节从动关节。
- 外部指令话题为 `/joint1_command`，类型为 `std_msgs/msg/Float64`，单位为米，范围 `0～0.020`。

例如移动到 `10 mm`：

```bash
ros2 topic pub --once /joint1_command std_msgs/msg/Float64 "{data: 0.010}"
```

越界、非有限值或无法闭环的指令会被拒绝，并保持上一有效姿态。GUI 同步显示外部指令。此接口仅用于运动学预览，不直接驱动硬件，也不执行速度规划。

## 启动参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `gui` | `true` | 显示单滑条窗口 |
| `rviz` | `true` | 启动 RViz2 |
| `publish_joint_states` | `true` | 启用本包的闭环状态发布节点 |
| `use_sim_time` | `false` | 使用仿真器的 `/clock` |
| `initial_position` | `0.0` | 初始行程，单位为米 |

无界面启动并设定初始行程为 `10 mm`：

```bash
ros2 launch omnipicker_3_description op3_p0_urdf_ec_launch.py \
  gui:=false rviz:=false initial_position:=0.010
```

接入外部关节状态时，设置 `publish_joint_states:=false`。此时滑条和闭环节点均关闭，外部节点必须发布完整 `/joint_states`，仅发送 `/joint1_command` 无效。启用 `use_sim_time:=true` 时需持续提供 `/clock`。

## 模型与接入

- 模型包含 16 个连杆、15 个关节，实体总质量 `0.5618 kg`，根坐标系为 `base_link`。
- 单位为 m、kg、rad、kg·m²；惯量定义在重心处，惯性坐标轴与连杆坐标轴一致。
- `link13`、`link23`、`link18`、`link28` 为无独立质量和网格的连接参考节点。
- 网格 URI 为 `package://omnipicker_3_description/meshes/...`。外部导入器需解析到安装包的 `share/omnipicker_3_description/` 或对应源码资源。
- 基座组件 STL 以毫米建模，URDF 使用 `0.001` 缩放；整体连杆 STL 使用米。导入时遵循各网格 `scale`，不要重复缩放。

从安装包展开 Xacro：

```bash
package_share="$(ros2 pkg prefix omnipicker_3_description)/share/omnipicker_3_description"
ros2 run xacro xacro "$package_share/urdf/xacro/op3_p0_urdf_ec.xacro" \
  -o /tmp/op3_p0_model.urdf
```

RViz2 固定坐标系设为 `base_link`；RobotModel 使用 `/robot_description`，QoS 为 Reliable / Transient Local。模型不显示时，检查包环境、网格路径及 `/joint_states`、TF 发布情况。

本包不包含动力学仿真工程。Gazebo、Isaac Sim 等需另行配置驱动、控制器、世界及闭环约束；关节 `effort`、`velocity` 当前为零，应按实际硬件设置。

## STL 文件名对照

`meshes/` 为平铺目录，原中文文件名已改为英文：

| 原文件名 | 现文件名 |
| --- | --- |
| 上壳_l / 上壳_r | `upper_shell_l` / `upper_shell_r` |
| 下壳_l / 下壳_r | `lower_shell_l` / `lower_shell_r` |
| 法兰接口 | `flange` |
| 滚珠丝杠 | `ball_screw` |
| 轴承端盖 | `bearing_cap` |
| 透光板_l / 透光板_r | `light_panel_l` / `light_panel_r` |
