# AGENTS.md — smart_wheelchair

本文档供 AI coding agent 阅读，用于快速理解本项目的结构、构建方式与开发约定。

---

## 项目概览

本项目是一个 **ROS Noetic** 的 catkin workspace，包含一个名为 `smart_wheelchair` 的功能包。该包定义了一款智能轮椅机器人的 **URDF/Xacro 模型**，并附带在 **Gazebo** 中仿真以及在 **RViz** 中可视化的启动配置。

当前包内**没有活跃的 C++ 或 Python 源码**，仅包含模型描述文件与启动文件，因此它是一个纯粹的机器人描述/仿真包。

### 核心能力

- 使用 Xacro 参数化定义轮椅的连杆（link）与关节（joint）结构
- 集成 Gazebo 差速驱动插件（`libgazebo_ros_diff_drive.so`），支持通过 `cmd_vel` 话题控制
- 集成 Gazebo 激光雷达插件（`libgazebo_ros_laser.so`），发布 `scan` 话题
- 集成 Gazebo 摄像头插件（`libgazebo_ros_camera.so`），发布 `image_raw` 与 `camera_info` 话题
- 提供预配置的 RViz 布局文件，可直接查看模型与 TF 树

---

## 技术栈

| 层级 | 技术 |
|------|------|
| 操作系统 | Linux (Ubuntu 20.04 / ROS Noetic) |
| 构建系统 | catkin / CMake |
| 仿真器 | Gazebo (配合 `gazebo_ros_pkgs`) |
| 可视化 | RViz |
| 建模 | URDF / Xacro |
| 语言 | 无活跃源码（仅 XML/launch 配置） |

---

## 项目结构

```
wheelchair_ws/
├── .catkin_workspace          # catkin workspace 标记文件
├── src/
│   ├── CMakeLists.txt         # 软链接到 /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
│   └── smart_wheelchair/
│       ├── CMakeLists.txt     # 包级 CMake（当前未编译任何可执行文件/库）
│       ├── package.xml        # ROS 包清单
│       ├── launch/
│       │   ├── display.launch    # 在 RViz 中加载并显示模型
│       │   └── gazebo.launch     # 在 Gazebo 空世界中加载并仿真模型
│       ├── urdf/
│       │   └── wheelchair.xacro  # 轮椅的完整 Xacro 描述（含 Gazebo 插件）
│       └── config/
│           └── wheelchair.rviz   # RViz 显示配置文件
├── build/                     # catkin 构建输出目录
└── devel/                     # catkin 开发环境目录（setup 脚本等）
```

---

## 构建与测试命令

### 首次构建

```bash
cd /home/soyorin/wheelchair_ws
catkin_make
```

构建完成后，建议 source 环境：

```bash
source devel/setup.bash
```

### 运行

**1. RViz 可视化（仅查看模型，无物理仿真）**

```bash
roslaunch smart_wheelchair display.launch
```

该启动文件会：
- 加载 `wheelchair.xacro` 到参数服务器 `robot_description`
- 启动 `joint_state_publisher_gui`（提供可拖动调节的关节 GUI）
- 启动 `robot_state_publisher`（发布 TF 变换）
- 启动 `rviz` 并加载预配置 `wheelchair.rviz`

**2. Gazebo 仿真**

```bash
roslaunch smart_wheelchair gazebo.launch
```

该启动文件会：
- 加载 `wheelchair.xacro` 到参数服务器
- 启动 Gazebo 空世界
- 使用 `gazebo_ros spawn_model` 在 `(0, 0, 0.1)` 处生成机器人

### 测试

当前包内没有启用任何测试目标。`CMakeLists.txt` 中的 `catkin_add_gtest` 与 `catkin_add_nosetests` 均被注释掉。如需添加测试，请取消相应注释并补充测试文件到 `test/` 目录。

---

## 机器人模型关键参数

`wheelchair.xacro` 中定义的主要结构：

- **base_link**: 主框架、座面、靠背立柱、脚踏板支撑、侧板等一体化连杆（质量 28 kg）
- **backrest_link**: 靠背垫（固定关节）
- **left_armrest_link / right_armrest_link**: 左右扶手（固定关节）
- **footrest_link**: 脚踏板（固定关节）
- **left_wheel_link / right_wheel_link**: 驱动后轮（`continuous` 关节，绕 Y 轴旋转）
- **front_caster_link / rear_caster_link**: 前后万向轮（`fixed` 关节）
- **lidar_link**: 激光雷达（固定于底座上方）
- **camera_link**: 摄像头（固定于 lidar_link 上方）

### Gazebo 插件配置

| 插件 | 类型 | 关键话题 |
|------|------|---------|
| 差速驱动 | `libgazebo_ros_diff_drive.so` | `cmd_vel` (订阅), `odom` (发布) |
| 激光雷达 | `libgazebo_ros_laser.so` | `scan` (发布) |
| 摄像头 | `libgazebo_ros_camera.so` | `image_raw`, `camera_info` (发布) |

---

## 代码风格与开发约定

- **注释语言**：所有 launch 文件与 xacro 文件中的注释均使用**中文**。新增注释请保持一致。
- **命名规范**：ROS 包名使用小写+下划线；link/joint 名使用小写+下划线；Xacro property 名使用缩写（如 `wh_r`, `ca_x`）。
- **建模约定**：每个 link 通常同时包含 `<visual>`、`<collision>` 与 `<inertial>`；颜色通过 `<material>` 标签集中定义。
- **未使用的 CMake 模板**：`CMakeLists.txt` 保留了大量 catkin 生成的注释模板。在添加 C++ 节点/库时，可直接取消对应注释并填写实际文件名。

---

## 依赖清单

### Build & Run 依赖

- `catkin`
- `gazebo_plugins`
- `gazebo_ros`
- `geometry_msgs`
- `joint_state_publisher_gui`
- `robot_state_publisher`
- `rospy`
- `rviz`
- `sensor_msgs`
- `std_msgs`
- `tf2_ros`
- `urdf`
- `xacro`

安装缺失依赖（在 Ubuntu/ROS Noetic 环境下）：

```bash
rosdep install --from-paths src --ignore-src -y
```

---

## 安全与注意事项

1. **Gazebo 物理参数**：`base_link` 的摩擦系数设置为 `mu1=0.3, mu2=0.3`；驱动轮为 `0.8`；万向轮为 `0.0`（理想滚动）。修改这些值会显著影响差速控制的滑动行为。
2. **雷达范围**：激光雷达的最小检测距离为 `0.10 m`，最大为 `10.0 m`。在极近距离避障场景中，此配置可能产生盲区。
3. **摄像头 clip**：近裁剪面 `0.05 m`，远裁剪面 `100 m`。如需用于视觉算法，请确认该参数与算法期望一致。
4. **无实际控制节点**：当前包仅提供仿真模型，没有实现任何上层控制、导航或感知节点。控制轮椅运动需手动向 `cmd_vel` 发布 `geometry_msgs/Twist` 消息。

---

## 快速参考

| 目的 | 命令 |
|------|------|
| 构建工作空间 | `catkin_make` |
| 加载环境 | `source devel/setup.bash` |
| RViz 查看模型 | `roslaunch smart_wheelchair display.launch` |
| Gazebo 仿真 | `roslaunch smart_wheelchair gazebo.launch` |
| 安装依赖 | `rosdep install --from-paths src --ignore-src -y` |
| 发送速度指令 | `rostopic pub /cmd_vel geometry_msgs/Twist ...` |
