# Smart Wheelchair ROS Simulation Project

这是一个基于 **ROS Noetic** 和 **Gazebo 11** 的智能轮椅仿真项目。本项目实现了一款具备激光雷达、摄像头和深度传感器集成的智能轮椅模型，并完成了 SLAM 建图与自主导航系统的配置与部署。

---

## 🚀 项目核心特性

- **高精度建模**：使用 Xacro 参数化定义的轮椅物理模型，包括驱动轮、万向轮及人体工程学组件。
- **平稳物理特性**：针对仿真环境优化的摩擦系数（万向轮 mu=0）与碰撞箱设计，彻底解决了“托底”与驱动打滑问题。
- **鲁棒的定位系统**：
    - 集成 **Gmapping SLAM**，针对大户型环境优化了粒子滤波参数。
    - 配置 **AMCL** 自适应定位，实现了转向时的快速位姿校正（update_min_a=0.1）。
- **感官视野优化**：雷达扫描范围经过软件裁剪（270°），有效屏蔽了轮椅靠背自遮挡产生的干扰。
- **导航全栈**：集成 `move_base` 与 DWA 局部路径规划器，支持动态避障。

---

## 🛠️ 技术栈

| 类别 | 技术 |
|------|------|
| 操作系统 | Ubuntu 20.04 (ROS Noetic) |
| 仿真引擎 | Gazebo 11 |
| 传感器 | 2D Lidar, RGB Camera, IMU |
| 算法库 | Gmapping, AMCL, move_base (DWA) |
| 编程语言 | C++, Python, URDF/Xacro |

---

## 📂 项目结构

```bash
smart_wheelchair/
├── config/             # RViz 布局与 nav.rviz 配置
├── launch/             # 核心启动文件 (display, gazebo, slam, navigation)
├── maps/               # 已生成的仿真环境地图 (.yaml, .pgm)
├── meshes/             # 复杂的 3D 模型文件 (DAE, STL)
├── param/              # 导航参数配置 (AMCL, Costmap, Planner)
├── urdf/               # 机器人 Xacro 描述文件与 Gazebo 插件
└── worlds/             # 仿真世界场景 (bighouse.world)
```

---

## 📖 快速开始

### 1. 安装依赖
```bash
rosdep install --from-paths src --ignore-src -y
```

### 2. 构建工作空间
```bash
catkin_make
source devel/setup.bash
```

### 3. 运行 SLAM 建图
```bash
roslaunch smart_wheelchair slam.launch
```
*提示：可在此模式下使用控制台或手柄移动机器人，扫描完成后使用 `map_saver` 保存地图。*

### 4. 运行自主导航
```bash
roslaunch smart_wheelchair navigation.launch
```
*操作：在 RViz 中点击 "2D Nav Goal" 设置目标点，观察轮椅的避障规划。*

---

## 💡 开发心得与填坑指南

本项目在开发过程中解决了若干关键痛点，并总结在 `report.md` 中：
- **TF Fighting**：通过关闭 EKF 的 TF 发布，解决了机器人在 RViz 中“反复横跳”的同步冲突。
- **转向漂移**：通过极大降低 AMCL 对里程计转向误差的信任度（odom_alpha），实现了雷达扫描线与地图的完美吸附。
- **万向轮干扰**：将万向轮设为物理上的“绝对零摩擦”，消除了转弯时的侧滑阻力。

---

## 🤝 贡献与反馈

如果你对本项目感兴趣，欢迎提 Issue 或 PR。

**GitHub**: [soyorin01/smart_wheelchair](https://github.com/soyorin01/smart_wheelchair)
