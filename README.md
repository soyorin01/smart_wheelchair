# Smart Wheelchair ROS Simulation Project

这是一个基于 **ROS Noetic** 和 **Gazebo 11** 的智能轮椅仿真项目。本项目实现了一款具备激光雷达、摄像头和深度传感器集成的智能轮椅模型，并完成了 SLAM 建图与自主导航系统的配置与部署。

---

## 🚀 项目核心特性

- **高精度建模**：使用 Xacro 参数化定义的轮椅物理模型，包括驱动轮、万向轮及人体工程学组件。
- **平稳物理特性**：针对仿真环境优化的摩擦系数（万向轮 mu=0）与碰撞箱设计，彻底解决了“托底”与驱动打滑问题。
- **鲁棒的定位系统**：
    - 集成 **Gmapping SLAM**，针对大户型环境优化了粒子滤波参数。
    - 配置 **AMCL** 自适应定位，实现了转向时的快速位姿校正（update_min_a=0.1）。
- **视觉感知与目标跟随**：
    - **[模块 1] 图像采集与预处理**：搭建 OpenCV 视觉处理流，完成图像灰度化与高斯滤波处理去噪计算，并转换回 ROS Image 消息发布重分发。
    - **[模块 2] 目标物体识别提取**：结合 HSV 色彩空间转换与形态学滤波（开闭运算）进行动态目标筛选提取，通过轮廓面积几何判断找准目标 Bounding Box 外接框。
    - **[模块 3] 视距随动跟随**：融合 CSRT/KCF Tracker 与自适应 P-Controller，通过屏幕中心的横向误差计算角速度（转弯），面积距离阈值推断安全线速度（跟进与停车），实现精准的无轨迹视觉伺服跟随 (Visual Servoing)。
- **三维点云处理 (加分项)**：
    - 整合 **PCL** 库，建立深度相机 C++ 点云处理管线。
    - 基于在 Gazebo 中的场景实现直通滤波 (PassThrough)、体素降采样 (VoxelGrid)。
    - 使用 **RANSAC** 完美剔除地面平面，并运行**欧式提取聚类算法 (Euclidean Cluster Extraction)** 切分出独立障碍物与目标并分别附色高亮。
- **感官视野优化**：雷达扫描范围经过软件裁剪（270°），有效屏蔽了轮椅靠背自遮挡产生的干扰。
- **导航全栈**：集成 `move_base` 与 DWA 局部路径规划器，支持动态避障。

---

## 🛠️ 技术栈

| 类别 | 技术 |
|------|------|
| 操作系统 | Ubuntu 20.04 (ROS Noetic) |
| 仿真引擎 | Gazebo 11 |
| 传感器 | 2D Lidar, RGB Camera, Depth Camera, IMU |
| 算法库 | OpenCV, PCL, Gmapping, AMCL, move_base (DWA) |
| 编程语言 | C++, Python, URDF/Xacro |

---

## 📂 项目结构

```bash
smart_wheelchair/
├── config/             # RViz 布局预设 (pcl.rviz, wheelchiar.rviz 等)
├── launch/             # 仿真、视觉跟随、点云、SLAM和导航等启动文件
├── maps/               # 已生成的仿真环境地图 (.yaml, .pgm)
├── meshes/             # 复杂的 3D 模型文件 (DAE, STL)
├── model/              # 自定义 Gazebo 测试道具 (桌子、彩色方块、瓶子等)
├── param/              # 导航参数配置 (AMCL, Costmap, Planner)
├── scripts/            # 计算机视觉与追踪等 Python 处理节点 (OpenCV)
├── src/                # 三维点云处理 C++ 节点 (PCL)
├── urdf/               # 机器人 Xacro 描述文件与 Gazebo 插件
└── worlds/             # 仿真世界场景 (bighouse.world)
```

### 📜 `scripts/` 核心节点文件说明

- `teleop_keyboard.py`：自研的键盘遥控节点，支持 WASD/方向键以及多档位调速、急停控制。
- `image_preprocess_node.py`：**(对应模块 1)** 订阅仿真摄像头话题并桥接至 OpenCV，执行灰度化与高斯滤波平滑运算，发布 `/image_gray` 和 `/image_blur`。
- `object_detection_node.py`：**(对应模块 2)** 订阅滤波图像，利用 HSV 色彩空间精准提取特定颜色目标（如红色几何体/瓶子），并应用形态学（开闭运算）去噪后框选提取最大目标，发布目标包围盒坐标。
- `robot_following_node.py`：**(对应模块 3)** 视觉伺服与追踪底盘控制节点。通过识别框初始化 CSRT/KCF Tracker 实现稳健抗遮挡跟踪。内部构建了 P 系数调节器，将横向像素误差映射为 `/cmd_vel` 的角速度（转向），将目标面积大小映射为安全线速度（行进与驻停）。

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

### 5. 运行视觉识别与伺服跟随 (涵盖并打通了模块 1、2、3)
首先请将目标道具（比如红蓝方块/瓶子等模型已经引入 Gazebo 中），然后打开本启动文件：
```bash
roslaunch smart_wheelchair sim_env.launch                # 加载带有桌面与彩色目标的仿真世界
```
接着在新的终端运行综合视觉启动文件，直接拉起图像处理（模块1）、颜色筛选框定（模块2）、伺服锁定跟随（模块3）三个连续流：
```bash
roslaunch smart_wheelchair vision_following.launch       # 启动三合一视觉管线
```
*提示：运行后，`cv2.imshow` 将在本地桌面自动弹出 **灰度/滤波图**（模块1能力验证）、**识别包围盒**（模块2能力验证）和 **追踪锁定框**（模块3能力验证）。您可以用鼠标在 Gazebo 内拖拽物体测试底盘的跟随位移和自主停止效果。*

### 6. 三维点云滤波与欧式聚类提取 (PCL 加分项)
```bash
roslaunch smart_wheelchair gazebo_pcl.launch
```
*提示：将自动加载机器人及其深度相机配置以及配好的 RViz 界面，实时剔除地面（绿色）并将非地面多目标通过颜色进行分簇（Cluster）标记。*

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
