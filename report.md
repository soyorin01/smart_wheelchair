# 基于 ROS 的智能轮椅仿真控制系统设计

> **说明**：本文档为课程设计报告正文，可直接复制到 Word 中排版。Word 格式要求：宋体、小四、1.5 倍行距；首行缩进 2 字符，段后空一行。标题层级请使用 Word 的样式功能统一设置。

---

## 一、设计背景与目标

### （一）设计背景

随着人口老龄化、慢性病和意外伤害等情况的持续增长，步行能力衰退甚至丧失的群体规模日益庞大。传统轮椅存在功能单一、操控困难、智能化程度低等不足：手动轮椅依赖使用者或他人推行，增加了生理负担；电动轮椅虽减轻了体力消耗，但缺乏环境感知与自主避障能力；附加了智能功能的轮椅往往操作复杂、速度设置不够人性化，对认知能力较弱的老年人不够友好。因此，开发一款具备语音交互、自主导航与智能避障能力的多功能智能轮椅，对于提升残障人士与老年人的生活质量具有重要的现实意义。

机器人仿真控制是智能轮椅研发的关键环节。在工业自动化与机器人教学领域，基于 ROS（Robot Operating System）的仿真平台能够显著降低硬件调试成本、缩短开发周期。ROS 提供了标准化的节点通信机制、丰富的传感器插件与导航算法库；Gazebo 物理引擎可高精度复现机器人在真实环境中的运动学与动力学特性；RViz 则支持对传感器数据、TF 变换与路径规划结果的可视化监控。借助 ROS + Gazebo 的联合仿真环境，可以在投入实体样机之前，完成模型验证、控制算法测试与导航系统调优。

### （二）设计目标

本设计基于 ROS Noetic 平台与 Gazebo 11 仿真环境，以智能轮椅为对象，逐步完成以下 5 个基础模块与 1 个选做模块：

- **模块 1**：ROS 开发环境配置与图像采集。搭建标准 catkin 工作空间，部署摄像头插件，实现图像采集、发布、实时显示及至少两种图像预处理（灰度化、高斯滤波等）。
- **模块 2**：目标识别与特征提取。基于 Hough 变换、ORB 等方法实现目标检测，编写 ROS 识别节点并在 RViz 中可视化标注。
- **模块 3**：目标识别与追踪。基于 OpenCV Tracker 实现不少于两种跟踪算法的对比实验，完成识别与跟踪联动，实时输出目标坐标话题。
- **模块 4**：SLAM 地图构建与 Navigation 自主导航。采用 Gmapping 完成环境 SLAM 建图，配置 Navigation 导航包，实现自主导航、避障与定点到达功能。
- **模块 5**：系统整合与工程化优化。编写总 launch 文件实现全系统一键启动，完成模块间话题与服务通信联调，保证系统连续稳定运行。
- **选做模块**：三维点云处理（加分项）。搭建 PCL + ROS 环境，实现激光雷达点云数据的采集、滤波与分割，并在 RViz 中可视化。

---

## 二、功能设计与实现

### （一）总体结构方案

本系统采用典型的 ROS 分布式节点架构，以智能轮椅 URDF/Xacro 模型为核心，整合传感器仿真、运动控制、视觉处理与导航规划四大子系统。系统总体结构如图 1 所示（节点计算图 / 系统功能示意图见附录）。

在物理层，智能轮椅模型由底盘（base_link）、驱动轮（left/right_wheel_link）、万向轮（front/rear_caster_link）、靠背、扶手、脚踏板、激光雷达（lidar_link）与摄像头（camera_link）等部件组成。各连杆通过固定关节（fixed）或连续旋转关节（continuous）连接，质量、惯性张量与摩擦系数均依据实际物理参数设定。

在仿真层，Gazebo 通过 `libgazebo_ros_diff_drive.so` 插件实现差速驱动控制，订阅 `/cmd_vel` 话题并发布 `/odom` 里程计信息；通过 `libgazebo_ros_laser.so` 插件模拟二维激光雷达，发布 `/scan` 话题；通过 `libgazebo_ros_camera.so` 插件模拟 RGB 摄像头，发布 `/image_raw` 与 `/camera_info` 话题。

在应用层，键盘遥控节点（`teleop_keyboard.py`）将键盘输入转换为 `geometry_msgs/Twist` 速度指令；图像处理节点订阅摄像头原始图像并发布预处理结果；目标识别与跟踪节点输出目标像素坐标；导航节点订阅激光雷达与里程计数据，完成地图构建与路径规划。

各模块之间通过 ROS 话题（Topic）进行异步数据交换，关键话题名称与消息类型如表 1 所示。

**表 1 系统核心话题接口**

| 话题名称 | 消息类型 | 发布者 | 功能说明 |
|---------|---------|--------|---------|
| `/cmd_vel` | `geometry_msgs/Twist` | 键盘遥控节点 | 轮椅线速度与角速度指令 |
| `/odom` | `nav_msgs/Odometry` | 差速驱动插件 | 里程计位姿与速度 |
| `/scan` | `sensor_msgs/LaserScan` | 激光雷达插件 | 二维环境距离扫描 |
| `/image_raw` | `sensor_msgs/Image` | 摄像头插件 | 原始彩色图像 |
| `/camera_info` | `sensor_msgs/CameraInfo` | 摄像头插件 | 摄像头内参与畸变系数 |

### （二）具体模块的设计与实现

#### 1. 模块 1：ROS 开发环境配置与图像采集

【待补充】本模块待完成以下内容：

- （1）在 `smart_wheelchair` 功能包中集成 `usb_cam` 或 Gazebo `libgazebo_ros_camera.so` 插件，确保 `/image_raw` 话题正常输出。
- （2）编写图像预处理节点，实现灰度化（`cv::cvtColor`）与高斯滤波（`cv::GaussianBlur`），封装为独立 ROS 节点 `image_preprocess_node.py`，订阅 `/image_raw`，发布 `/image_gray` 与 `/image_blur`。
- （3）使用 `rqt_image_view` 或 RViz 的 Image 插件验证图像流，确保话题通信正常。

实验验证要点：
- 参数配置：摄像头分辨率 640×480，帧率 30 Hz，水平视场角 1.047 rad。
- RViz 展示：添加 Image 显示插件，分别订阅 `/image_raw`、`/image_gray`、`/image_blur` 进行并排对比。
- Gazebo 运动：在轮椅运动过程中观察图像是否出现抖动或丢帧，验证摄像头插件的 `updateRate` 与 `clip` 参数合理性。

#### 2. 模块 2：目标识别与特征提取

【待补充】本模块待完成以下内容：

- （1）选定实验目标（如红色立方体、圆形标志物或走廊特征角点）。
- （2）基于 Hough 圆变换或 ORB 特征检测编写 `object_detection_node.py`，在图像中标注检测框或关键点。
- （3）将检测结果（像素坐标、目标类别）封装为自定义消息或 `geometry_msgs/PointStamped`，发布到 `/detected_object` 话题。
- （4）在 RViz 中通过 Marker 插件可视化检测框，分析光照变化、拍摄角度与复杂背景对识别精度的影响，并通过调整高斯滤波核大小、自适应阈值等策略优化。

实验验证要点：
- 参数配置：Hough 变换的累加器阈值、最小/最大半径；ORB 的特征点数与金字塔层数。
- RViz 展示：添加 MarkerArray 显示识别框，对比不同参数下的漏检率与误检率。
- Gazebo 运动：在轮椅靠近目标过程中，观察识别框的稳定性与尺度变化。

#### 3. 模块 3：目标识别与追踪

【待补充】本模块待完成以下内容：

- （1）基于 OpenCV 的 KCF 与 CSRT  tracker 实现两种跟踪算法节点。
- （2）设计识别-跟踪联动逻辑：当目标首次出现时，由模块 2 的检测节点给出初始包围盒，随后触发跟踪节点持续锁定目标；若目标丢失超过阈值，则重新进入检测模式。
- （3）实时输出目标中心坐标到 `/target_position` 话题，并在 RViz 中绘制跟踪轨迹（`nav_msgs/Path`）。
- （4）对比 KCF（速度快、轻量）与 CSRT（精度高、抗遮挡能力强）在 Gazebo 仿真环境中的帧率、鲁棒性与重捕能力，记录性能数据并完成参数调优。

实验验证要点：
- 参数配置：KCF 的 Hog/Color 特征开关；CSRT 的模板更新率与正则化系数。
- RViz 展示：叠加显示跟踪框与历史轨迹路径。
- Gazebo 运动：控制轮椅做匀速直线运动与原地旋转，观察跟踪算法在摄像机运动状态下的表现。

#### 4. 模块 4：SLAM 地图构建与 Navigation 自主导航

本项目在实现稳定 SLAM 建图与导航的过程中，克服了多个物理与算法层面的技术难题。以下是核心经验与技术教训总结：

##### （1）物理仿真环境的调优（教训与经验）
- **万向轮摩擦力问题**：初期轮椅在转向时里程计（Odom）误差极大。经分析发现，万向轮（Caster）的摩擦系数不当会导致驱动轮打滑。
  - **解决方案**：将万向轮的摩擦系数 `mu1`、`mu2` 设为 **0.0**，并将关节类型改为 `continuous`，实现了“零摩擦”滑动，显著提升了里程计的线性度。
- **雷达自遮挡干扰**：轮椅的高靠背挡住了雷达后方的视场，导致 Gmapping 误认为背后始终有墙，引起坐标系剧烈跳动。
  - **解决方案**：除了物理上调高安装位置，在软件层面将雷达扫描角度缩减至 **270度**（屏蔽后方 90度盲区），彻底消除了自遮挡噪点。

##### （2）坐标变换（TF）冲突的规避
- **教训**：严禁多个节点同时发布 `odom -> base_footprint` 的变换（即 TF Fighting 现象）。
- **最佳实践**：在建图模式下，由 Gazebo 插件直接负责里程计 TF；在使用 AMCL 导航时，通过 `publish_tf: false` 规避冲突。确保 TF 树为唯一的单向链条（`map -> odom -> base_footprint`），解决了机器人在 RViz 中“反复横跳”的难题。

##### （3）导航算法参数优化
- **AMCL 快速响应**：针对轮椅低速运行的特点，将角度更新阈值 `update_min_a` 从 `0.5` 降至 **`0.1`**，使轮椅在微小转向时也能立即校正位姿。
- **里程计模型信任度**：在导航参数中压低了旋转误差系数（`odom_alpha1` 等），强迫定位系统更多地相信激光匹配结果，从而有效抑制了由于转向打滑引起的位姿漂移。

实验验证要点：
- 参数配置：`gmapping` 的粒子滤波参数；`costmap_2d` 的膨胀半径与观测源配置；`move_base` 的最大线速度、角速度与加速度约束。
- RViz 展示：显示地图（Map）、粒子云（PoseArray）、全局路径（Path，红色）、局部路径（Path，绿色）、代价地图（Costmap）。
- Gazebo 运动：在仿真环境中设置起点与终点，验证轮椅能否平稳避障并准确到达目标位姿。

#### 5. 模块 5：系统整合与工程化优化

本模块已完成部分基础工作，并将在后续迭代中持续完善。

**（1）一键启动架构**

目前已编写以下 launch 文件，实现分层启动：

- `display.launch`：在 RViz 中加载并显示模型。启动 `robot_state_publisher`、`joint_state_publisher_gui` 与 RViz。
- `gazebo.launch`：在 Gazebo 空世界中加载并仿真模型。加载 Xacro 到参数服务器，启动 Gazebo，调用 `spawn_model` 在 `(0, 0, 0.1)` 处生成机器人。
- `gazebo_teleop.launch`：在 `gazebo.launch` 基础上自动启动键盘遥控节点 `teleop_keyboard.py`，实现"一键启动仿真 + 控制"。

后续将编写 `system_bringup.launch`，集成摄像头、图像处理、目标识别、跟踪、SLAM 与导航节点，实现全系统单指令启动。

**（2）节点代码结构**

已实现的源码清单：

- `urdf/wheelchair.xacro`：轮椅完整 Xacro 描述，含连杆、关节、材料、Gazebo 插件与物理属性。
- `scripts/teleop_keyboard.py`：键盘遥控节点，支持 WASD/方向键控制、速度档位调节与紧急停止。
- `launch/display.launch`、`gazebo.launch`、`gazebo_teleop.launch`：分层启动配置。
- `config/wheelchair.rviz`：预配置 RViz 布局，可直接查看模型与 TF 树。

**（3）话题通信联调**

差速驱动插件、激光雷达插件与摄像头插件的话题接口已验证通畅。键盘遥控节点发布的 `/cmd_vel` 可被差速插件正确接收，并驱动轮椅在 Gazebo 中运动。`/odom`、`/scan`、`/image_raw` 等话题数据可在 RViz 中正常可视化。

### （三）测试验证

#### 1. 轮椅模型加载测试

- **参数配置**：通过 `roslaunch smart_wheelchair display.launch` 加载模型。`robot_description` 参数由 `xacro` 命令解析生成。
- **RViz 展示**：在 RViz 中确认所有 link（base_link、wheel_link、lidar_link、camera_link 等）的 TF 变换正确，无断裂或错位。
- **Gazebo 运动**：通过 `roslaunch smart_wheelchair gazebo_teleop.launch` 启动仿真。在终端中按住 `w` 键，观察 Gazebo 中轮椅是否平稳前进，轮子是否沿地面滚动而非空转。

#### 2. 差速驱动与键盘遥控测试

- **参数配置**：差速插件的 `wheelSeparation=0.68`、`wheelDiameter=0.30`、`wheelTorque=30`，更新频率 50 Hz。
- **RViz 展示**：在 RViz 中添加 Odometry 显示，订阅 `/odom`，观察轮椅运动时里程计轨迹是否与 Gazebo 中实际位姿一致。
- **Gazebo 运动**：测试前进、后退、左转、右转与原地旋转，记录不同线速度（0.1 ~ 0.5 m/s）下的运动平稳性。验证发现，当 `wheelTorque` 过低（默认值 5.0）时，轮子在原地打滑；调整为 30 后，驱动力充足，底盘响应灵敏。

#### 3. 传感器数据验证

- **参数配置**：激光雷达最小测距 0.10 m，最大测距 10.0 m，采样数 360；摄像头分辨率 640×480，裁剪面 0.05 ~ 100 m。
- **RViz 展示**：添加 LaserScan 显示插件订阅 `/scan`，确认扫描点云与 Gazebo 环境中的障碍物吻合；添加 Image 插件订阅 `/image_raw`，确认画面正常无畸变。
- **Gazebo 运动**：在轮椅运动过程中，观察激光雷达点云是否实时更新，是否存在因车体晃动导致的扫描错位。

---

## 三、问题与解决方法

| 遇到的问题 | 原因分析 | 解决方法 |
|-----------|---------|---------|
| Gazebo 中轮子飞速转动但轮椅底盘不动 | URDF 根 link `base_footprint` 缺少 `<inertial>` 标签，导致 Gazebo 物理引擎可能将模型视为 static，底盘被隐形锁定；同时差速插件参数 `<torque>` 名称错误，实际使用了默认值 5.0，驱动力偏弱 | ① 为 `base_footprint` 添加 `<inertial>`（质量 0.001 kg，极小惯性张量），使模型被正确识别为动态刚体；② 将 `<torque>` 更正为 `<wheelTorque>`，确保 30 N·m 的电机力矩生效；③ 清理已废弃的 `<legacyMode>` 与 `<alwaysOn>` 参数 |
| 键盘控制节点无法在 roslaunch 中自动启动 | 新建的 Python 脚本未在 `CMakeLists.txt` 中声明安装，导致 `rosrun` 找不到可执行文件 | 在 `CMakeLists.txt` 中取消注释 `catkin_install_python`，将 `scripts/teleop_keyboard.py` 安装到 `${CATKIN_PACKAGE_BIN_DESTINATION}` |
| 差速插件参数未被正确解析 | `gazebo_ros_diff_drive` 在 ROS Noetic 中已不再识别部分旧版参数名（如 `publishOdometryTf` 应为 `publishOdomTF`），虽然部分参数有默认值，但命名不统一存在隐患 | 对照 `gazebo_ros_diff_drive` 源码核对所有参数名，统一使用插件支持的驼峰命名（`wheelTorque`、`publishOdomTF`、`wheelSeparation` 等） |
| 深度图反投影点云出现在 RViz 正上方 | URDF 中 `camera_link` 的坐标轴与相机光学坐标系不一致。标准针孔反投影公式输出的是光学坐标系（Z 轴为光轴向前），但 `camera_link` 的坐标轴和 `base_link` 对齐：X=前、Y=左、Z=上。若直接将 depth 值赋给 `z`，点云会被画到 `camera_link` 的正上方 | 建立光学坐标系 → `camera_link` 的显式映射：`depth(Z_opt) → X_link`（前方），`(u-cx)*depth/fx (X_opt) → -Y_link`（右方），`(v-cy)*depth/fy (Y_opt) → -Z_link`（下方）。在 `get_pointcloud.py` 的函数注释中详细记录此映射关系，防止后续复用踩坑 |
| 【待补充】 | 【待补充】 | 【待补充】 |

---

## 四、创新点说明

1. **精细化 URDF/Xacro 建模**：不同于简单的方块车模型，本设计对轮椅进行了高保真建模，包含底盘框架、座垫、靠背、扶手、脚踏板、驱动轮、万向轮、激光雷达立柱与摄像头支架等 10 余个连杆。每个连杆均配置了独立的 `<visual>`、`<collision>` 与 `<inertial>`，并依据实际尺寸与质量分布计算惯性张量，确保 Gazebo 物理仿真的真实性。

2. **参数化 Gazebo 插件配置**：通过 Xacro 宏定义与属性变量（`wh_r`、`wh_y`、`ca_r` 等），实现轮子半径、轮距、万向轮位置等关键参数的一键修改。差速驱动、激光雷达与摄像头插件的话题名、更新频率、摩擦系数均通过 Xacro 集中管理，便于后续调参与复用。

3. **自研键盘遥控节点**：不依赖外部 `teleop_twist_keyboard` 包，自主编写了带速度档位调节（`u/m` 增减线速度、`i/n` 增减角速度）、方向键兼容、紧急停止（空格）与实时状态回显的遥控节点，代码轻量、无额外依赖，可直接集成到一键启动流程中。

4. 【待补充：后续模块的创新点，如自定义跟踪策略、改进的导航参数等】

---

## 五、总结与展望

### （一）总结

通过本次课程设计，完成了智能轮椅的 URDF/Xacro 建模、Gazebo 仿真环境搭建与键盘遥控运动控制。掌握了以下核心技术：

- **URDF/Xacro 建模**：学会了使用 Xacro 宏定义复用对称结构（左右轮、扶手、万向轮），理解了 `<visual>`、`<collision>`、`<inertial>` 三要素对仿真真实性的影响，以及 Gazebo 插件与 URDF 的集成方式。
- **ROS 通信机制**：熟悉了 Topic 发布/订阅模式，理解了 `geometry_msgs/Twist`、`nav_msgs/Odometry`、`sensor_msgs/LaserScan` 等标准消息类型的用途。
- **仿真验证与调试**：通过反复测试发现了 Gazebo 根 link 必须包含 inertial、插件参数名必须严格匹配等关键细节，积累了 ROS + Gazebo 联合调试的经验。

目前已实现的功能包括：高保真轮椅模型、差速驱动仿真、激光雷达与摄像头数据发布、键盘遥控控制、RViz 可视化监控。后续将继续完成图像处理、目标识别跟踪、SLAM 建图与自主导航模块。

### （二）展望

未来可在以下方向深入拓展：

1. **硬件迁移**：将 Gazebo 仿真中验证的控制算法与导航参数迁移到真实智能轮椅平台，通过 ROS 串口节点驱动实际电机与传感器。
2. **AI 算法增强**：引入深度学习目标检测（YOLO、SSD）替代传统图像算法，提升复杂场景下的识别准确率；结合强化学习优化局部路径规划策略。
3. **人机交互升级**：集成语音交互模块（如科大讯飞 SDK 或 ROS `pocketsphinx`），实现语音控制轮椅前进、后退、停车等指令，降低老年人操作门槛。
4. **多机协同**：在仿真环境中部署多台轮椅，研究多机器人协同导航与动态避碰算法。

---

## 六、参考文献

[1] 王晶. 智能小车运动控制技术的研究[D]. 武汉理工大学.

[2] 陈懂. 智能小车运动控制系统的研究与实现[D]. 东南大学.

[3] M. J Potasek and G. P. Agrawal. Single-Chip microcomputer data[J]. IEEE Electron, 1995, Vol. 31, No. 1: 183-189.

[4] Zhu W., Ruan H. Design and research of solar photovoltaic power generation controller based on 89C51 microcontroller[J]. Advanced Materials Research, 2011, 345: 66-69.

[5] ROS Wiki. URDF Tutorials[EB/OL]. http://wiki.ros.org/urdf/Tutorials.

[6] ROS Wiki. gazebo_ros_diff_drive[EB/OL]. http://wiki.ros.org/gazebo_ros_pkgs.

[7] OpenCV Documentation. Object Tracking[EB/OL]. https://docs.opencv.org/4.x/d9/df8/group__tracking.html.

---

## 七、其他附件说明

- **源代码压缩包**：`学号_姓名_智能轮椅仿真控制.zip`
  - 包含 `smart_wheelchair` 功能包全部源码：`urdf/`、`launch/`、`scripts/`、`config/`、`CMakeLists.txt`、`package.xml`。
- **演示视频**：包含 RViz 模型展示、TF 树验证、Gazebo 键盘遥控运动、传感器数据可视化。
- **补充材料**：Xacro 建模参数表、差速插件配置说明、问题排查记录。

---

> **附录提示**：请在 Word 版本中插入以下图片：
> - 图 1：系统功能示意图 / rqt_graph 节点计算图（后续模块整合后生成）。
> - 图 2：RViz 模型展示截图。
> - 图 3：Gazebo 仿真环境截图（含轮椅与障碍物）。
> - 图 4：图像预处理对比图（模块 1 完成后补充）。
> - 图 5：目标识别与跟踪可视化截图（模块 2、3 完成后补充）。
> - 图 6：SLAM 地图与导航路径截图（模块 4 完成后补充）。
