/**
 * @file pcl_process_node.cpp
 * @brief RGB-D 点云处理节点：滤波、地面分割、聚类、特征提取与可视化
 *
 * 支持双输入源对比：
 *   - /camera/depth/points    (Gazebo 深度相机原生点云)
 *   - /pointcloud_output      (get_pointcloud.py 反投影生成的点云)
 *
 * 处理流程：
 *   1. 直通滤波（Z 范围限制）
 *   2. 体素滤波降采样
 *   3. RANSAC 平面分割提取地面
 *   4. 欧氏聚类分割目标
 *   5. 对每个聚类提取特征：中心点、尺寸、体积、方向
 *   6. 发布 MarkerArray 包围盒与标签（RViz 可视化）
 *   7. 发布 PoseArray /object_clusters（供识别/跟踪模块订阅）
 *
 * 发布话题：
 *   /processed_cloud      降采样后的纯净点云
 *   /ground_plane         RANSAC 提取的地面（绿色）
 *   /cluster_cloud        聚类分割后的彩色点云
 *   /object_markers       MarkerArray（包围盒 + 中心点 + 标签）
 *   /object_clusters      PoseArray（每个目标的 3D 中心位姿）
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

ros::Publisher pub_processed;
ros::Publisher pub_ground;
ros::Publisher pub_clusters;
ros::Publisher pub_markers;
ros::Publisher pub_object_poses;

// 参数
double voxel_leaf_size = 0.02;
double pass_z_min = 0.05;
double pass_z_max = 3.0;
double seg_distance_thresh = 0.05;
double cluster_tolerance = 0.05;
int    cluster_min_size = 50;
int    cluster_max_size = 50000;
std::string input_topic = "/camera/depth/points";

// 聚类颜色调色板（RGB）
const std::vector<std::array<uint8_t, 3>> CLUSTER_COLORS = {
  {255, 0, 0},    // 红
  {0, 0, 255},    // 蓝
  {255, 255, 0},  // 黄
  {255, 0, 255},  // 紫
  {0, 255, 255},  // 青
  {255, 128, 0},  // 橙
  {0, 255, 0},    // 绿
  {128, 0, 255}   // 深紫
};

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_msg)
{
  // 1. ROS -> PCL
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*input_msg, *cloud);
  if (cloud->empty()) {
    ROS_WARN_THROTTLE(5.0, "Received empty point cloud from %s", input_msg->header.frame_id.c_str());
    return;
  }

  // 2. 直通滤波（Z 轴范围）
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_z(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(pass_z_min, pass_z_max);
  pass.filter(*cloud_z);

  // 3. 体素滤波降采样
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
  voxel.setInputCloud(cloud_z);
  voxel.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  voxel.filter(*cloud_voxel);

  // 4. RANSAC 地面分割
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(seg_distance_thresh);
  seg.setInputCloud(cloud_voxel);
  seg.segment(*inliers, *coeffs);

  // 5. 提取地面
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (!inliers->indices.empty()) {
    pcl::ExtractIndices<pcl::PointXYZRGB> ex;
    ex.setInputCloud(cloud_voxel);
    ex.setIndices(inliers);
    ex.filter(*cloud_ground);
    for (auto& pt : cloud_ground->points) { pt.r = 0; pt.g = 255; pt.b = 0; }
  }

  // 6. 提取非地面（障碍物）
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_obs(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (!inliers->indices.empty()) {
    pcl::ExtractIndices<pcl::PointXYZRGB> ex;
    ex.setInputCloud(cloud_voxel);
    ex.setIndices(inliers);
    ex.setNegative(true);
    ex.filter(*cloud_obs);
  } else {
    cloud_obs = cloud_voxel;
  }

  // 7. 欧氏聚类
  visualization_msgs::MarkerArray marker_array;
  geometry_msgs::PoseArray pose_array;
  pose_array.header = input_msg->header;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clustered(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (!cloud_obs->empty()) {
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud_obs);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(cluster_min_size);
    ec.setMaxClusterSize(cluster_max_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_obs);
    ec.extract(cluster_indices);

    int marker_id = 0;
    for (size_t i = 0; i < cluster_indices.size(); ++i) {
      const auto& indices = cluster_indices[i];
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      for (int idx : indices.indices) cluster_cloud->points.push_back(cloud_obs->points[idx]);
      cluster_cloud->width = cluster_cloud->points.size();
      cluster_cloud->height = 1;
      cluster_cloud->is_dense = true;

      const auto& c = CLUSTER_COLORS[i % CLUSTER_COLORS.size()];

      // 7.1 特征提取：AABB 包围盒 + 质心
      pcl::PointXYZRGB min_pt, max_pt;
      pcl::getMinMax3D(*cluster_cloud, min_pt, max_pt);
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cluster_cloud, centroid);

      float cx = centroid[0];
      float cy = centroid[1];
      float cz = centroid[2];
      float sx = max_pt.x - min_pt.x;
      float sy = max_pt.y - min_pt.y;
      float sz = max_pt.z - min_pt.z;
      float volume = sx * sy * sz;

      ROS_INFO_THROTTLE(1.0,
        "Object[%zu] pts=%zu center=(%.2f,%.2f,%.2f) size=(%.2f,%.2f,%.2f) vol=%.3f",
        i, cluster_cloud->points.size(), cx, cy, cz, sx, sy, sz, volume);

      // 7.2 填充彩色聚类点云（用于 /cluster_cloud 可视化）
      for (auto& pt : cluster_cloud->points) {
        pt.r = c[0]; pt.g = c[1]; pt.b = c[2];
        cloud_clustered->points.push_back(pt);
      }

      // 7.3 构建 Marker：包围盒（CUBE）
      visualization_msgs::Marker box_marker;
      box_marker.header = input_msg->header;
      box_marker.ns = "object_boxes";
      box_marker.id = marker_id++;
      box_marker.type = visualization_msgs::Marker::CUBE;
      box_marker.action = visualization_msgs::Marker::ADD;
      box_marker.pose.position.x = cx;
      box_marker.pose.position.y = cy;
      box_marker.pose.position.z = cz;
      box_marker.pose.orientation.w = 1.0;
      box_marker.scale.x = sx;
      box_marker.scale.y = sy;
      box_marker.scale.z = sz;
      box_marker.color.r = c[0] / 255.0;
      box_marker.color.g = c[1] / 255.0;
      box_marker.color.b = c[2] / 255.0;
      box_marker.color.a = 0.25;  // 半透明
      box_marker.lifetime = ros::Duration(0.5);
      marker_array.markers.push_back(box_marker);

      // 7.4 构建 Marker：中心点（SPHERE）
      visualization_msgs::Marker center_marker;
      center_marker.header = input_msg->header;
      center_marker.ns = "object_centers";
      center_marker.id = marker_id++;
      center_marker.type = visualization_msgs::Marker::SPHERE;
      center_marker.action = visualization_msgs::Marker::ADD;
      center_marker.pose.position.x = cx;
      center_marker.pose.position.y = cy;
      center_marker.pose.position.z = cz;
      center_marker.pose.orientation.w = 1.0;
      center_marker.scale.x = 0.08;
      center_marker.scale.y = 0.08;
      center_marker.scale.z = 0.08;
      center_marker.color.r = c[0] / 255.0;
      center_marker.color.g = c[1] / 255.0;
      center_marker.color.b = c[2] / 255.0;
      center_marker.color.a = 1.0;
      center_marker.lifetime = ros::Duration(0.5);
      marker_array.markers.push_back(center_marker);

      // 7.5 构建 Marker：标签文字
      visualization_msgs::Marker text_marker;
      text_marker.header = input_msg->header;
      text_marker.ns = "object_labels";
      text_marker.id = marker_id++;
      text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::Marker::ADD;
      text_marker.pose.position.x = cx;
      text_marker.pose.position.y = cy;
      text_marker.pose.position.z = max_pt.z + 0.1;
      text_marker.pose.orientation.w = 1.0;
      text_marker.scale.z = 0.1;  // 字体大小
      char label[128];
      snprintf(label, sizeof(label), "Obj[%zu]\npts:%zu\nvol:%.3f",
               i, cluster_cloud->points.size(), volume);
      text_marker.text = label;
      text_marker.color.r = 1.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 1.0;
      text_marker.color.a = 1.0;
      text_marker.lifetime = ros::Duration(0.5);
      marker_array.markers.push_back(text_marker);

      // 7.6 填充 PoseArray（供识别/跟踪模块订阅）
      geometry_msgs::Pose pose;
      pose.position.x = cx;
      pose.position.y = cy;
      pose.position.z = cz;
      pose.orientation.w = 1.0;
      pose_array.poses.push_back(pose);
    }

    cloud_clustered->width = cloud_clustered->points.size();
    cloud_clustered->height = 1;
    cloud_clustered->is_dense = true;
  }

  // 8. 发布所有话题
  sensor_msgs::PointCloud2 out_processed;
  pcl::toROSMsg(*cloud_voxel, out_processed);
  out_processed.header = input_msg->header;
  pub_processed.publish(out_processed);

  if (!cloud_ground->empty()) {
    sensor_msgs::PointCloud2 out_ground;
    pcl::toROSMsg(*cloud_ground, out_ground);
    out_ground.header = input_msg->header;
    pub_ground.publish(out_ground);
  }

  if (!cloud_clustered->empty()) {
    sensor_msgs::PointCloud2 out_clusters;
    pcl::toROSMsg(*cloud_clustered, out_clusters);
    out_clusters.header = input_msg->header;
    pub_clusters.publish(out_clusters);
  }

  if (!marker_array.markers.empty()) {
    pub_markers.publish(marker_array);
  }

  if (!pose_array.poses.empty()) {
    pub_object_poses.publish(pose_array);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_process_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // 读取参数
  pnh.param<std::string>("input_topic", input_topic, "/camera/depth/points");
  pnh.param<double>("voxel_leaf_size", voxel_leaf_size, 0.02);
  pnh.param<double>("pass_z_min", pass_z_min, 0.05);
  pnh.param<double>("pass_z_max", pass_z_max, 3.0);
  pnh.param<double>("seg_distance_thresh", seg_distance_thresh, 0.05);
  pnh.param<double>("cluster_tolerance", cluster_tolerance, 0.05);
  pnh.param<int>("cluster_min_size", cluster_min_size, 50);
  pnh.param<int>("cluster_max_size", cluster_max_size, 50000);

  pub_processed = nh.advertise<sensor_msgs::PointCloud2>("/processed_cloud", 1);
  pub_ground    = nh.advertise<sensor_msgs::PointCloud2>("/ground_plane", 1);
  pub_clusters  = nh.advertise<sensor_msgs::PointCloud2>("/cluster_cloud", 1);
  pub_markers   = nh.advertise<visualization_msgs::MarkerArray>("/object_markers", 1);
  pub_object_poses = nh.advertise<geometry_msgs::PoseArray>("/object_clusters", 1);

  ros::Subscriber sub = nh.subscribe(input_topic, 1, cloudCallback);

  ROS_INFO("PCL process node started.");
  ROS_INFO("Input topic: %s", input_topic.c_str());
  ROS_INFO("Parameters: voxel=%.3f, z_range=[%.2f, %.2f], plane_thresh=%.3f",
           voxel_leaf_size, pass_z_min, pass_z_max, seg_distance_thresh);

  ros::spin();
  return 0;
}
