/**
 * @file pcl_process_node.cpp
 * @brief RGB-D 点云处理节点：滤波、地面分割、聚类
 *
 * 订阅 /camera/depth/points，发布：
 *   - /processed_cloud    降采样 + 离群点移除后的点云
 *   - /ground_plane       RANSAC 提取的地面平面点云
 *   - /cluster_cloud      聚类后的彩色点云
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

ros::Publisher pub_processed;
ros::Publisher pub_ground;
ros::Publisher pub_clusters;

// 参数
double voxel_leaf_size = 0.02;      // 体素滤波叶子尺寸 (m)
double pass_z_min = 0.05;           // 直通滤波 Z 最小值 (m)
double pass_z_max = 3.0;            // 直通滤波 Z 最大值 (m)
double seg_distance_thresh = 0.05;  // RANSAC 平面距离阈值 (m)
double cluster_tolerance = 0.05;    // 欧氏聚类距离阈值 (m)
int    cluster_min_size = 50;       // 聚类最小点数
int    cluster_max_size = 50000;    // 聚类最大点数

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_msg)
{
  // 1. ROS PointCloud2 -> PCL PointCloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*input_msg, *cloud);

  if (cloud->empty()) {
    ROS_WARN_THROTTLE(5.0, "Received empty point cloud");
    return;
  }

  // 2. 直通滤波：限制 Z 范围（去除摄像头后方和过远的点）
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(pass_z_min, pass_z_max);
  pass.filter(*cloud_filtered);

  // 3. 体素滤波：降采样，减少计算量
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
  voxel.setInputCloud(cloud_filtered);
  voxel.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  voxel.filter(*cloud_voxel);

  // 4. RANSAC 平面分割：提取地面
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(seg_distance_thresh);
  seg.setInputCloud(cloud_voxel);
  seg.segment(*inliers, *coefficients);

  // 5. 提取地面点云
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (!inliers->indices.empty()) {
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud_voxel);
    extract.setIndices(inliers);
    extract.filter(*cloud_ground);

    // 将地面染成绿色，便于区分
    for (auto& pt : cloud_ground->points) {
      pt.r = 0;
      pt.g = 255;
      pt.b = 0;
    }
  }

  // 6. 提取非地面点云（障碍物）
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_obstacles(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (!inliers->indices.empty()) {
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud_voxel);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_obstacles);
  } else {
    cloud_obstacles = cloud_voxel;
  }

  // 7. 欧氏聚类：对非地面点进行分割
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clusters(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (!cloud_obstacles->empty()) {
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud_obstacles);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(cluster_min_size);
    ec.setMaxClusterSize(cluster_max_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_obstacles);
    ec.extract(cluster_indices);

    // 为每个聚类分配不同颜色
    std::vector<std::array<uint8_t, 3>> colors = {
      {255, 0, 0}, {0, 0, 255}, {255, 255, 0},
      {255, 0, 255}, {0, 255, 255}, {255, 128, 0}
    };

    size_t color_idx = 0;
    for (const auto& indices : cluster_indices) {
      for (int idx : indices.indices) {
        pcl::PointXYZRGB pt = cloud_obstacles->points[idx];
        pt.r = colors[color_idx % colors.size()][0];
        pt.g = colors[color_idx % colors.size()][1];
        pt.b = colors[color_idx % colors.size()][2];
        cloud_clusters->points.push_back(pt);
      }
      color_idx++;
    }
    cloud_clusters->width = cloud_clusters->points.size();
    cloud_clusters->height = 1;
    cloud_clusters->is_dense = true;
  }

  // 8. 发布处理后的点云
  sensor_msgs::PointCloud2 output_processed;
  pcl::toROSMsg(*cloud_voxel, output_processed);
  output_processed.header = input_msg->header;
  pub_processed.publish(output_processed);

  if (!cloud_ground->empty()) {
    sensor_msgs::PointCloud2 output_ground;
    pcl::toROSMsg(*cloud_ground, output_ground);
    output_ground.header = input_msg->header;
    pub_ground.publish(output_ground);
  }

  if (!cloud_clusters->empty()) {
    sensor_msgs::PointCloud2 output_clusters;
    pcl::toROSMsg(*cloud_clusters, output_clusters);
    output_clusters.header = input_msg->header;
    pub_clusters.publish(output_clusters);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_process_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // 读取参数
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

  ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, cloudCallback);

  ROS_INFO("PCL process node started.");
  ROS_INFO("Parameters: voxel=%.3f, z_range=[%.2f, %.2f], plane_thresh=%.3f",
           voxel_leaf_size, pass_z_min, pass_z_max, seg_distance_thresh);

  ros::spin();
  return 0;
}
