#include <clustering_cxx/clustering.hxx>
#include <clustering_cxx/parameters.hxx>
#include <dbscan_kdtree/DBSCAN_precomp.h>
#include <omp.h>
#include <pcl/PointIndices.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <vision_msgs/msg/detail/bounding_box3_d__struct.hpp>
#include <visualization.h>

ClusteringNode::ClusteringNode() : Node("clustering_node") {
  this->declare_parameter(LIDAR_TOPIC, rclcpp::PARAMETER_STRING);
  this->declare_parameter(PARAM_MIN_CORE_POINTS, rclcpp::PARAMETER_INTEGER);
  this->declare_parameter(PARAM_MIN_CLUSTER_SIZE, rclcpp::PARAMETER_INTEGER);
  this->declare_parameter(PARAM_MAX_CLUSTER_SIZE, rclcpp::PARAMETER_INTEGER);
  this->declare_parameter(PARAM_TOLERANCE, rclcpp::PARAMETER_DOUBLE);

  this->get_parameter_or<std::string>(LIDAR_TOPIC, this->lidar_topic_,
                                      "/ouster/points");
  this->get_parameter_or<uint32_t>(PARAM_MIN_CORE_POINTS,
                                   this->min_core_points_, 3);
  this->get_parameter_or<uint32_t>(PARAM_MIN_CLUSTER_SIZE,
                                   this->min_cluster_size_, 3);
  this->get_parameter_or<uint32_t>(PARAM_MAX_CLUSTER_SIZE,
                                   this->max_cluster_size_, 10);
  this->get_parameter_or<double>(PARAM_TOLERANCE, this->tolerance_, 0.2);

  this->lidar_sub_.subscribe(this, this->lidar_topic_, rmw_qos_profile_sensor_data);
  this->lidar_sub_.registerCallback(&ClusteringNode::lidarCallback, this);

  this->lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/clustering/points", 10);
  this->visualization_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/clustering/visualization", 10);
  this->box3d_pub_ =
      this->create_publisher<vision_msgs::msg::BoundingBox3DArray>(
          "/clustering/result", 10);

  this->timer_ =
      this->create_wall_timer(std::chrono::milliseconds(200),
                              std::bind(&ClusteringNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(),
              "MIN_CORE_POINTS: %d "
              "MIN_CLUSTER_SIZE: %d "
              "MAX_CLUSTER_SIZE: %d "
              "TOLERANCE: %lf \n",
              this->min_core_points_, this->min_cluster_size_,
              this->max_cluster_size_, this->tolerance_);
}

std::vector<pcl::PointIndices> ClusteringNode::makeClusterVector(
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud) {
  std::vector<pcl::PointIndices> cluster_indices;
  DBSCANPrecompCluster<pcl::PointXYZ> dc;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(pclCloud);

  dc.setSearchMethod(tree);

  dc.setCorePointMinPts(this->min_core_points_);
  dc.setMinClusterSize(this->min_cluster_size_);
  dc.setMaxClusterSize(this->max_cluster_size_);
  dc.setClusterTolerance(this->tolerance_);
  dc.setInputCloud(pclCloud);
  dc.extract(cluster_indices);

  RCLCPP_INFO(this->get_logger(), "Size: %ld", cluster_indices.size());

  return cluster_indices;
}

void ClusteringNode::lidarCallback(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
  static rclcpp::Time time(0);
  if (time.nanoseconds() == 0) {
    time = this->clock_.now();
  }

  rclcpp::Time now = this->clock_.now();
  rclcpp::Duration duration = now - time;
  RCLCPP_INFO(this->get_logger(), "Duration: %lf", duration.seconds());

  pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloud, *pclCloud);

  for (uint32_t i = 0; i < pclCloud->points.size();) {
    auto const point = pclCloud->points[i];
    float distance = point.x * point.x + point.y * point.y + point.z * point.z;

    if (distance > 1000 || distance < 1 || point.z > -0.4) {
      pclCloud->points[i] = pclCloud->points[pclCloud->points.size() - 1];
      pclCloud->points.resize(pclCloud->points.size() - 1);
    } else {
      ++i;
    }
  }

  std::vector<pcl::PointIndices> clusters = makeClusterVector(pclCloud);
  std::vector<std::vector<std::pair<Eigen::Vector4f, Eigen::Vector4f>>>
      clustersPerThread(omp_get_max_threads());
  std::vector<vision_msgs::msg::BoundingBox3D> boxes;
  std::vector<std::vector<vision_msgs::msg::BoundingBox3D>> boxesPerThread(
      omp_get_max_threads());

#pragma omp parallel for
  for (auto cluster = clusters.begin(); cluster != clusters.end(); ++cluster) {
    if (cluster->indices.size() > this->min_cluster_size_) {
      Eigen::Vector4f min_pt, max_pt, mid_pt, size_pt;
      pcl::getMinMax3D(*pclCloud, cluster->indices, min_pt, max_pt);
      mid_pt = (min_pt + max_pt) / 2;
      size_pt = max_pt - min_pt;
      if (size_pt.x() < 0.1 || size_pt.y() < 0.1 || size_pt.z() < 0.1)
        continue;
      if (size_pt.x() > 1.0 || size_pt.y() > 1.0 || size_pt.z() > 1.0)
        continue;

      vision_msgs::msg::BoundingBox3D box;
      box.center.position.x = mid_pt.x();
      box.center.position.y = mid_pt.y();
      box.center.position.z = mid_pt.z();
      box.size.x = std::max(0.01f, size_pt.x());
      box.size.y = std::max(0.01f, size_pt.y());
      box.size.z = std::max(0.01f, size_pt.z());

      box.center.orientation.x = 0.0;
      box.center.orientation.y = 0.0;
      box.center.orientation.z = 0.0;
      box.center.orientation.w = 1.0;

      clustersPerThread[omp_get_thread_num()].push_back(std::make_pair(
          mid_pt, Eigen::Vector4f{0.3f, 0.3f, size_pt.z(), 1.0f}));
      boxesPerThread[omp_get_thread_num()].push_back(box);
    }
  }

  for (auto clusters = clustersPerThread.begin();
       clusters < clustersPerThread.end(); ++clusters) {
    visualizeAnyPoint(this->visualization_pub_, "visualization", cloud->header,
                      *clusters, {1, 0, 0, 1}, duration);
  }

  sensor_msgs::msg::PointCloud2::SharedPtr newCloud(
      new sensor_msgs::msg::PointCloud2);
  newCloud->header = cloud->header;
  pcl::toROSMsg(*pclCloud, *newCloud);

  vision_msgs::msg::BoundingBox3DArray box_array;
  box_array.header = cloud->header;
  for (const auto &thread_boxes : boxesPerThread) {
    box_array.boxes.insert(box_array.boxes.end(), thread_boxes.begin(),
                           thread_boxes.end());
  }

  this->lidar_pub_->publish(*newCloud);
  this->box3d_pub_->publish(box_array);

  time = now;
}

void ClusteringNode::timerCallback() {}