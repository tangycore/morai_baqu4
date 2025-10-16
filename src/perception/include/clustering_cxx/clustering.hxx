#pragma once

#include <pcl/impl/point_types.hpp>
#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <vision_msgs/msg/bounding_box3_d.hpp>
#include <vision_msgs/msg/bounding_box3_d_array.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/PointIndices.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class ClusteringNode : public rclcpp::Node {
public:
  ClusteringNode();

  std::vector<pcl::PointIndices>
  makeClusterVector(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud);

  void lidarCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);
  void timerCallback();

private:
  rclcpp::Clock clock_;

  std::string lidar_topic_;

  uint32_t min_core_points_;
  uint32_t min_cluster_size_;
  uint32_t max_cluster_size_;
  double tolerance_;

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      visualization_pub_;
  rclcpp::Publisher<vision_msgs::msg::BoundingBox3DArray>::SharedPtr box3d_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};