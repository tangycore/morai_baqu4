/**
 * LiDAR Clustering Node with Object Tracking
 * Added: Simple IOU-based Multi-Object Tracking for stable IDs
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vision_msgs/Detection3D.h>
#include <vision_msgs/Detection3DArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <Eigen/Dense>
#include <omp.h>

#include "dbscan_kdtree/DBSCAN_kdtree.h"
#include "lidar_clustering/simple_tracker.h"  // Tracker 추가

class LidarClusterNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  
  ros::Publisher cluster_cloud_pub_;
  ros::Publisher detection3d_pub_;
  ros::Publisher marker_pub_;
  
  ros::Subscriber lidar_sub_;
  
  boost::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  boost::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  std::string lidar_topic_;
  std::string output_frame_;
  std::string cluster_result_topic_;
  float cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;
  
  float max_distance_;
  float min_x_;
  float max_z_;
  float min_z_;
  float min_y_;
  float max_y_;
  
  ros::Time last_call_time_;
  
  // ✅ Tracker 추가
  SimpleTracker tracker_;

public:
  LidarClusterNode() : pnh_("~")
  {
    pnh_.param<std::string>("lidar_topic", lidar_topic_, "/front_lidar/velodyne_points");
    pnh_.param<std::string>("output_frame", output_frame_, "base_link");
    pnh_.param<std::string>("cluster_result_topic", cluster_result_topic_, "cluster_result");
    pnh_.param<float>("cluster_tolerance", cluster_tolerance_, 0.5);
    pnh_.param<int>("min_cluster_size", min_cluster_size_, 20);
    pnh_.param<int>("max_cluster_size", max_cluster_size_, 25000);
    
    pnh_.param<float>("max_distance", max_distance_, 50.0);
    pnh_.param<float>("min_x", min_x_, 1.0);
    pnh_.param<float>("max_z", max_z_, -0.5);
    pnh_.param<float>("min_z", min_z_, -2.0);
    pnh_.param<float>("min_y", min_y_, -5.0);
    pnh_.param<float>("max_y", max_y_, 5.0);
    
    cluster_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cluster_cloud", 1);
    detection3d_pub_ = nh_.advertise<vision_msgs::Detection3DArray>(cluster_result_topic_, 1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("cluster_markers", 1);
    
    lidar_sub_ = nh_.subscribe(lidar_topic_, 1, &LidarClusterNode::lidarCallback, this);
    
    tf_buffer_.reset(new tf2_ros::Buffer(ros::Duration(2.0), true));
    tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
    
    last_call_time_ = ros::Time::now();
    
    ROS_INFO("LiDAR Cluster Node with Tracking initialized");
    ROS_INFO("Subscribing to: %s", lidar_topic_.c_str());
    ROS_INFO("Cluster tolerance: %.2f", cluster_tolerance_);
    ROS_INFO("Min cluster size: %d", min_cluster_size_);
    ROS_INFO("Max cluster size: %d", max_cluster_size_);
  }
  
  bool isEgoVehicle(const pcl::PointXYZ& point)
  {
    return (point.x >= -1.0 && point.x <= 4.7 &&
            point.y >= -1.0 && point.y <= 1.0 &&
            point.z >= -0.5 && point.z <= 2.0);
  }
  
  void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    ros::Time current_call_time = ros::Time::now();
    ros::Duration callback_interval = current_call_time - last_call_time_;
    last_call_time_ = current_call_time;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud = downsampleCloudMsg(cloud_msg);
    
    if (downsampled_cloud->empty())
    {
      ROS_WARN("Downsampled cloud is empty!");
      return;
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud;
    if (cloud_msg->header.frame_id != output_frame_)
    {
      transformed_cloud = cloud2TransformedCloud(downsampled_cloud, 
                                                 cloud_msg->header.frame_id, 
                                                 output_frame_,
                                                 cloud_msg->header.stamp);
    }
    else
    {
      transformed_cloud = downsampled_cloud;
    }
    
    std::vector<pcl::PointIndices> cluster_indices = euclideanClusterExtraction(transformed_cloud);
    
    if (cluster_indices.empty())
    {
      ROS_INFO("No clusters found");
      return;
    }
    
    vision_msgs::Detection3DArray detections3d_msg;
    detections3d_msg.header = cloud_msg->header;
    detections3d_msg.header.frame_id = output_frame_;
    
    pcl::PointCloud<pcl::PointXYZ> combined_cluster_cloud;
    
    for (const auto& cluster : cluster_indices)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      for (const auto& idx : cluster.indices)
      {
        cluster_cloud->points.push_back(transformed_cloud->points[idx]);
        combined_cluster_cloud.points.push_back(transformed_cloud->points[idx]);
      }
      
      createBoundingBox(detections3d_msg, cluster_cloud);
    }
    
    //  Tracking 업데이트 (ID 할당)
    tracker_.update(detections3d_msg, cloud_msg->header.stamp.toSec());
    
    //  Marker에 stable ID 표시
    visualization_msgs::MarkerArray marker_array_msg = 
        createMarkerArray(detections3d_msg, callback_interval.toSec());
    
    detection3d_pub_.publish(detections3d_msg);
    marker_pub_.publish(marker_array_msg);
    
    sensor_msgs::PointCloud2 cluster_cloud_msg;
    pcl::toROSMsg(combined_cluster_cloud, cluster_cloud_msg);
    cluster_cloud_msg.header = detections3d_msg.header;
    cluster_cloud_pub_.publish(cluster_cloud_msg);
    
    ROS_INFO("Published %lu clusters (tracked)", detections3d_msg.detections.size());
  }
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleCloudMsg(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    
    float const max_squared_distance = max_distance_ * max_distance_;
    
    for (uint32_t i = 0; i < cloud->points.size(); ++i)
    {
      auto const& point = cloud->points[i];
      float const squared_distance = std::pow(point.x, 2) + std::pow(point.y, 2) + std::pow(point.z, 2);
      
      if (isEgoVehicle(point) ||
          squared_distance > max_squared_distance || 
          point.x < min_x_ ||
          point.y < min_y_ ||
          point.y > max_y_ ||  
          point.z > max_z_ || 
          point.z < min_z_)
      {
        cloud->points[i] = cloud->points[cloud->points.size() - 1];
        cloud->points.resize(cloud->points.size() - 1);
        --i;
      }
    }
    
    ROS_INFO("Downsampled cloud size: %lu", cloud->points.size());
    return cloud;
  }
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2TransformedCloud(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
      const std::string& source_frame, 
      const std::string& target_frame,
      const ros::Time& stamp)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    try
    {
      geometry_msgs::TransformStamped tf = tf_buffer_->lookupTransform(
          target_frame, source_frame, stamp, ros::Duration(0.1));
      pcl_ros::transformPointCloud(*cloud, *transformed_cloud, tf.transform);
      ROS_INFO("Transformed cloud from %s to %s", source_frame.c_str(), target_frame.c_str());
    }
    catch (tf2::TransformException& e)
    {
      ROS_WARN("TF transform failed: %s", e.what());
      return cloud;
    }
    
    return transformed_cloud;
  }
  
  std::vector<pcl::PointIndices> euclideanClusterExtraction(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
  {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    std::vector<pcl::PointIndices> cluster_indices;
    DBSCANKdtreeCluster<pcl::PointXYZ> dc;
    
    tree->setInputCloud(cloud);
    
    dc.setCorePointMinPts(min_cluster_size_);
    dc.setMinClusterSize(min_cluster_size_);
    dc.setMaxClusterSize(max_cluster_size_);
    dc.setClusterTolerance(cluster_tolerance_);
    dc.setSearchMethod(tree);
    dc.setInputCloud(cloud);
    dc.extract(cluster_indices);
    
    ROS_INFO("Found %lu clusters", cluster_indices.size());
    return cluster_indices;
  }
  
  void createBoundingBox(
      vision_msgs::Detection3DArray& detections3d_msg, 
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
  {
    if (cloud->points.empty())
      return;
    
    vision_msgs::Detection3D detection3d;
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ min_pt, max_pt;
    Eigen::Vector4f centroid;
    
    pcl::compute3DCentroid(*cloud, centroid);
    
    double theta = -atan2(centroid[1], sqrt(pow(centroid[0], 2) + pow(centroid[2], 2)));
    
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
    
    pcl::getMinMax3D(*transformed_cloud, min_pt, max_pt);
    
    Eigen::Vector4f transformed_bbox_center = Eigen::Vector4f(
        (min_pt.x + max_pt.x) / 2, 
        (min_pt.y + max_pt.y) / 2, 
        (min_pt.z + max_pt.z) / 2, 
        1);
    Eigen::Vector4f bbox_center = transform.inverse() * transformed_bbox_center;
    Eigen::Quaternionf q(transform.inverse().rotation());
    
    detection3d.bbox.center.position.x = bbox_center[0];
    detection3d.bbox.center.position.y = bbox_center[1];
    detection3d.bbox.center.position.z = bbox_center[2];
    detection3d.bbox.center.orientation.x = q.x();
    detection3d.bbox.center.orientation.y = q.y();
    detection3d.bbox.center.orientation.z = q.z();
    detection3d.bbox.center.orientation.w = q.w();
    detection3d.bbox.size.x = max_pt.x - min_pt.x;
    detection3d.bbox.size.y = max_pt.y - min_pt.y;
    detection3d.bbox.size.z = max_pt.z - min_pt.z;
    
    vision_msgs::ObjectHypothesisWithPose result;
    result.id = -1;  // ✅ Tracker가 나중에 ID 할당
    result.score = 1.0;
    detection3d.results.push_back(result);
    
    detections3d_msg.detections.push_back(detection3d);
  }
  
  visualization_msgs::MarkerArray createMarkerArray(
      const vision_msgs::Detection3DArray& detections3d_msg, 
      const double& duration)
  {
    visualization_msgs::MarkerArray marker_array;
    
    visualization_msgs::Marker delete_marker;
    delete_marker.header = detections3d_msg.header;
    delete_marker.ns = "clusters";
    delete_marker.id = 0;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    
    for (size_t i = 0; i < detections3d_msg.detections.size(); i++)
    {
      const auto& det = detections3d_msg.detections[i];
      
      if (std::isfinite(det.bbox.size.x) &&
          std::isfinite(det.bbox.size.y) &&
          std::isfinite(det.bbox.size.z))
      {
        visualization_msgs::Marker marker;
        marker.header = detections3d_msg.header;
        marker.ns = "clusters";
        marker.id = det.results[0].id;  // ✅ Stable ID 사용
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = det.bbox.center;
        marker.scale.x = det.bbox.size.x;
        marker.scale.y = det.bbox.size.y;
        marker.scale.z = det.bbox.size.z;
        
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5;
        
        marker.lifetime = ros::Duration(duration * 2);
        marker_array.markers.push_back(marker);
        
        visualization_msgs::Marker text_marker;
        text_marker.header = detections3d_msg.header;
        text_marker.ns = "cluster_ids";
        text_marker.id = det.results[0].id;  // ✅ Stable ID 사용
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose = det.bbox.center;
        text_marker.pose.position.z += det.bbox.size.z / 2 + 0.5;
        text_marker.scale.z = 0.5;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        text_marker.text = "ID " + std::to_string(det.results[0].id);  // ✅ 텍스트도 변경
        text_marker.lifetime = ros::Duration(duration * 2);
        marker_array.markers.push_back(text_marker);
      }
    }
    
    return marker_array;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_cluster_node");
  LidarClusterNode node;
  ros::spin();
  return 0;
}
