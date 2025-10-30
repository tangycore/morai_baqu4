/**
 * LiDAR Clustering Node for MORAI Simulator
 * Modified to work in base_link frame
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

// DBSCAN header (use project files)
#include "dbscan_kdtree/DBSCAN_kdtree.h"

class LidarClusterNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  
  // Publishers
  ros::Publisher cluster_cloud_pub_;
  ros::Publisher detection3d_pub_;
  ros::Publisher marker_pub_;
  
  // Subscriber
  ros::Subscriber lidar_sub_;
  
  // TF
  boost::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  boost::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Parameters
  std::string lidar_topic_;
  std::string output_frame_;
  std::string cluster_result_topic_;
  float cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;
  
  // Filtering parameters (base_link frame)
  float max_distance_;
  float min_x_;
  float max_x_;
  float min_y_;
  float max_y_;
  float min_z_;
  float max_z_;
  
  // Ego vehicle filtering parameters (base_link frame)
  float ego_min_x_, ego_max_x_;
  float ego_min_y_, ego_max_y_;
  float ego_min_z_, ego_max_z_;
  
  // Marker parameters
  // float marker_size_x_;
  // float marker_size_y_;
  // float marker_size_z_;
  
  ros::Time last_call_time_;

public:
  LidarClusterNode() : pnh_("~")
  {
    // Load parameters
    pnh_.param<std::string>("lidar_topic", lidar_topic_, "/front_lidar/velodyne_points");
    pnh_.param<std::string>("output_frame", output_frame_, "base_link");
    pnh_.param<std::string>("cluster_result_topic", cluster_result_topic_, "cluster_result");
    pnh_.param<float>("cluster_tolerance", cluster_tolerance_, 0.5);
    pnh_.param<int>("min_cluster_size", min_cluster_size_, 20);
    pnh_.param<int>("max_cluster_size", max_cluster_size_, 25000);
    
    // Filtering parameters (base_link frame)
    pnh_.param<float>("max_distance", max_distance_, 60.0);
    pnh_.param<float>("min_x", min_x_, 0.0);
    pnh_.param<float>("max_x", max_x_, 60.0);
    pnh_.param<float>("min_y", min_y_, -5.0);
    pnh_.param<float>("max_y", max_y_, 5.0);
    pnh_.param<float>("min_z", min_z_, -0.2);
    pnh_.param<float>("max_z", max_z_, 3.0);
    
    // Ego vehicle filtering parameters (base_link frame)
    pnh_.param<float>("ego_min_x", ego_min_x_, -1.0);
    pnh_.param<float>("ego_max_x", ego_max_x_, 4.7);
    pnh_.param<float>("ego_min_y", ego_min_y_, -1.0);
    pnh_.param<float>("ego_max_y", ego_max_y_, 1.0);
    pnh_.param<float>("ego_min_z", ego_min_z_, -0.5);
    pnh_.param<float>("ego_max_z", ego_max_z_, 2.0);
    
    // Marker parameters
    // pnh_.param<float>("marker_size_x", marker_size_x_, 1.0);
    // pnh_.param<float>("marker_size_y", marker_size_y_, 1.0);
    // pnh_.param<float>("marker_size_z", marker_size_z_, 1.0);
    
    // Publishers
    cluster_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cluster_cloud", 1);
    detection3d_pub_ = nh_.advertise<vision_msgs::Detection3DArray>(cluster_result_topic_, 1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("cluster_markers", 1);
    
    // Subscriber
    lidar_sub_ = nh_.subscribe(lidar_topic_, 1, &LidarClusterNode::lidarCallback, this);
    
    // TF setup
    tf_buffer_.reset(new tf2_ros::Buffer(ros::Duration(2.0), true));
    tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
    
    last_call_time_ = ros::Time::now();
    
    ROS_INFO("LiDAR Cluster Node initialized");
    ROS_INFO("Subscribing to: %s", lidar_topic_.c_str());
    ROS_INFO("Output frame: %s", output_frame_.c_str());
    ROS_INFO("Cluster tolerance: %.2f m", cluster_tolerance_);
    ROS_INFO("Min/Max cluster size: %d / %d", min_cluster_size_, max_cluster_size_);
    ROS_INFO("Filtering range (base_link): x[%.1f, %.1f], y[%.1f, %.1f], z[%.1f, %.1f]",
             min_x_, max_x_, min_y_, max_y_, min_z_, max_z_);
    ROS_INFO("Ego vehicle bounds: x[%.1f, %.1f], y[%.1f, %.1f], z[%.1f, %.1f]",
             ego_min_x_, ego_max_x_, ego_min_y_, ego_max_y_, ego_min_z_, ego_max_z_);
  }
  
  bool isEgoVehicle(const pcl::PointXYZ& point)
  {
    // base_link frame ego vehicle filtering
    return (point.x >= ego_min_x_ && point.x <= ego_max_x_ &&
            point.y >= ego_min_y_ && point.y <= ego_max_y_ &&
            point.z >= ego_min_z_ && point.z <= ego_max_z_);
  }
  
  void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    ros::Time current_call_time = ros::Time::now();
    ros::Duration callback_interval = current_call_time - last_call_time_;
    last_call_time_ = current_call_time;
    
    // 1. Convert ROS message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    
    if (cloud->empty())
    {
      ROS_WARN("Input cloud is empty!");
      return;
    }
    
    // 2. Transform to base_link frame first
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud;
    if (cloud_msg->header.frame_id != output_frame_)
    {
      transformed_cloud = cloud2TransformedCloud(cloud, 
                                                 cloud_msg->header.frame_id, 
                                                 output_frame_,
                                                 cloud_msg->header.stamp);
      if (transformed_cloud->empty())
      {
        ROS_WARN("TF transformation failed or resulted in empty cloud!");
        return;
      }
    }
    else
    {
      transformed_cloud = cloud;
    }
    
    // 3. Filter point cloud in base_link frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = filterPointCloud(transformed_cloud);
    
    if (filtered_cloud->empty())
    {
      ROS_WARN("Filtered cloud is empty!");
      return;
    }
    
    ROS_INFO("Filtered cloud size: %lu points", filtered_cloud->points.size());
    
    // 4. Perform clustering
    std::vector<pcl::PointIndices> cluster_indices = euclideanClusterExtraction(filtered_cloud);
    
    if (cluster_indices.empty())
    {
      ROS_INFO("No clusters found");
      return;
    }
    
    // 5. Create 3D Detection messages
    vision_msgs::Detection3DArray detections3d_msg;
    detections3d_msg.header = cloud_msg->header;
    detections3d_msg.header.frame_id = output_frame_;
    
    pcl::PointCloud<pcl::PointXYZ> combined_cluster_cloud;
    
    for (const auto& cluster : cluster_indices)
    {
      // Extract points for each cluster
      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      for (const auto& idx : cluster.indices)
      {
        cluster_cloud->points.push_back(filtered_cloud->points[idx]);
        combined_cluster_cloud.points.push_back(filtered_cloud->points[idx]);
      }
      
      // Create 3D bounding box
      createBoundingBox(detections3d_msg, cluster_cloud);
    }
    
    // 6. Create markers
    visualization_msgs::MarkerArray marker_array_msg = 
        createMarkerArray(detections3d_msg, callback_interval.toSec());
    
    // 7. Publish results
    detection3d_pub_.publish(detections3d_msg);
    marker_pub_.publish(marker_array_msg);
    
    // Publish clustered points
    sensor_msgs::PointCloud2 cluster_cloud_msg;
    pcl::toROSMsg(combined_cluster_cloud, cluster_cloud_msg);
    cluster_cloud_msg.header = detections3d_msg.header;
    cluster_cloud_pub_.publish(cluster_cloud_msg);
    
    ROS_INFO("Published %lu clusters", detections3d_msg.detections.size());
  }
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointCloud(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
  {
    // Filter point cloud in base_link frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    
    float const max_squared_distance = max_distance_ * max_distance_;
    
    for (const auto& point : cloud->points)
    {
      float const squared_distance = point.x * point.x + point.y * point.y + point.z * point.z;
      
      // Apply all filters in base_link frame
      if (isEgoVehicle(point) ||                    // Remove ego vehicle
          squared_distance > max_squared_distance || // Distance limit
          point.x < min_x_ || point.x > max_x_ ||   // X range
          point.y < min_y_ || point.y > max_y_ ||   // Y range
          point.z < min_z_ || point.z > max_z_)     // Z range (ground/height)
      {
        continue;  // Skip this point
      }
      
      filtered->points.push_back(point);
    }
    
    ROS_INFO("Filtered: %lu -> %lu points", cloud->points.size(), filtered->points.size());
    return filtered;
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
      return cloud;  // Return original on failure
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
    
    // Calculate centroid
    pcl::compute3DCentroid(*cloud, centroid);
    
    // Calculate Z-axis rotation angle (object facing center)
    double theta = -atan2(centroid[1], sqrt(pow(centroid[0], 2) + pow(centroid[2], 2)));
    
    // Apply rotation transform
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
    
    // Calculate Min/Max
    pcl::getMinMax3D(*transformed_cloud, min_pt, max_pt);
    
    // Calculate bounding box center
    Eigen::Vector4f transformed_bbox_center = Eigen::Vector4f(
        (min_pt.x + max_pt.x) / 2, 
        (min_pt.y + max_pt.y) / 2, 
        (min_pt.z + max_pt.z) / 2, 
        1);
    Eigen::Vector4f bbox_center = transform.inverse() * transformed_bbox_center;
    Eigen::Quaternionf q(transform.inverse().rotation());
    
    // Fill Detection3D message
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
    
    // Add result (class ID = unknown)
    vision_msgs::ObjectHypothesisWithPose result;
    result.id = 0;  // unknown object
    result.score = 1.0;
    detection3d.results.push_back(result);
    
    detections3d_msg.detections.push_back(detection3d);
  }
  
  visualization_msgs::MarkerArray createMarkerArray(
      const vision_msgs::Detection3DArray& detections3d_msg, 
      const double& duration)
  {
    visualization_msgs::MarkerArray marker_array;
    
    // Delete all previous markers
    visualization_msgs::Marker delete_marker;
    delete_marker.header = detections3d_msg.header;
    delete_marker.ns = "clusters";
    delete_marker.id = 0;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    
    // Create markers for each detection
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
        marker.id = i + 1;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = det.bbox.center;
        marker.scale.x = det.bbox.size.x;
        marker.scale.y = det.bbox.size.y;
        marker.scale.z = det.bbox.size.z;
    
        // Color: Green (cluster)
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5;
        
        marker.lifetime = ros::Duration(duration * 2);  // 2x to prevent flickering
        marker_array.markers.push_back(marker);
        
        // Text marker for cluster ID
        visualization_msgs::Marker text_marker;
        text_marker.header = detections3d_msg.header;
        text_marker.ns = "cluster_ids";
        text_marker.id = i + 1;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose = det.bbox.center;
        text_marker.pose.position.z += marker.scale.z / 2 + 0.5;  // Display above
        text_marker.scale.z = 0.5;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        text_marker.text = "Cluster " + std::to_string(i);
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