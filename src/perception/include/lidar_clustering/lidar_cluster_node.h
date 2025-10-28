#ifndef LIDAR_CLUSTER_NODE_H
#define LIDAR_CLUSTER_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <vision_msgs/Detection3DArray.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#include "lidar_clustering/simple_tracker.h"

/**
 * Clustering Parameters
 */
struct ClusteringParams 
{
    float tolerance;
    int min_size;
    int max_size;
    
    ClusteringParams() 
        : tolerance(0.5f), min_size(20), max_size(25000) {}
};

/**
 * Point Cloud Filtering Parameters
 */
struct FilteringParams 
{
    float max_distance;
    float min_x;
    float max_x;
    float min_y;
    float max_y;
    float min_z;
    float max_z;
    
    FilteringParams() 
        : max_distance(50.0f), 
          min_x(1.0f), max_x(100.0f),
          min_y(-5.0f), max_y(5.0f),
          min_z(-2.0f), max_z(-0.5f) {}
};

/**
 * Ego Vehicle Bounding Box (for filtering)
 */
struct EgoVehicleBounds 
{
    float min_x, max_x;
    float min_y, max_y;
    float min_z, max_z;
    
    EgoVehicleBounds() 
        : min_x(-1.0f), max_x(4.7f),
          min_y(-1.0f), max_y(1.0f),
          min_z(-0.5f), max_z(2.0f) {}
    
    inline bool contains(const pcl::PointXYZ& point) const
    {
        return (point.x >= min_x && point.x <= max_x &&
                point.y >= min_y && point.y <= max_y &&
                point.z >= min_z && point.z <= max_z);
    }
};

/**
 * LiDAR Clustering Node with Object Tracking
 */
class LidarClusterNode
{
public:
    LidarClusterNode();
    
private:
    // ROS NodeHandles
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    // ROS Publishers
    ros::Publisher cluster_cloud_pub_;
    ros::Publisher detection3d_pub_;
    ros::Publisher marker_pub_;
    
    // ROS Subscriber
    ros::Subscriber lidar_sub_;
    
    // TF
    boost::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    boost::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Parameters
    std::string lidar_topic_;
    std::string output_frame_;
    std::string cluster_result_topic_;
    
    ClusteringParams clustering_params_;
    FilteringParams filtering_params_;
    EgoVehicleBounds ego_bounds_;
    
    // Tracker
    SimpleTracker tracker_;
    
    // Timing
    ros::Time last_call_time_;
    
    // Parameter loading
    void loadParameters();
    
    // Callbacks
    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    
    // Processing pipeline
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleCloudMsg(
        const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2TransformedCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const std::string& source_frame, 
        const std::string& target_frame,
        const ros::Time& stamp);
    
    std::vector<pcl::PointIndices> euclideanClusterExtraction(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    
    void createBoundingBox(
        vision_msgs::Detection3DArray& detections3d_msg, 
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    
    visualization_msgs::MarkerArray createMarkerArray(
        const vision_msgs::Detection3DArray& detections3d_msg, 
        const double& duration);
};

#endif // LIDAR_CLUSTER_NODE_H