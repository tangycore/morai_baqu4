#include "lidar_clustering/lidar_cluster_node.h"
#include <sstream>
#include <iomanip>
#include <cmath>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <vision_msgs/Detection3DArray.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>            
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <Eigen/Dense>
#include "dbscan_kdtree/DBSCAN_kdtree.h"

LidarClusterNode::LidarClusterNode() : pnh_("~")
{
    loadParameters();
    
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
    
    ROS_INFO("=== LiDAR Cluster Node with Tracking Initialized ===");
    ROS_INFO("Subscribing to: %s", lidar_topic_.c_str());
    ROS_INFO("Output frame: %s", output_frame_.c_str());
    ROS_INFO("Cluster tolerance: %.2f m", clustering_params_.tolerance);
    ROS_INFO("Cluster size: [%d, %d]", clustering_params_.min_size, clustering_params_.max_size);
    ROS_INFO("Filter distance: %.1f m", filtering_params_.max_distance);
    ROS_INFO("Filter X: [%.1f, %.1f] m", filtering_params_.min_x, filtering_params_.max_x);
    ROS_INFO("Filter Y: [%.1f, %.1f] m", filtering_params_.min_y, filtering_params_.max_y);
    ROS_INFO("Filter Z: [%.1f, %.1f] m", filtering_params_.min_z, filtering_params_.max_z);
    ROS_INFO("Ego bounds: X[%.1f, %.1f], Y[%.1f, %.1f], Z[%.1f, %.1f]",
             ego_bounds_.min_x, ego_bounds_.max_x,
             ego_bounds_.min_y, ego_bounds_.max_y,
             ego_bounds_.min_z, ego_bounds_.max_z);
}

void LidarClusterNode::loadParameters()
{
    // Topic settings
    pnh_.param<std::string>("lidar_topic", lidar_topic_, "/front_lidar/velodyne_points");
    pnh_.param<std::string>("output_frame", output_frame_, "base_link");
    pnh_.param<std::string>("cluster_result_topic", cluster_result_topic_, "cluster_result");
    
    // Clustering parameters
    pnh_.param<float>("cluster_tolerance", clustering_params_.tolerance, 0.7f);
    pnh_.param<int>("min_cluster_size", clustering_params_.min_size, 20);
    pnh_.param<int>("max_cluster_size", clustering_params_.max_size, 8000);
    
    // Filtering parameters
    pnh_.param<float>("max_distance", filtering_params_.max_distance, 60.0f);
    pnh_.param<float>("min_x", filtering_params_.min_x, 0.0f);
    pnh_.param<float>("max_x", filtering_params_.max_x, 60.0f);
    pnh_.param<float>("min_y", filtering_params_.min_y, -3.5f);
    pnh_.param<float>("max_y", filtering_params_.max_y, 3.5f);
    pnh_.param<float>("min_z", filtering_params_.min_z, 0.1f);
    pnh_.param<float>("max_z", filtering_params_.max_z, 3.0f);
    
    // Ego vehicle bounds
    pnh_.param<float>("ego_min_x", ego_bounds_.min_x, -1.0f);
    pnh_.param<float>("ego_max_x", ego_bounds_.max_x, 4.7f);
    pnh_.param<float>("ego_min_y", ego_bounds_.min_y, -1.0f);
    pnh_.param<float>("ego_max_y", ego_bounds_.max_y, 1.0f);
    pnh_.param<float>("ego_min_z", ego_bounds_.min_z, -0.5f);
    pnh_.param<float>("ego_max_z", ego_bounds_.max_z, 2.0f);
}

void LidarClusterNode::lidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    ros::Time current_call_time = ros::Time::now();
    ros::Duration callback_interval = current_call_time - last_call_time_;
    last_call_time_ = current_call_time;
    
    // 1. Downsample point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud = downsampleCloudMsg(cloud_msg);
    
    if (downsampled_cloud->empty())
    {
        ROS_WARN("Downsampled cloud is empty!");
        return;
    }
    
    // 2. Transform to output frame if needed
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud;
    if (cloud_msg->header.frame_id != output_frame_)
    {
        transformed_cloud = cloud2TransformedCloud(
            downsampled_cloud, 
            cloud_msg->header.frame_id, 
            output_frame_,
            cloud_msg->header.stamp);
    }
    else
    {
        transformed_cloud = downsampled_cloud;
    }
    
    // 3. Perform clustering
    std::vector<pcl::PointIndices> cluster_indices = euclideanClusterExtraction(transformed_cloud);
    
    if (cluster_indices.empty())
    {
        ROS_INFO("No clusters found");
        // Empty detection message still goes to tracker
    }
    
    // 4. Create 3D detections
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
    
    // 5. Update tracker (assign stable IDs + add lost tracks)
    tracker_.update(detections3d_msg, cloud_msg->header.stamp.toSec());
    
    // 6. Create visualization markers
    visualization_msgs::MarkerArray marker_array_msg = 
        createMarkerArray(detections3d_msg, callback_interval.toSec());
    
    // 7. Publish results
    detection3d_pub_.publish(detections3d_msg);
    marker_pub_.publish(marker_array_msg);
    
    sensor_msgs::PointCloud2 cluster_cloud_msg;
    pcl::toROSMsg(combined_cluster_cloud, cluster_cloud_msg);
    cluster_cloud_msg.header = detections3d_msg.header;
    cluster_cloud_pub_.publish(cluster_cloud_msg);
    
    ROS_INFO("Published %lu total detections (including lost tracks)", 
             detections3d_msg.detections.size());
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LidarClusterNode::downsampleCloudMsg(
    const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    
    float const max_squared_distance = filtering_params_.max_distance * filtering_params_.max_distance;
    
    for (uint32_t i = 0; i < cloud->points.size(); ++i)
    {
        auto const& point = cloud->points[i];
        float const squared_distance = std::pow(point.x, 2) + std::pow(point.y, 2) + std::pow(point.z, 2);
        //bool is_front_critical = (point.x > ego_bounds_.max_x && point.x < 20.0f && std::abs(point.y) < 1.5f);
        //float min_z_check = is_front_critical ? -0.3f : filtering_params_.min_z;
        // Filter out: ego vehicle, out of range, out of bounds
        if (ego_bounds_.contains(point) ||
            squared_distance > max_squared_distance || 
            point.x < filtering_params_.min_x ||
            point.x > filtering_params_.max_x ||
            point.y < filtering_params_.min_y ||
            point.y > filtering_params_.max_y ||  
            point.z > filtering_params_.max_z || 
            point.z < filtering_params_.min_z
        )
            //point.z < min_z_check)
        {
            cloud->points[i] = cloud->points[cloud->points.size() - 1];
            cloud->points.resize(cloud->points.size() - 1);
            --i;
        }
    }
    
    ROS_INFO("Filtered cloud: %lu points", cloud->points.size());
    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LidarClusterNode::cloud2TransformedCloud(
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

std::vector<pcl::PointIndices> LidarClusterNode::euclideanClusterExtraction(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    std::vector<pcl::PointIndices> cluster_indices;
    DBSCANKdtreeCluster<pcl::PointXYZ> dc;
    
    tree->setInputCloud(cloud);
    
    dc.setCorePointMinPts(clustering_params_.min_size);
    dc.setMinClusterSize(clustering_params_.min_size);
    dc.setMaxClusterSize(clustering_params_.max_size);
    dc.setClusterTolerance(clustering_params_.tolerance);
    dc.setSearchMethod(tree);
    dc.setInputCloud(cloud);
    dc.extract(cluster_indices);
    
    ROS_INFO("Found %lu clusters", cluster_indices.size());
    return cluster_indices;
}

void LidarClusterNode::createBoundingBox(
    vision_msgs::Detection3DArray& detections3d_msg, 
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    if (cloud->points.empty())
        return;
    
    vision_msgs::Detection3D detection3d;
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ min_pt, max_pt;
    Eigen::Vector4f centroid;
    
    // Compute centroid
    pcl::compute3DCentroid(*cloud, centroid);
    
    // Compute rotation angle (align with centroid direction)
    double theta = -atan2(centroid[1], sqrt(pow(centroid[0], 2) + pow(centroid[2], 2)));
    
    // Apply rotation
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
    
    // Get min/max bounds
    pcl::getMinMax3D(*transformed_cloud, min_pt, max_pt);
    
    // Compute bounding box center in original frame
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
    
    // Add result (ID will be assigned by tracker)
    vision_msgs::ObjectHypothesisWithPose result;
    result.id = -1;  // Tracker will assign stable ID
    result.score = 1.0;
    detection3d.results.push_back(result);
    
    detections3d_msg.detections.push_back(detection3d);
}

visualization_msgs::MarkerArray LidarClusterNode::createMarkerArray(
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
            // Bounding box marker
            visualization_msgs::Marker marker;
            marker.header = detections3d_msg.header;
            marker.ns = "clusters";
            marker.id = det.results[0].id;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = det.bbox.center;
            marker.scale.x = det.bbox.size.x;
            marker.scale.y = det.bbox.size.y;
            marker.scale.z = det.bbox.size.z;
            
            // Color based on score (lost tracks are dimmer)
            float score = det.results[0].score;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.3 + 0.5 * score;  // Lost tracks fade out
            
            marker.lifetime = ros::Duration(duration * 2);
            marker_array.markers.push_back(marker);
            
            // Text ID marker
            visualization_msgs::Marker text_marker;
            text_marker.header = detections3d_msg.header;
            text_marker.ns = "cluster_ids";
            text_marker.id = det.results[0].id;
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::ADD;
            text_marker.pose = det.bbox.center;
            text_marker.pose.position.z += det.bbox.size.z / 2 + 0.5;
            text_marker.scale.z = 0.5;
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            
            // Show ID and score
            std::stringstream ss;
            ss << "ID " << det.results[0].id << " (" << std::fixed << std::setprecision(2) << (score * 100) << "%)";
            text_marker.text = ss.str();
            
            text_marker.lifetime = ros::Duration(duration * 2);
            marker_array.markers.push_back(text_marker);
        }
    }
    
    return marker_array;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_cluster_node");
    LidarClusterNode node;
    ros::spin();
    return 0;
}