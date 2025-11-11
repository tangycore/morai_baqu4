#ifndef LIDAR_CLUSTER_NODE_H
#define LIDAR_CLUSTER_NODE_H
#include <string>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>   
#include <boost/shared_ptr.hpp>        
#include "lidar_clustering/tracker.h"  

// 전방 선언들
namespace tf2_ros {
class Buffer;
class TransformListener;
}

namespace visualization_msgs {
struct MarkerArray;
}

namespace vision_msgs {
struct Detection3DArray;
}

namespace pcl {
template<typename T> class PointCloud;
struct PointXYZ;
struct PointIndices;
}

/**
 * Clustering Parameters
 */
struct ClusteringParams {
    float tolerance;
    int   min_size;
    int   max_size;
    ClusteringParams() : tolerance(0.7f), min_size(20), max_size(8000) {}
};

/**
 * Point Cloud Filtering Parameters
 */
struct FilteringParams {
    float max_distance;
    float min_x, max_x;
    float min_y, max_y;
    float min_z, max_z;
    FilteringParams()
        : max_distance(60.0f),
          min_x(0.0f), max_x(60.0f),
          min_y(-3.5f), max_y(3.5f),
          min_z(0.1f),  max_z(3.0f) {}
};

/**
 * Ego Vehicle Bounding Box (for filtering)
 */
struct EgoVehicleBounds {
    float min_x, max_x;
    float min_y, max_y;
    float min_z, max_z;
    EgoVehicleBounds()
        : min_x(-1.0f), max_x(4.7f),
          min_y(-1.0f), max_y(1.0f),
          min_z(-0.5f), max_z(2.0f) {}
    inline bool contains(const pcl::PointXYZ& p) const {
        return (p.x >= min_x && p.x <= max_x &&
                p.y >= min_y && p.y <= max_y &&
                p.z >= min_z && p.z <= max_z);
    }
};

class LidarClusterNode {
public:
    LidarClusterNode();

private:
    // ROS
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Publisher  cluster_cloud_pub_;
    ros::Publisher  detection3d_pub_;
    ros::Publisher  marker_pub_;
    ros::Subscriber lidar_sub_;

    // TF
    boost::shared_ptr<tf2_ros::Buffer>           tf_buffer_;
    boost::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Parameters
    std::string lidar_topic_;
    std::string output_frame_;
    std::string cluster_result_topic_;

    ClusteringParams  clustering_params_;
    FilteringParams   filtering_params_;
    EgoVehicleBounds  ego_bounds_;

    // Tracker
    Tracker tracker_;   // 더 줄이려면 std::unique_ptr<Tracker>로 바꾸고 여기서 tracker.h 제거 가능

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
