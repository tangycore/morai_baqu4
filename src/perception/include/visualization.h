#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

template <typename T>
void visualizeAnyPoint(
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        publisher,
    std::string const &ns, std_msgs::msg::Header const &header,
    std::vector<std::pair<T, T>> const &points, std::vector<double> const &color,
    rclcpp::Duration const &duration) {
  auto markerArray = std::make_shared<visualization_msgs::msg::MarkerArray>();

  auto delete_marker = visualization_msgs::msg::Marker();
  delete_marker.header = header;
  delete_marker.ns = ns;
  delete_marker.id = 0;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;

  markerArray->markers.push_back(delete_marker);

  for (int32_t i = 0; i < static_cast<int32_t>(points.size()); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = ns;
    marker.id = i + 1;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = points[i].first.x();
    marker.pose.position.y = points[i].first.y();
    marker.pose.position.z = points[i].first.z();

    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    marker.scale.x = points[i].second.x();
    marker.scale.y = points[i].second.y();
    marker.scale.z = points[i].second.z();

    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];

    marker.lifetime = duration;
    markerArray->markers.push_back(marker);
  }

  publisher->publish(*markerArray);
}