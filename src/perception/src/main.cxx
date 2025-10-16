#include <clustering_cxx/clustering.hxx>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ClusteringNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}