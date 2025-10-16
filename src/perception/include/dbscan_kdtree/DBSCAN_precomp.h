#ifndef DBSCAN_PRECOMP_H
#define DBSCAN_PRECOMP_H

#include "DBSCAN_simple.h"
#include <pcl/point_types.h>

template <typename PointT>
class DBSCANPrecompCluster : public DBSCANSimpleCluster<PointT> {
public:
  virtual void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    this->input_cloud_ = cloud;
    int size = this->input_cloud_->points.size();
    adjoint_indexes_ = std::vector<std::vector<int>>(size, std::vector<int>());
    distances_ = std::vector<std::vector<float>>(size, std::vector<float>());
    precomp();
  }

protected:
  std::vector<std::vector<float>> distances_;
  std::vector<std::vector<int>> adjoint_indexes_;

  void precomp() {
    for (int i = 0; i < this->input_cloud_->points.size(); i++) {
      this->search_method_->radiusSearch(i, this->eps_, adjoint_indexes_[i], distances_[i]);
    }
  }

  virtual int radiusSearch(int index, double radius,
                           std::vector<int> &k_indices,
                           std::vector<float> &k_sqr_distances) const {
    // radius = eps_
    k_indices = adjoint_indexes_[index];
    k_sqr_distances = distances_[index];
    return k_indices.size();
  }
}; // class DBSCANCluster

#endif // DBSCAN_PRECOMP_H