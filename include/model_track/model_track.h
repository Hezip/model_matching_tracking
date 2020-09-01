/*
 * @Copyright: 2020, Authors. All rights reserved.
 * @Description:
 * @Author: v_hezhenpeng
 * @Date: 2020-06-10
 */
#ifndef POSE_ESTIMATION_LOCAL_TRACKING_H
#define POSE_ESTIMATION_LOCAL_TRACKING_H

#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh_omp.h>

namespace pose_estimation{
class ModelTrack {
 public:
  ModelTrack() = default;
  ModelTrack(const ModelTrack&) = delete;
  ModelTrack& operator=(const ModelTrack&) = delete;
  virtual ~ModelTrack() = default;

  bool Initialize(const double &time, const Eigen::Matrix4f &transform,
                  const pcl::PointCloud<pcl::PointXYZ>::Ptr &model);

  bool TrackModel(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scene);

  void Visualization(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scene,
                     const Eigen::Matrix4f &transform);

  Eigen::Matrix4f get_transform();

 private:
  bool Predict(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scene);

  bool Update(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scene);

  bool set_transform(const Eigen::Matrix4f &transform);

  bool set_time(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scene);

  double t_;
  double t_last_;
  std::vector<float> status_;
  std::vector<float> pos_last_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_;
};
}  // namespace pose_estimation

#endif //POSE_ESTIMATION_LOCAL_TRACKING_H
