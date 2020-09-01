/*
 * @Copyright: 2020, Authors. All rights reserved.
 * @Description:
 * @Author: v_hezhenpeng
 * @Date: 2020-06-08
 */

#ifndef POSE_ESTIMATION_PCL_PROCESS_H
#define POSE_ESTIMATION_PCL_PROCESS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/visualization/pcl_visualizer.h>


namespace pose_estimation {
class ModelMatch {
 public:
  ModelMatch() = default;
  ModelMatch(const ModelMatch&) = delete;
  ModelMatch& operator=(const ModelMatch&) = delete;
  virtual ~ModelMatch() = default;

  bool Initialization(const std::string &yaml_path, const pcl::PointCloud<pcl::PointXYZ>::Ptr model);

  bool MatchModel(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scene,
                  Eigen::Matrix4f &transform);

  bool RemoveGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left);

  void Visualization(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scene,
                     const Eigen::Matrix4f &transform);

 private:
  bool LoadParams(const std::string &yaml_path);

  bool LoadModel(const pcl::PointCloud<pcl::PointXYZ>::Ptr model);

  bool FeatureExtraction(const float voxel_size,
                         pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
                         pcl::PointCloud<pcl::FPFHSignature33>::Ptr features);

  bool RansacPoseEstimation(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scene,
                            Eigen::Matrix4f &transform);

  bool IcpFineTuning(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scene,
                     const Eigen::Matrix4f &initial_guess,
                     Eigen::Matrix4f &transform);

  void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event, void* nothing);

  int show_align_;
  float voxel_size_scene_;
  float voxel_size_model_;
  int ransac_times_;
  float match_threshold_;
  pcl::PointCloud<pcl::PointNormal>::Ptr normal_model_;
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature_model_;
};
}  // namespace pose_estimation


#endif  // POSE_ESTIMATION_PCL_PROCESS_H
