//
// Created by he on 2020/6/10.
//

#include "model_track/model_track.h"

#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/visualization/pcl_visualizer.h>


using pcl::PointXYZ;
using pcl::PointCloud;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointXYZ> ColorHandlerXYZ;

void
print4x4Matrix (const Eigen::Matrix4f & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f %6.3f| \n", matrix (0, 0), matrix (0, 1), matrix (0, 2), matrix (0, 3));
  printf ("R = | %6.3f %6.3f %6.3f %6.3f| \n", matrix (1, 0), matrix (1, 1), matrix (1, 2), matrix (1, 3));
  printf ("    | %6.3f %6.3f %6.3f %6.3f| \n", matrix (2, 0), matrix (2, 1), matrix (2, 2), matrix (2, 3));
  printf ("    | %6.3f %6.3f %6.3f %6.3f| \n", matrix (3, 0), matrix (3, 1), matrix (3, 2), matrix (3, 3));
}

namespace pose_estimation{

bool ModelTrack::Initialize(const double &time, const Eigen::Matrix4f &transform,
                            const pcl::PointCloud<pcl::PointXYZ>::Ptr &model) {
  model_ = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>);
  pcl::copyPointCloud(*model, *model_);
  status_ = std::vector<float>{0,0,0, 1,0,0,0, 0,0,0};
  pos_last_ = std::vector<float>{0,0,0};
  t_ = 0;
  t_last_ = time;
  set_transform(transform);
  status_[7] = status_[8] = status_[9] = 0;
}

bool ModelTrack::TrackModel(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scene) {
  Predict(scene);
  Update(scene);
  return true;
}


bool ModelTrack::Update(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scene) {
  PointCloud<PointXYZ>::Ptr model_tf(new PointCloud<PointXYZ>);
  Eigen::Matrix4f initial_guess = get_transform();
  pcl::transformPointCloud(*model_, *model_tf, initial_guess);
  PointCloud<PointXYZ>::Ptr registered (new PointCloud<PointXYZ>);
//  pcl::IterativeClosestPoint<PointXYZ, PointXYZ> icp;
  pcl::GeneralizedIterativeClosestPoint<PointXYZ, PointXYZ> icp;
  icp.setMaximumIterations(30);
  icp.setMaxCorrespondenceDistance(0.15);
  icp.setInputTarget(scene);
  icp.setInputSource(model_tf);
  {
    pcl::ScopeTime t("Icp tracking");
    icp.align(*registered);
  }
//  pcl::io::savePCDFileASCII ("model_tf.pcd", *model_tf); //将点云保存到PCD文件中
//  pcl::io::savePCDFileASCII ("scene.pcd", *scene); //将点云保存到PCD文件中
  if (icp.hasConverged()) {
    Eigen::Matrix4f icp_transformation = icp.getFinalTransformation();
    Eigen::Matrix4f transform = icp_transformation * initial_guess;
    set_transform(transform);
    return true;
  } else {
    pcl::console::print_error("Update not converge\n");
    return false;
  }
}

bool ModelTrack::Predict(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scene) {
  set_time(scene);
  status_[0] += status_[7] * t_;
  status_[1] += status_[8] * t_;
  status_[2] += status_[9] * t_;
  return true;
}

Eigen::Matrix4f ModelTrack::get_transform() {
  Eigen::Matrix4f transform;
  Eigen::Quaternionf quaternion(status_[3],status_[4],status_[5],status_[6]);
  transform.block<3,3>(0,0) = quaternion.matrix();
  transform.block<4,1>(0,3) << status_[0], status_[1], status_[2], 1;
  return transform;
}

bool ModelTrack::set_transform(const Eigen::Matrix4f &transform) {
  Eigen::Quaternionf quaternion(transform.block<3,3>(0,0));
  quaternion.normalize();
  Eigen::Vector3f position_new = transform.block<3,1>(0,3);
  Eigen::Vector3f position_old(pos_last_[0], pos_last_[1], pos_last_[2]);
  Eigen::Vector3f velocity = (position_new - position_old) / t_;
  std::vector<float> status_new{position_new.x(), position_new.y(),position_new.z(),
                                quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z(),
                                velocity.x(), velocity.y(), velocity.z()};
  status_.swap(status_new);
  pos_last_[0] = position_new(0);
  pos_last_[1] = position_new(1);
  pos_last_[2] = position_new(2);
//  for(auto i:status_){
//    std::cout << i << std::endl;
//  }
}

bool ModelTrack::set_time(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scene){
  double time = scene->header.stamp * 1.0 / 1e3;
  if (time < t_last_){
    pcl::console::print_error("Wrong time stamp\n");
    return false;
  } else {
    t_ = time - t_last_;
    t_last_ = time;
    std::cout << "t_: " << t_ << std::endl;
    std::cout << "t_last_: " << t_last_ << std::endl;
  }
  return true;
}

void ModelTrack::Visualization(const PointCloud<PointXYZ>::Ptr &scene,
                                 const Eigen::Matrix4f &transform) {
  PointCloud<PointXYZ>::Ptr model_tf(new PointCloud<PointXYZ>);
  copyPointCloud(*model_, *model_tf);
  transformPointCloud(*model_tf, *model_tf, transform);
  pcl::visualization::PCLVisualizer visu("Update");
  visu.addPointCloud(scene, ColorHandlerXYZ(scene, 0.0, 255.0, 0.0), "scene");
  visu.addPointCloud(model_tf, ColorHandlerXYZ(model_tf, 255.0, 0.0, 255.0), "object_aligned");
  Eigen::Vector3f translation = transform.block<3,1>(0,3);
  Eigen::Quaternionf quaternion(transform.block<3,3>(0,0));
  quaternion.normalize();
  visu.addCube(translation, quaternion, 6, 4, 3, "OBB");
  visu.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");
  visu.setCameraPosition(-50,0,0,1,0,0,0,0,1);
  visu.spin();
}

} // namespace local_tracking