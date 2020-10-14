/*
 * @Copyright: 2020, Authors. All rights reserved.
 * @Description: A set of tools of PCL for pose estimation.
 * @Author: v_hezhenpeng
 * @Date: 2020-06-08
 */

#include <memory>

#include <pcl/io/pcd_io.h>

#include "model_match/model_match.h"

void print4x4Matrix (const Eigen::Matrix4f & matrix) {
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}


int main(int argc, char **argv) {
  //  load model and scene
  pcl::PointCloud<pcl::PointXYZ>::Ptr load_object(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr load_scene(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *load_object) < 0 ||
      pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *load_scene) < 0) {
    pcl::console::print_error("Error loading file!\n");
    return 1;
  }

  std::shared_ptr<pose_estimation::ModelMatch> model_match_ptr = std::make_shared<pose_estimation::ModelMatch>();
  Eigen::Matrix4f precise_tf = Eigen::Matrix4f::Identity();
  model_match_ptr->Initialization("../config/config.yaml", load_object);
  model_match_ptr->MatchModel(load_scene, precise_tf);

  print4x4Matrix(precise_tf);

  Eigen::Vector3f eulerAngle=precise_tf.block<3,3>(0,0).eulerAngles(2,1,0);
  float yaw_angle = eulerAngle(0) * 180 / M_PI;
  float x = precise_tf(0,3);
  float y = precise_tf(1,3);
  pcl::console::print_highlight("Truck in lidar coordinate  X:%.2f(m)  Y:%.2f(m)  Heading:%.2f(degree)\n",
                                x, y, yaw_angle);

  model_match_ptr->Visualization(load_scene, precise_tf);
  return (0);
}