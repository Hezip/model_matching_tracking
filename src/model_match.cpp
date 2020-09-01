/*
 * @Copyright: 2020, Authors. All rights reserved.
 * @Description: A set of tools of PCL for pose estimation.
 * @Author: v_hezhenpeng
 * @Date: 2020-06-08
 */
#include "model_match/model_match.h"

#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/filters/passthrough.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>

#include <string>

using namespace pcl;

// Types
typedef PointNormal PointNT;
typedef FPFHSignature33 FeatureT;
typedef PointCloud<FeatureT> FeatureCloudT;
typedef visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;
typedef visualization::PointCloudColorHandlerCustom<PointXYZ> ColorHandlerXYZ;

namespace pose_estimation {

bool ModelMatch::LoadParams(const std::string &yaml_path) {
  YAML::Node config = YAML::LoadFile(yaml_path);
  voxel_size_model_ = config["voxel_size_model"].as<float>();
  voxel_size_scene_ = config["voxel_size_scene"].as<float>();
  ransac_times_ = config["ransac_times"].as<float>();
  match_threshold_ = config["match_threshold"].as<float>();

  show_align_ = 0;
  return true;
}


bool ModelMatch::LoadModel(const PointCloud<PointXYZ>::Ptr model) {
  normal_model_ = PointCloud<PointNormal>::Ptr(new PointCloud<PointNormal>);
  feature_model_ = FeatureCloudT::Ptr(new FeatureCloudT);
  copyPointCloud(*model, *normal_model_);
  FeatureExtraction(voxel_size_model_, normal_model_, feature_model_);
  return true;
}

bool ModelMatch::Initialization(const std::string &yaml_path, const pcl::PointCloud<pcl::PointXYZ>::Ptr model) {
  LoadParams(yaml_path);
  LoadModel(model);
  return true;
}

bool ModelMatch::MatchModel(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scene,
                            Eigen::Matrix4f &transform) {
  Eigen::Matrix4f rough_tf = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f precise_tf = Eigen::Matrix4f::Identity();
  if (!RansacPoseEstimation(scene, rough_tf)) {
    return false;
  }
  if (!IcpFineTuning(scene, rough_tf, precise_tf)) {
    return false;
  }
  transform = precise_tf;
  return true;
}

bool ModelMatch::RemoveGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left) {
  Eigen::Vector3f vector(0,1,0);
  Eigen::Vector3f point(0,0,0);
//  float t = 24.637/180 * M_PI;
  float t = 24.637/180 * M_PI;
	float u = vector(0);
	float v = vector(1);
	float w = vector(2);
	float a = point(0);
	float b = point(1);
	float c = point(2);

	Eigen::Matrix4f tf_to_horizontal;
	tf_to_horizontal<<u*u + (v*v + w*w)*cos(t), u*v*(1 - cos(t)) - w*sin(t), u*w*(1 - cos(t)) + v*sin(t), (a*(v*v + w*w) - u*(b*v + c*w))*(1 - cos(t)) + (b*w - c*v)*sin(t),
		u*v*(1 - cos(t)) + w*sin(t), v*v + (u*u + w*w)*cos(t), v*w*(1 - cos(t)) - u*sin(t), (b*(u*u + w*w) - v*(a*u + c*w))*(1 - cos(t)) + (c*u - a*w)*sin(t),
		u*w*(1 - cos(t)) - v*sin(t), v*w*(1 - cos(t)) + u*sin(t), w*w + (u*u + v*v)*cos(t), (c*(u*u + v*v) - w*(a*u + b*v))*(1 - cos(t)) + (a*v - b*u)*sin(t),
		0, 0, 0, 1;

  transformPointCloud(*cloud, *cloud, tf_to_horizontal);
//  pcl::io::savePCDFileASCII ("ground.pcd", *cloud); //将点云保存到PCD文件中
  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-2.8, 10.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_left);

  return true;
}

void ModelMatch::Visualization(const PointCloud<PointXYZ>::Ptr &scene,
                               const Eigen::Matrix4f &transform) {
  PointCloud<PointXYZ>::Ptr model_tf(new PointCloud<PointXYZ>);
  PointCloud<PointXYZ>::Ptr model_null(new PointCloud<PointXYZ>);
  copyPointCloud(*normal_model_, *model_tf);
  transformPointCloud(*model_tf, *model_tf, transform);

  visualization::PCLVisualizer visu("Alignment");
  visu.addPointCloud(scene, ColorHandlerXYZ(scene, 0.0, 255.0, 0.0), "scene");
  visu.addPointCloud(model_tf, ColorHandlerXYZ(model_tf, 0.0, 0.0, 255.0), "object_aligned");
  visu.setBackgroundColor(0.5,0.5,0.5);
  visu.setCameraPosition(-5,0,0,1,0,0,0,0,1);
  visu.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "object_aligned");
  visu.registerKeyboardCallback (&ModelMatch::keyboardEventOccurred, *this);

  while(!visu.wasStopped()) {
    visu.spinOnce();
    if (show_align_ == 0) {
      visu.updatePointCloud(model_tf, ColorHandlerXYZ(model_tf, 0.0, 0.0, 255.0), "object_aligned");
      visu.updatePointCloud(scene, ColorHandlerXYZ(scene, 0.0, 255.0, 0.0), "scene");
    } else if (show_align_ == 1) {
      visu.updatePointCloud(model_null, ColorHandlerXYZ(model_null, 0.0, 0.0, 255.0), "object_aligned");
      visu.updatePointCloud(scene, ColorHandlerXYZ(scene, 0.0, 255.0, 0.0), "scene");
    } else if (show_align_ == 2) {
      visu.updatePointCloud(model_tf, ColorHandlerXYZ(model_tf, 0.0, 0.0, 255.0), "object_aligned");
      visu.updatePointCloud(model_null, ColorHandlerXYZ(model_null, 0.0, 255.0, 0.0), "scene");
    }
  }
}

bool ModelMatch::FeatureExtraction(const float voxel_size,
                                    PointCloud<PointNormal>::Ptr cloud,
                                    PointCloud<FPFHSignature33>::Ptr features) {
  VoxelGrid<PointNormal> grid;
  float leaf = voxel_size;
  grid.setLeafSize(leaf, leaf, leaf);
  grid.setInputCloud(cloud);
  grid.filter(*cloud);

  console::print_highlight("Estimating scene normals...\n");
  NormalEstimationOMP<PointNormal,PointNormal> nest;
  nest.setRadiusSearch(voxel_size*3);
  nest.setInputCloud(cloud);
  nest.compute(*cloud);

  console::print_highlight("Estimating features...\n");
  FPFHEstimationOMP<PointNormal,PointNormal,FeatureT> fest;
  fest.setRadiusSearch(voxel_size*7);
  fest.setInputCloud(cloud);
  fest.setInputNormals(cloud);
  fest.compute(*features);

  return true;
}

bool ModelMatch::RansacPoseEstimation(const PointCloud<PointXYZ>::Ptr &scene,
                                      Eigen::Matrix4f &transform) {
  PointCloud<PointNormal>::Ptr normal_scene(new PointCloud<PointNormal>);
  copyPointCloud(*scene, *normal_scene);

  FeatureCloudT::Ptr feature_scene (new FeatureCloudT);
  FeatureExtraction(voxel_size_scene_, normal_scene, feature_scene);

  console::print_highlight("Starting alignment...\n");
  PointCloud<PointNormal>::Ptr object_aligned(new PointCloud<PointNormal>);
  SampleConsensusPrerejective<PointNormal,PointNormal,FeatureT> align;
  align.setInputSource(normal_model_);
  align.setSourceFeatures(feature_model_);
  align.setInputTarget(normal_scene);
  align.setTargetFeatures(feature_scene);
  align.setMaximumIterations(ransac_times_);  // Number of RANSAC iterations
  align.setNumberOfSamples(3);  // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness(5);  // Number of nearest features to use
  align.setSimilarityThreshold(0.8f);  // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance(3.0f * voxel_size_model_);  // Inlier threshold
  align.setInlierFraction(match_threshold_);  // Required inlier fraction for accepting a pose hypothesis
  {
    ScopeTime t("Alignment");
    align.align(*object_aligned);
  }

  if (align.hasConverged()) {
    console::print_info("Inliers: %i/%i: %.2f\n", align.getInliers().size(), normal_model_->size(), align.getInliers().size()*1.0/normal_model_->size());
    transform = align.getFinalTransformation();
    return true;
  } else {
    pcl::console::print_error("Ransac Alignment failed!\n");
    return false;
  }
}

bool ModelMatch::IcpFineTuning(const PointCloud<PointXYZ>::Ptr &scene,
                               const Eigen::Matrix4f &initial_guess,
                               Eigen::Matrix4f &transform) {
  PointCloud<PointXYZ>::Ptr model_tf(new PointCloud<PointXYZ>);
  copyPointCloud(*normal_model_, *model_tf);
  transformPointCloud(*model_tf, *model_tf, initial_guess);

  PointCloud<PointXYZ>::Ptr registered (new PointCloud<PointXYZ>);
  IterativeClosestPoint<PointXYZ, PointXYZ> icp;
  icp.setMaximumIterations(30);
  icp.setMaxCorrespondenceDistance(1.0);
  icp.setInputTarget(scene);
  icp.setInputSource(model_tf);
  icp.align(*registered);
  if (icp.hasConverged()) {
    Eigen::Matrix4f icp_transformation = icp.getFinalTransformation();
    transform = icp_transformation * initial_guess;
    return true;
  } else {
    pcl::console::print_error("Icp Alignment failed!\n");
    return false;
  }
}

void ModelMatch::keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event, void* nothing) {
  if (event.getKeySym () == "space" && event.keyDown ()) {
    if (show_align_ >= 2) {
      show_align_ = 0;
    } else {
      show_align_ ++;
    }
  }
}

}  // namespace pose_estimation
