/*
 * @Copyright: 2020, Authors. All rights reserved.
 * @Description: A set of tools of PCL for pose estimation.
 * @Author: v_hezhenpeng
 * @Date: 2020-06-08
 */
#include "model_match/model_match.h"
#include "model_match/common.hpp"

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
#include <chrono>

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
  ransac_times_ = config["ransac"]["ransac_times"].as<float>();
  match_threshold_ = config["ransac"]["match_threshold"].as<float>();
  enable_icp_ = config["enable_icp"].as<bool>();
  has_initial_guess_ = config["has_initial_guess"].as<bool>();
  method_ = config["method"].as<std::string>();
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

  console::print_highlight("Estimating normals...\n");
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

  console::print_highlight("Starting alignment RANSAC...\n");
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

bool ModelMatch::TeaserPoseEstimation(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scene,
                          Eigen::Matrix4f &transform) {
  PointCloud<PointNormal>::Ptr normal_scene(new PointCloud<PointNormal>);
  copyPointCloud(*scene, *normal_scene);

  FeatureCloudT::Ptr feature_scene (new FeatureCloudT);
  FeatureExtraction(voxel_size_scene_, normal_scene, feature_scene);

  // Convert to teaser point cloud
  teaser::PointCloud tgt_cloud;
  for (size_t i = 0; i < normal_scene->size(); ++i) {
    tgt_cloud.push_back({normal_scene->points[i].x, normal_scene->points[i].y, normal_scene->points[i].z});
  }
  teaser::PointCloud src_cloud;
  for (size_t i = 0; i < normal_model_->size(); ++i) {
    src_cloud.push_back({normal_model_->points[i].x, normal_model_->points[i].y, normal_model_->points[i].z});
  }

  teaser::Matcher matcher;
  auto correspondences = matcher.calculateCorrespondences(
      src_cloud, tgt_cloud, *feature_model_, *feature_scene, false, true, false, 0.95);

  console::print_highlight("Starting alignment Teaser++...\n");
  // Run TEASER++ registration
  // Prepare solver parameters
  teaser::RobustRegistrationSolver::Params params;
  params.noise_bound = 0.01;
  params.cbar2 = 1;
  params.estimate_scaling = false;
  params.rotation_max_iterations = 500;
  params.rotation_gnc_factor = 1.4;
  params.rotation_estimation_algorithm =
      teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
  params.rotation_cost_threshold = 1e-6;

  // Solve with TEASER++
  teaser::RobustRegistrationSolver solver(params);
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  solver.solve(src_cloud, tgt_cloud, correspondences);
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

  auto solution = solver.getSolution();
  transform.block<3,3>(0,0) = solution.rotation.cast<float>();
  transform.block<3,1>(0,3) = solution.translation.cast<float>();
  transform(3,3) = 1.0;
  return true;
}

bool ModelMatch::IcpFineTuning(const PointCloud<PointXYZ>::Ptr &scene,
                               const Eigen::Matrix4f &initial_guess,
                               Eigen::Matrix4f &transform) {
  console::print_highlight("Starting ICP...\n");
  PointCloud<PointXYZ>::Ptr model_tf(new PointCloud<PointXYZ>);
  copyPointCloud(*normal_model_, *model_tf);
  transformPointCloud(*model_tf, *model_tf, initial_guess);

  PointCloud<PointXYZ>::Ptr registered (new PointCloud<PointXYZ>);
  IterativeClosestPoint<PointXYZ, PointXYZ> icp;
  icp.setMaximumIterations(100);
  icp.setMaxCorrespondenceDistance(0.8);
  icp.setInputTarget(scene);
  icp.setInputSource(model_tf);
  icp.align(*registered);
  if (icp.hasConverged()) {
    Eigen::Matrix4f icp_transformation = icp.getFinalTransformation();
    transform = icp_transformation * initial_guess;

    console::print_highlight("Icp alignment fitness score: %.2f / 1.0\n", IcpFitnessScore(scene, model_tf, icp_transformation, 0.2));
    return true;
  } else {
    pcl::console::print_error("Icp alignment failed!\n");
    return false;
  }
}

double ModelMatch::IcpFitnessScore(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& target,
                                      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& source,
                                      const Eigen::Matrix4f& relpose, double max_range) {
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_(new pcl::search::KdTree<pcl::PointXYZ>());
  tree_->setInputCloud(target);

  double fitness_score = 0.0;
  // Transform the input dataset using the final transformation
  pcl::PointCloud<pcl::PointXYZ> input_transformed;
  pcl::transformPointCloud (*source, input_transformed, relpose.cast<float>());

  std::vector<int> nn_indices (1);
  std::vector<float> nn_dists (1);

  // For each point in the source dataset
  int nr = 0;
  for (size_t i = 0; i < input_transformed.points.size (); ++i) {
    // Find its nearest neighbor in the target
    tree_->nearestKSearch (input_transformed.points[i], 1, nn_indices, nn_dists);
    // Deal with occlusions (incomplete targets)
    if (nn_dists[0] <= max_range) {
      // Add to the fitness score
      fitness_score += nn_dists[0];
      nr++;
    }
  }

  if (nr > 0)
//    return (fitness_score / nr);
    return (nr*1.0/source->size());
  else
    return (std::numeric_limits<double>::max ());
}

double weight(double a, double max_x, double min_y, double max_y, double x)  {
  a = 20.0;
  max_x = 0.5;
  min_y = 0.1 * 0.1;
  max_y = 5 * 5;
  double y = (1.0 - std::exp(-a * x)) / (1.0 - std::exp(-a * max_x));
  return min_y + (max_y - min_y) * y;
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

bool ModelMatch::MatchModel(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scene, Eigen::Matrix4f &transform) {
  Eigen::Matrix4f rough_tf = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f precise_tf = Eigen::Matrix4f::Identity();

  if(has_initial_guess_) {
    //if we have initial guess;
    Eigen::Matrix4d initial_guess;
    std::string file_path = "../config/initial_guess.txt";
    loadMatrix(file_path, initial_guess);
    rough_tf = initial_guess.cast<float>();
  } else {
    //we use ransac of teaser to measure initial guess
    if(method_ == "ransac") {
      if (!RansacPoseEstimation(scene, rough_tf)) {
        return false;
      }
    } else {
      if (!TeaserPoseEstimation(scene, rough_tf)) {
        return false;
      }
    }
  }

  if (enable_icp_) {
    //we want to use icp to fine tune our transfrom
    if (!IcpFineTuning(scene, rough_tf, precise_tf)) {
      return false;
    }
  } else {
    precise_tf = rough_tf;
  }

  transform = precise_tf;
  return true;
}

}  // namespace pose_estimation
