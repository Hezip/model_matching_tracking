#include <iostream>
#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/shot_omp.h>
// #include <yaml-cpp/yaml.h>

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
//typedef pcl::SHOT352 FeatureT;
//typedef pcl::SHOTEstimationOMP<PointNT, PointNT, FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;


void
print4x4Matrix (const Eigen::Matrix4f & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}


// Align a rigid object to a scene with clutter and occlusions
int
main (int argc, char **argv)
{
  // // load yaml file
  // YAML::Node config = YAML::LoadFile("../config.yaml");
  // float leaf_size = config["leaf"].as<float>();
  float leaf_size = 0.05;

  // Point clouds
  PointCloudT::Ptr load_object (new PointCloudT);
  PointCloudT::Ptr object (new PointCloudT);
  PointCloudT::Ptr object_aligned (new PointCloudT);
  PointCloudT::Ptr load_scene (new PointCloudT);
  PointCloudT::Ptr scene (new PointCloudT);
  FeatureCloudT::Ptr object_features (new FeatureCloudT);
  FeatureCloudT::Ptr scene_features (new FeatureCloudT);
  
  // Get input object and scene
  if (argc != 3)
  {
    pcl::console::print_error ("Syntax is: %s object.pcd scene.pcd\n", argv[0]);
    return (1);
  }
  
  // Load object and scene
  pcl::console::print_highlight ("Loading point clouds...\n");
  if (pcl::io::loadPCDFile<PointNT> (argv[1], *load_object) < 0 ||
      pcl::io::loadPCDFile<PointNT> (argv[2], *load_scene) < 0)
  {
    pcl::console::print_error ("Error loading object/scene file!\n");
    return (1);
  }
  
  // Downsample
  pcl::console::print_highlight ("Downsampling...\n");
  pcl::VoxelGrid<PointNT> grid;
  const float leaf = leaf_size;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (load_scene);
  grid.filter (*scene);
  *object = *load_object;

  // Estimate normals for scene
  pcl::console::print_highlight ("Estimating scene normals...\n");
  pcl::NormalEstimationOMP<PointNT,PointNT> nest;
  nest.setRadiusSearch (leaf*3);
  nest.setInputCloud (scene);
  nest.compute (*scene);

  // Estimate features
  pcl::console::print_highlight ("Estimating features...\n");
  FeatureEstimationT fest;
  fest.setRadiusSearch (leaf*7);
  fest.setInputCloud (object);
  fest.setInputNormals (object);
  fest.compute (*object_features);
  fest.setInputCloud (scene);
  fest.setInputNormals (scene);
  fest.compute (*scene_features);

  // Perform alignment
  pcl::console::print_highlight ("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
  align.setInputSource (object);
  align.setSourceFeatures (object_features);
  align.setInputTarget (scene);
  align.setTargetFeatures (scene_features);
  align.setMaximumIterations (30000); // Number of RANSAC iterations
  align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (5); // Number of nearest features to use
  align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (2.0f * leaf); // Inlier threshold
  align.setInlierFraction (0.7f); // Required inlier fraction for accepting a pose hypothesis
  {
    pcl::ScopeTime t("Alignment");
    align.align (*object_aligned);
  }
  
  if (align.hasConverged ())
  {
    //ICP
    PointCloudT::Ptr object_tf (new PointCloudT);
    Eigen::Matrix4f sac_transformation = align.getFinalTransformation ();
    pcl::transformPointCloud (*load_object, *object_tf, sac_transformation);
    std::cout << "--- ICP ---------" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_res(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr object_res(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*load_scene, *scene_res);
    pcl::copyPointCloud(*object_tf, *object_res);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations (30);
    icp.setMaxCorrespondenceDistance (3.0f * leaf);
    icp.setInputTarget (scene_res);
    icp.setInputSource (object_res);
    pcl::PointCloud<pcl::PointXYZ>::Ptr registered (new pcl::PointCloud<pcl::PointXYZ>);
    icp.align (*registered);
    if (icp.hasConverged ())
    {
      std::cout << "ICP Aligned!" << std::endl;
    }
    else
    {
      std::cout << "ICP Not Aligned!" << std::endl;
    }

    printf ("\n");
    Eigen::Matrix4f icp_transformation = icp.getFinalTransformation ();
    Eigen::Matrix4f transformation = icp_transformation * sac_transformation;

    print4x4Matrix(transformation);
    pcl::transformPointCloud (*load_object, *load_object, transformation);

    pcl::console::print_info ("\n");
    pcl::console::print_info ("Inliers: %i/%i: %.2f\n", align.getInliers().size(), object->size(), align.getInliers().size()*1.0/object->size());

    // Show alignment
    pcl::visualization::PCLVisualizer visu("Alignment");
    visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
    visu.addPointCloud (load_object, ColorHandlerT (load_object, 0.0, 0.0, 255.0), "object_aligned");
    visu.spin ();
  }
  else
  {
    pcl::console::print_error ("Alignment failed!\n");
    return (1);
  }
  
  return (0);
}