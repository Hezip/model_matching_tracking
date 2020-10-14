/*
 * @Copyright: 2020, Authors. All rights reserved.
 * @Description: A set of tools of PCL for pose estimation.
 * @Author: v_hezhenpeng
 * @Date: 2020-06-08
 */
#include <boost/filesystem.hpp>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>

#include "model_match/model_match.h"
#include "model_track/model_track.h"

typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ColorHandlerXYZ;

bool key_start = false;

void LoadSceneFolder(const std::string &scene_path,
    std::vector<std::string> &filenames, std::string &path) {
  std::string scene_folder_path = scene_path;
  boost::filesystem::directory_iterator itr(scene_folder_path);
  boost::filesystem::directory_iterator end;
  for (itr; itr != end; itr++) {
    if (itr->path().extension() != ".pcd") {
      continue;
    }
    filenames.push_back(itr->path().stem().string() + itr->path().extension().string());
    path = itr->path().parent_path().string() + "/";
  }
  sort(filenames.begin(), filenames.end(), std::less<std::string>());
}

void LoadCloudWithTime(const std::string path, const std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  pcl::console::print_highlight("Load File: %s\n", (path + filename).c_str());
  pcl::io::loadPCDFile<pcl::PointXYZ> (path + filename, *cloud);
  uint64_t time = std::stol(filename.substr(7,3) + filename.substr(12,3));
  cloud->header.stamp = time;
}

void
keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event,
                       void* nothing)
{
  if (event.getKeySym () == "space" && event.keyDown ())
    key_start = !key_start;
}

int main(int argc, char **argv) {
  //  load model
  pcl::PointCloud<pcl::PointXYZ>::Ptr load_object(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr load_scene(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr truck(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<std::string> filenames;
  std::string path;
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *load_object) < 0 ||
      pcl::io::loadPCDFile<pcl::PointXYZ>(argv[3], *truck) < 0) {
    pcl::console::print_error("Error loading object file!\n");
    return 1;
  }

  LoadSceneFolder(argv[2], filenames, path);
  std::shared_ptr<pose_estimation::ModelMatch> model_match_ptr = std::make_shared<pose_estimation::ModelMatch>();
  model_match_ptr->Initialization("../config.yaml", load_object);
  std::shared_ptr<pose_estimation::ModelTrack> model_track_ptr = std::make_shared<pose_estimation::ModelTrack>();
  Eigen::Matrix4f precise_tf = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  pcl::visualization::PCLVisualizer viewer("Tracking Demo");

  int i = 0;
  LoadCloudWithTime(path, filenames[i], load_scene);
  *scene = *load_scene;
  model_match_ptr->RemoveGround(scene, load_scene);
  model_match_ptr->MatchModel(load_scene, precise_tf);
  model_match_ptr->Visualization(load_scene, precise_tf);
  double time1 = load_scene->header.stamp * 1.0 / 1e3;
  model_track_ptr->Initialize(time1, precise_tf, load_object);

  int v1 (0);
  int v2 (1);
  viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
  viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
  viewer.addPointCloud(truck, ColorHandlerXYZ(truck, 255.0, 0.0, 255.0), "object_aligned", v2);
  viewer.addCube(-2,2,-5,5,-1.5,1.5,1,0,0,"OBB", v2);
//  viewer.addCube(-3.5,3.5,-1.5,1.5,-1.5,1.5,1,0,0,"OBB", v2);
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION , pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB", v2);
  viewer.addPointCloud(scene, ColorHandlerXYZ(scene, 0.0, 255.0, 0.0), "scene", v2);
  viewer.addPointCloud(scene, ColorHandlerXYZ(scene, 0.0, 255.0, 0.0), "scene_origin", v1);
  viewer.setShowFPS(true);
  // Register keyboard callback :
  viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);
  viewer.setCameraPosition(-50,0,0,1,0,0,0,0,1);
  viewer.setBackgroundColor(0.5,0.5,0.5, v2);
  viewer.setBackgroundColor(0.4,0.4,0.4, v1);
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "object_aligned", v2);

  // Adding text descriptions in each viewport
  viewer.addText ("Green: Original point cloud\n", 10, 15, 16, 0, 0, 0, "info_v1", v1);
  viewer.addText ("Green: Original point cloud\nRed: Aligned model", 10, 15, 16, 0, 0, 0, "info_v2", v2);


  while(!viewer.wasStopped() && i < filenames.size()){
    viewer.spinOnce();
    if(key_start){
      i++;
      LoadCloudWithTime(path, filenames[i], load_scene);

      pcl::VoxelGrid<pcl::PointXYZ> grid;
      float leaf = 0.05;
      grid.setLeafSize(leaf, leaf, leaf);
      grid.setInputCloud(load_scene);
      grid.filter(*load_scene);
      *scene = *load_scene;
      model_match_ptr->RemoveGround(scene, load_scene);
      model_track_ptr->TrackModel(load_scene);

      transform = model_track_ptr->get_transform();
      Eigen::Affine3f aff_tf;
      aff_tf.matrix() = transform;
      viewer.updatePointCloudPose ("object_aligned", aff_tf);
      viewer.updatePointCloud(scene, ColorHandlerXYZ(scene, 0.0, 255.0, 0.0), "scene");
      viewer.updatePointCloud(scene, ColorHandlerXYZ(scene, 0.0, 255.0, 0.0), "scene_origin");
      Eigen::Vector3f eulerAngle=transform.block<3,3>(0,0).eulerAngles(2,1,0);
      Eigen::AngleAxisf yawAngle(Eigen::AngleAxisf(eulerAngle(0),Eigen::Vector3f::UnitZ()));
      Eigen::Matrix3f rotation_matrix;
      rotation_matrix=yawAngle;
      transform.block<3,3>(0,0) = rotation_matrix;
      aff_tf.matrix() = transform;
      viewer.updateShapePose("OBB", aff_tf);
    }
  }
  return (0);
}
