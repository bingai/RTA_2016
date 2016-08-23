// Current package includes
#include "object_detection_3d/object_detection_3d.h"

#include <sisyphus/box_detection.hpp>

#include <pcl/filters/box_clipper3D.h>

bool g1::vision::segmentFridgeInterior( const pcl::PointCloud<PointNC>::ConstPtr &scene_cloud,
                                        pcl::PointCloud<PointNC>::Ptr &fridge_cloud,
                                        g1::vision::Box &fridge_box,
                                        const bool visualize
                                      )
{
  // Segmenting parameters
  int min_cluster_size = 1000;
  int ransac_iter = 1000;
  float max_plane_dist = 0.015;
  float max_seg_dist = 0.2;
  float rg_radius = 0.02;
  float rg_normal_var_thresh = 100*30.0*M_PI/180.0;
  float rg_angle_thresh = 30.0*M_PI/180.0;
  float angle_tol = 5.0*M_PI/180.0;
  float max_angle_diff = 15.0*M_PI/180.0;
  float angle_diff_cluster = 10.0*M_PI/180.0;

  // Grouping parameters
  float max_side_dist = 0.1;
  float ratio_thresh = 0.05;
  float range_thresh = 0.40;
  bool outside = false;
  bool require_cliques = false;
  bool brute_force = false;

  // Detect boxes
  std::vector<::Box<PointNC> > boxes = detectBoxes<PointNC>(scene_cloud, min_cluster_size, ransac_iter, max_plane_dist, max_seg_dist, rg_radius, rg_normal_var_thresh, rg_angle_thresh, angle_tol, max_angle_diff, angle_diff_cluster, max_side_dist, ratio_thresh, range_thresh, outside, require_cliques, brute_force);

  if (boxes.empty())
  {
    std::cout << "[g1::vision::segmentFridgeInterior] no boxes detected!" << std::endl;
    return false;
  }

  // Get the largest box
  float max_v = 0.0;
  int max_ind;
  for (int i = 0; i < boxes.size(); ++i)
  {
    float v = boxes[i].size.prod();
    if (max_v < v)
    {
      max_v = v;
      max_ind = i;
    }
  }

  renameBoxAxes<PointNC>(boxes[max_ind], Eigen::Vector3f::Zero(), Eigen::Vector3f::UnitZ());

  fridge_box.pose_.matrix() = boxes[max_ind].pose;
  fridge_box.size_ = boxes[max_ind].size;
  fridge_box.id_ = "fridge_interior";

  // Clip the interior cloud
  float cut_off_dist = 0.025;
  std::vector<Eigen::Vector4f> constr;
  for (int i = 0; i < boxes[max_ind].planes.size(); ++i)
  {
    Eigen::Vector4f plane = boxes[max_ind].planes[i];
    plane(3) -= cut_off_dist;
    constr.push_back(plane);
  }
  std::vector<int> inlier_ind;
  pointIndicesInConvexPolytope<PointNC>(scene_cloud, constr, inlier_ind);
  fridge_cloud = extractPointCloudFromIndices<PointNC>(scene_cloud, inlier_ind, false, false);

  if (!visualize)
    return true;
  
  pcl::visualization::PCLVisualizer visualizer;
  visualizer.setCameraPosition (  2.5, 0.0, 2.0,   // camera position
                                  0.0, 0.0, 0.0,   // viewpoint
                                  0.0, 0.0, 1.0,   // normal
                                  0.0);            // viewport
  visualizer.setBackgroundColor (utl::pclvis::bgColor.r, utl::pclvis::bgColor.g, utl::pclvis::bgColor.b);
  
  // Add pointcloud
  utl::pclvis::showPointCloudColor<PointNC>(visualizer, fridge_cloud, "cloud");

  // Add bounding boxes
  std::string id = "fridge_interior";
  utl::pclvis::Color color = utl::pclvis::getGlasbeyColor(0);
  showBox(visualizer, fridge_box, id, color, 0.5);
  
  // Add robot frame
  visualizer.addCoordinateSystem(0.5);

  // Spin!
  visualizer.spin();

  return true;
}