// Current package includes
#include <object_detection_3d/object_detection_3d.h>

#include <sisyphus/box_fitting.hpp>


////////////////////////////////////////////////////////////////////////////////
// Visualize a box
void g1::vision::showBox  ( pcl::visualization::PCLVisualizer &visualizer,
                            const g1::vision::Box &box,
                            const std::string &id,
                            utl::pclvis::Color color,
                            const float opacity
                          )
{
  // Add coordinate system
  visualizer.addCoordinateSystem(box.size_.norm() / 2, box.pose_, id + "_frame");

  // Add bounding box
  Eigen::Quaternionf q(box.pose_.linear());
  visualizer.addCube(box.pose_.translation(), q, box.size_(0), box.size_(1), box.size_(2), id);
  utl::pclvis::setShapeRenderProps(visualizer, id, color, opacity);
}

////////////////////////////////////////////////////////////////////////////////
// Run
bool g1::vision::fitBoundingBoxes ( const pcl::PointCloud<PointNC>::ConstPtr &tabletop_cloud,
                                    const utl::map::Map &segments,
                                    const Eigen::Vector4f &table_plane_coefficients,
                                    const std::vector<int> &box_segments_ind,
                                    std::vector<Box> &bounding_boxes,
                                    bool visualize
                                  )
{
  //----------------------------------------------------------------------------
  // Fit boxes
  //----------------------------------------------------------------------------
  
  // Generate segment pointclouds
  std::vector<pcl::PointCloud<PointNC>::Ptr> segmentClouds (segments.size());
  for (int segId = 0; segId < segments.size(); ++segId)
  {
    segmentClouds[segId].reset(new pcl::PointCloud<PointNC>);
    pcl::copyPointCloud<PointNC>(*tabletop_cloud, segments[segId], *segmentClouds[segId]);
  }
  
  // Fit boxes
  bounding_boxes.resize(box_segments_ind.size());
  for (int segIdIt = 0; segIdIt < box_segments_ind.size(); ++segIdIt)
  {
    int segId = box_segments_ind[segIdIt];
    // fitMinimumVolumeBoundingBoxConstrained<PointNC> ( segmentClouds[segId],
    //                                                   table_plane_coefficients,
    //                                                   bounding_boxes[segIdIt].pose_,
    //                                                   bounding_boxes[segIdIt].size_
    //                                                 );
    // int num_inliers;
    // float mean_inlier_dist;
    // fitMaximumContactBoundingBoxConstrainedIterative<PointNC> ( segmentClouds[segId],
    //                                                             table_plane_coefficients,
    //                                                             bounding_boxes[segIdIt].pose_,
    //                                                             bounding_boxes[segIdIt].size_,
    //                                                             num_inliers,
    //                                                             mean_inlier_dist,
    //                                                             Eigen::Vector3f::UnitX(),
    //                                                             0.025
    //                                                           );
    fitMaximumContactBoundingBoxConstrained<PointNC>( segmentClouds[segId],
                                                      table_plane_coefficients,
                                                      bounding_boxes[segIdIt].pose_,
                                                      bounding_boxes[segIdIt].size_,
                                                      0.025
                                                    );
    bounding_boxes[segIdIt].id_ = "box_" + std::to_string(segIdIt);
  }
  
  //----------------------------------------------------------------------------
  // Visualize
  //----------------------------------------------------------------------------
  
  if (!visualize)
    return true;
  
  pcl::visualization::PCLVisualizer visualizer;
  visualizer.setCameraPosition (  2.5, 0.0, 2.0,   // camera position
                                  0.0, 0.0, 0.0,   // viewpoint
                                  0.0, 0.0, 1.0,   // normal
                                  0.0);            // viewport
  visualizer.setBackgroundColor (utl::pclvis::bgColor.r, utl::pclvis::bgColor.g, utl::pclvis::bgColor.b);
  
  // Add pointcloud
  utl::pclvis::showPointCloudColor<PointNC>(visualizer, tabletop_cloud, "cloud");

  // Add bounding boxes
  for (int boxId = 0; boxId < bounding_boxes.size(); ++boxId)
  {
    std::string id = "box_" + std::to_string(boxId);
    utl::pclvis::Color color = utl::pclvis::getGlasbeyColor(boxId);
    showBox(visualizer, bounding_boxes[boxId], id, color, 0.5);
  }
  
  // Add robot frame
  visualizer.addCoordinateSystem(0.5);

  // Spin!
  visualizer.spin();
  
  return true;
}
