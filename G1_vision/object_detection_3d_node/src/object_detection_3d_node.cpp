// Project includes
#include "object_detection_3d_node/object_detection_3d_node.h"

// OpenCV
#include <opencv2/core/eigen.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <cv_bridge/cv_bridge.h>

// ROS includes
#include <pcl_conversions/pcl_conversions.h>    // This is required for PCL to ROS conversions
#include <eigen_conversions/eigen_msg.h>


////////////////////////////////////////////////////////////////////////////////
// Conver g1::vision::Cylinder to world_model_msgs::Object
void cylinderToObject  (const g1::vision::Cylinder &cylinder, const g1::vision::Box &handle_box, world_model_msgs::Object &object)
{
  object.primitives.resize(1);
  object.primitive_poses.resize(1);
  
  // Solid primitive
  shape_msgs::SolidPrimitive sp;
  sp.type = sp.CYLINDER;
  sp.dimensions.resize(2);
  sp.dimensions[sp.CYLINDER_HEIGHT] = cylinder.height_;
  sp.dimensions[sp.CYLINDER_RADIUS] = cylinder.radius_;
  object.primitives[0] = sp;
  
  // Pose
  tf::poseEigenToMsg(cylinder.pose_.cast<double>(), object.primitive_poses[0]);  
  
  // Id
  object.id = cylinder.id_;
  
  // Add handle if it is present
  if (handle_box.id_ != "")
  {
    object.primitives.resize(2);
    object.primitive_poses.resize(2);
        
    // Solid primitive
    shape_msgs::SolidPrimitive sp_handle;
    sp_handle.type = sp_handle.BOX;
    sp_handle.dimensions.resize(3);
    sp_handle.dimensions[sp_handle.BOX_X] = handle_box.size_(0);
    sp_handle.dimensions[sp_handle.BOX_Y] = handle_box.size_(1);
    sp_handle.dimensions[sp_handle.BOX_Z] = handle_box.size_(2);
    object.primitives[1] = sp_handle;
    
    // Pose
    tf::poseEigenToMsg(handle_box.pose_.cast<double>(), object.primitive_poses[1]);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Conver g1::vision::Box to world_model_msgs::Object
void boxToObject  (const g1::vision::Box &box, world_model_msgs::Object &object)
{
  object.primitives.resize(1);
  object.primitive_poses.resize(1);
  
  // Solid primitive
  shape_msgs::SolidPrimitive sp;
  sp.type = sp.BOX;
  sp.dimensions.resize(3);
  sp.dimensions[sp.BOX_X] = box.size_(0);
  sp.dimensions[sp.BOX_Y] = box.size_(1);
  sp.dimensions[sp.BOX_Z] = box.size_(2);
  object.primitives[0] = sp;
  
  // Pose
  tf::poseEigenToMsg(box.pose_.cast<double>(), object.primitive_poses[0]);
  
  // Id
  object.id = box.id_;
}

////////////////////////////////////////////////////////////////////////////////
// Constructor
ObjectDetection3dNode::ObjectDetection3dNode ()
  : n_ ()
  , service_ (n_.advertiseService ("object_detection_3d_service", &ObjectDetection3dNode::replyToRequest, this))
  , update_objects_client_ (n_.serviceClient<world_model_msgs::UpdateStatesObjects>("/world_model/update_states_objects"))
  , get_reconstruction_client_ (n_.serviceClient<world_model_msgs::ReconstructionQuery>("/world_model/get_reconstruction"))
  , object_recognition_2d_client_ (n_.serviceClient<object_cls_msgs::GetObjectClass>("get_object_class"))
  , visual_tools_ (new rviz_visual_tools::RvizVisualTools("/base","/object_detection_3d"))
  , tabletop_cloud_publisher_ (n_.advertise<sensor_msgs::PointCloud2> ("tabletop_cloud", 1000))
  , tabletop_cloud_msg_ ()
  , tabletop_polygon_publisher_ (n_.advertise<geometry_msgs::PolygonStamped> ("tabletop_polygon", 1000))
  , tabletop_polygon_msg_()
  , object_msg_ ()
  , reconstructed_cloud_ (new pcl::PointCloud<PointNC>)
  , tabletop_cloud_ (new pcl::PointCloud<PointNC>)
  , table_plane_coefficients_ ()
  , tabletop_polygon_ (new pcl::PointCloud<PointNC>)
  , object_cylinders_ ()
  , object_boxes_ ()
{
  ROS_INFO("Waiting for world model services: ");
  ROS_INFO("  %s", update_objects_client_.getService().c_str());
  ROS_INFO("  %s", get_reconstruction_client_.getService().c_str());
  update_objects_client_.waitForExistence();
  get_reconstruction_client_.waitForExistence();
  ROS_INFO("Ready to process.");
}

////////////////////////////////////////////////////////////////////////////////
// Process service request
bool ObjectDetection3dNode::replyToRequest  ( object_detection_3d_node::ObjectDetection3dServiceRequest  &req,
                                              object_detection_3d_node::ObjectDetection3dServiceResponse &res
         )
{ 
  std::cout << "----------------------------------------" << std::endl;
  std::cout << "Received request!" << std::endl;

  //----------------------------------------------------------------------------
  // Get input
  world_model_msgs::ReconstructionQuery rec_srv;
  rec_srv.request.id = req.reconstruction_id;
  if (!get_reconstruction_client_.call (rec_srv))
  {
    ROS_ERROR("Tried to fetch reconstruction from world model, but failed to get response.");
    return false;
  }
  if (!rec_srv.response.success) {
    ROS_ERROR("Requested reconstruction does not exist in world model.");
    return false;
  }
  else
  {
    ROS_INFO("Received reconstruction from world model.");
  }

  reconstruction_msg_ = rec_srv.response.reconstruction;
  
  pcl::fromROSMsg<PointNC> (reconstruction_msg_.model_point_cloud, *reconstructed_cloud_);
  // pcl::fromROSMsg<PointNC> (req.reconstruction.model_point_cloud, *reconstructed_cloud_);

  std::string frameId = reconstruction_msg_.model_point_cloud.header.frame_id;
  // std::string frameId = req.reconstruction.model_point_cloud.header.frame_id;
  
  //----------------------------------------------------------------------------
  // Process
  if (!process())
    return false;
  
  //----------------------------------------------------------------------------
  // Construct result messages
  
  // Construct tabletop cloud message
  pcl::toROSMsg (*tabletop_cloud_, tabletop_cloud_msg_);
  tabletop_cloud_msg_.header.frame_id = frameId;
  
  // Construct table polygon message
  tabletop_polygon_msg_.polygon.points.resize (tabletop_polygon_->size ());
  for (size_t pointId = 0; pointId < tabletop_polygon_->size (); pointId++)
  {
    tabletop_polygon_msg_.polygon.points[pointId].x = tabletop_polygon_->points[pointId].x;
    tabletop_polygon_msg_.polygon.points[pointId].y = tabletop_polygon_->points[pointId].y;
    tabletop_polygon_msg_.polygon.points[pointId].z = tabletop_polygon_->points[pointId].z;
  }
  tabletop_polygon_msg_.header.frame_id = frameId;
  
  // Get 2D recognition result
  object_cls_msgs::GetObjectClass cls_srv;

  cls_srv.request.task = cls_srv.request.TASK_TYPE_DISH;

  cls_srv.request.images.resize(bounding_boxes_2d_.size());
  cls_srv.request.bbox_lists.resize(bounding_boxes_2d_.size());
  for (int i = 0; i < bounding_boxes_2d_.size(); ++i)
  {
    cls_srv.request.images[i] = reconstruction_msg_.registered_views[i].image;
    cls_srv.request.bbox_lists[i].bbox_list.resize(bounding_boxes_2d_[i].size());
    for (int j = 0; j < bounding_boxes_2d_[i].size(); ++j)
    {
      cls_srv.request.bbox_lists[i].bbox_list[j].box = bounding_boxes_2d_[i][j];
      cls_srv.request.bbox_lists[i].bbox_list[j].id = objects_msg_[j].id;
    }
  }

  if (!object_recognition_2d_client_.call (cls_srv))
  {
    ROS_ERROR("Tried to query 2D classifier service, but failed to get response.");
    // return false;
  }
  else
  {
    ROS_INFO("Received 2D classification results.");
    // Update object ids according to received labels
    for (int i = 0; i < objects_msg_.size(); ++i)
    {
      if (!cls_srv.response.predictions[i].empty())
      {
        objects_msg_[i].id = cls_srv.response.predictions[i];
      }
    }
  }
  
  //----------------------------------------------------------------------------
  // Update world model
  
  world_model_msgs::UpdateStatesObjects upd_srv;

  // Delete all objects
  upd_srv.request.operation = upd_srv.request.DELETE;
  if (!update_objects_client_.call (upd_srv))
  {
    ROS_ERROR("Tried to update world model, but failed to get response.");
  }
  else
  {
    ROS_INFO("Deleted previous objects from the world model");
  }

  // Update with newly detected objects
  upd_srv.request.operation = upd_srv.request.UPDATE;
  upd_srv.request.objects_info = objects_msg_;
  if (!update_objects_client_.call (upd_srv))
  {
    ROS_ERROR("Tried to update world model, but failed to get response.");
  }
  else
  {
    ROS_INFO("Updated world model with detected objects");
  }

  // Update response
  res.objects = objects_msg_;

  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Do the work!
bool ObjectDetection3dNode::process()
{
  //----------------------------------------------------------------------------
  // Segment the table
  if (!g1::vision::segmentTable (reconstructed_cloud_, tabletop_cloud_, table_plane_coefficients_, tabletop_polygon_, false))
  {
    ROS_ERROR("Could not detect table plane in the reconstructed cloud.");
    return false;
  }
  std::cout << std::endl;
  
  double startTime = ros::Time::now().toNSec();
  
  //----------------------------------------------------------------------------
  // Segment objects
  utl::map::Map segments, segments_unused;
  g1::vision::segmentObjects (tabletop_cloud_, segments, segments_unused, false);
  std::cout << std::endl;

  //----------------------------------------------------------------------------
  // Detect rotational objects
  std::vector<std::vector<int> > segmentsUsedForRotationalObjects;
  std::vector<sym::RotationalSymmetry> rotationalObjectSymmetries;
  
  g1::vision::rotationalObjectDetection ( tabletop_cloud_,
                                          segments,
                                          table_plane_coefficients_,
                                          rotationalObjectSymmetries,
                                          object_cylinders_,
                                          segmentsUsedForRotationalObjects,
                                          false );
  
  // Get indices of points used for each rotationally symmetric object
  utl::map::Map rotationalObjectSegments (segmentsUsedForRotationalObjects.size());
  for (size_t objId = 0; objId < segmentsUsedForRotationalObjects.size(); objId++)
  {
    std::vector<int> indices;
    for (size_t segIdIt = 0; segIdIt < segmentsUsedForRotationalObjects[objId].size(); segIdIt++)
    {
      int segId = segmentsUsedForRotationalObjects[objId][segIdIt];
      indices.insert(indices.end(), segments[segId].begin(), segments[segId].end());
    }
    rotationalObjectSegments[objId] = indices;
  }
  
  // Detect handles
  std::vector<g1::vision::Box> rotationalObjectHandleBoxes;
  g1::vision::handleDetection ( tabletop_cloud_,
                                table_plane_coefficients_,
                                rotationalObjectSegments,
                                rotationalObjectSymmetries,
                                object_cylinders_,
                                rotationalObjectHandleBoxes,
                                false  );
    
  //----------------------------------------------------------------------------
  // Fit boxes to the rest of the segments
  
  // Get segments not used for rotational objects
  std::vector<int> nonRotationalSegmentIds;
  for (size_t segId = 0; segId < segments.size(); segId++)
  {
    bool used = false;
    
    for (size_t rotObjId = 0; rotObjId < segmentsUsedForRotationalObjects.size(); rotObjId++)
      for (size_t rotObjSegIdIt = 0; rotObjSegIdIt < segmentsUsedForRotationalObjects[rotObjId].size(); rotObjSegIdIt++)
        if (std::find(segmentsUsedForRotationalObjects[rotObjId].begin(), segmentsUsedForRotationalObjects[rotObjId].end(), segId) != segmentsUsedForRotationalObjects[rotObjId].end())
          used = true;

    if (!used)
      nonRotationalSegmentIds.push_back(segId);
  }
  
  // Fit boxes
  g1::vision::fitBoundingBoxes(tabletop_cloud_, segments, table_plane_coefficients_, nonRotationalSegmentIds, object_boxes_, false);
  
  //----------------------------------------------------------------------------
  // Construct object message
  objects_msg_.resize(object_cylinders_.size() + object_boxes_.size());
  for (size_t objId = 0; objId < object_cylinders_.size(); objId++)
    cylinderToObject(object_cylinders_[objId], rotationalObjectHandleBoxes[objId], objects_msg_[objId]);
  for (size_t objId = 0; objId < object_boxes_.size(); objId++)
    boxToObject(object_boxes_[objId], objects_msg_[object_cylinders_.size() + objId]);  
  
  //----------------------------------------------------------------------------
  // Get 2D bounding box for classification

  // Get pointclouds for rotational objects
  std::vector<pcl::PointCloud<PointNC>::Ptr> rotationalObjectClouds;
  rotationalObjectClouds.resize(segmentsUsedForRotationalObjects.size());
  for (size_t objId = 0; objId < segmentsUsedForRotationalObjects.size(); objId++)
  {
    rotationalObjectClouds[objId].reset (new pcl::PointCloud<PointNC>);
    
    std::vector<int> indices;
    for (size_t segIdIt = 0; segIdIt < segmentsUsedForRotationalObjects[objId].size(); segIdIt++)
    {
      int segId = segmentsUsedForRotationalObjects[objId][segIdIt];
      indices.insert(indices.end(), segments[segId].begin(), segments[segId].end());
    }
    
    pcl::copyPointCloud<PointNC>(*tabletop_cloud_, indices, *rotationalObjectClouds[objId]);
  }
  
  // Get the rest segment clouds
  std::vector<pcl::PointCloud<PointNC>::Ptr> segmentClouds;
  segmentClouds = rotationalObjectClouds;
  for (int i = 0; i < nonRotationalSegmentIds.size(); ++i)
  {
    pcl::PointCloud<PointNC>::Ptr segmentCloud(new pcl::PointCloud<PointNC>);
    pcl::copyPointCloud<PointNC>(*tabletop_cloud_, segments[nonRotationalSegmentIds[i]], *segmentCloud);
    segmentClouds.push_back(segmentCloud);
  }

  // std::cout << "clouds: " << segmentClouds.size() << std::endl;
  // std::cout << "objects: " << objects_msg_.size() << std::endl;

  // Project segments

  // std::cout << "all segments: " << segmentClouds.size() << std::endl;
  // std::cout << "rot: " << rotationalObjectClouds.size() << std::endl;
  // std::cout << "non rot: " << nonRotationalSegmentIds.size() << std::endl;

  bounding_boxes_2d_.resize(reconstruction_msg_.registered_views.size());
  for (int i = 0; i < reconstruction_msg_.registered_views.size(); ++i)
  {
    bounding_boxes_2d_[i].resize(segmentClouds.size());
    for (int j = 0; j < segmentClouds.size(); ++j)
    {
      Eigen::MatrixXf pose, K_e, d_e;
      pose = Eigen::MatrixXf::Map(reconstruction_msg_.registered_views[i].pose.data(), 4, 4);
      K_e = Eigen::MatrixXf::Map(reconstruction_msg_.registered_views[i].intrinsics_matrix.data(), 3, 3);
      d_e = Eigen::MatrixXf::Map(reconstruction_msg_.registered_views[i].distortion_coefficients.data(), reconstruction_msg_.registered_views[i].distortion_coefficients.size(), 1);
      cv::Mat K, d;
      cv::eigen2cv(K_e, K);
      cv::eigen2cv(d_e, d);
      Eigen::MatrixXf projPts = g1::vision::projectPointCloudToImageView(segmentClouds[j], K, d, pose.inverse());

      std::vector<int> bbCorners(4);
      g1::vision::boundingBox2DCorners(projPts, bbCorners[0], bbCorners[1], bbCorners[2], bbCorners[3]);

      // std::cout << projPts.rows() << " x " << projPts.cols() << std::endl;
      // for (int k = 0; k < 4; ++k) {
      //   std::cout << bbCorners[k] << "  ";
      // }
      // std::cout << std::endl;

      bounding_boxes_2d_[i][j] = bbCorners;
    }

    // cv::namedWindow("test", CV_WINDOW_AUTOSIZE);
    // cv::Mat img = cv_bridge::toCvCopy(reconstruction_msg_.registered_views[i].image)->image;
    // for (int j = 0; j < segmentClouds.size(); ++j)
    // {
    //   cv::Point2i pt1, pt2;
    //   pt1.x = bounding_boxes_2d_[i][j][0];
    //   pt1.y = bounding_boxes_2d_[i][j][1];
    //   pt2.x = bounding_boxes_2d_[i][j][2];
    //   pt2.y = bounding_boxes_2d_[i][j][3];
    //   cv::rectangle(img, pt1, pt2, 0);
    // }

    // cv::imshow("test", img);
    // cv::waitKey(0);
    // cv::destroyWindow("test");
  }
    
  // Report total time
  std::cout << "Total processing time: " << ((ros::Time::now().toNSec() - startTime) * 1e-9) << " seconds" << std::endl;  
  
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Run
void ObjectDetection3dNode::run()
{
  ros::Rate loop_rate (1);
  while (ros::ok ())
  {
    // Publish all required data to topics
    tabletop_cloud_publisher_.publish (tabletop_cloud_msg_);
    tabletop_polygon_publisher_.publish (tabletop_polygon_msg_);
    
//     // Publish detected cylinders
//     for (size_t objectId = 0; objectId < object_cylinders_.size (); objectId++)
//     {
//       Eigen::Affine3f pose = object_cylinders_[objectId].pose_;
//       float radius = object_cylinders_[objectId].radius_;
//       float height = object_cylinders_[objectId].height_;
//       std::string object_id = object_cylinders_[objectId].id_;
//       
//       pose.translate (pose.rotation().col(2) * height / 2);
//       radius = radius * 2;
//       
//       visual_tools_->publishCylinder  ( pose.cast<double> (),
//                                         rviz_visual_tools::BLUE,
//                                         static_cast<double> (height),
//                                         static_cast<double> (radius),
//                                         object_id );
//     }
    
    // Spin once
    ros::spinOnce ();
    
    // Sleep
    loop_rate.sleep();
  }
}

////////////////////////////////////////////////////////////////////////////////
int main (int argc, char **argv)
{
  // Initialize ROS
  ros::init (argc, argv, "object_detection_3d_node");
  
  ObjectDetection3dNode od3d;
  od3d.run ();

  return 0;
}
