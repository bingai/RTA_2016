// Project includes
#include "object_detection_3d_node/object_detection_3d_node.h"

// OpenCV
#include <opencv2/core/eigen.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <cv_bridge/cv_bridge.h>

// ROS includes
#include <pcl_conversions/pcl_conversions.h>    // This is required for PCL to ROS conversions
#include <eigen_conversions/eigen_msg.h>

// World model
#include <world_model_msgs/UpdateStatesObjects.h>

////////////////////////////////////////////////////////////////////////////////
// Conver g1::vision::Cylinder to world_model_msgs::Object
void cylinderToObject  (const g1::vision::Cylinder &cylinder, world_model_msgs::Object &object)
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
    ROS_INFO("Received reconstruction from world model");
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
  object_cls_msgs::GetObjectClass od2d_srv;





  // Construct object message
  res.objects.resize(object_cylinders_.size() + object_boxes_.size());
  for (size_t objId = 0; objId < object_cylinders_.size(); objId++)
    cylinderToObject(object_cylinders_[objId], res.objects[objId]);
  for (size_t objId = 0; objId < object_boxes_.size(); objId++)
    boxToObject(object_boxes_[objId], res.objects[object_cylinders_.size() + objId]);
  
  //----------------------------------------------------------------------------
  // Update world model
  
  world_model_msgs::UpdateStatesObjects upd_srv;
  upd_srv.request.objects_info = res.objects;
  
    // Send request
  if (!update_objects_client_.call (upd_srv))
  {
    ROS_ERROR("Tried to update world model, but failed to get response.");
  }
  else
  {
    ROS_INFO("Updated world model");
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Do the work!
bool ObjectDetection3dNode::process()
{
  if (!g1::vision::segmentTable (reconstructed_cloud_, tabletop_cloud_, table_plane_coefficients_, tabletop_polygon_, false))
  {
    ROS_ERROR("Could not detect table plane in the reconstructed cloud.");
    return false;
  }
  std::cout << std::endl;
  
  double startTime = ros::Time::now().toNSec();
  
  
  // Segment objects
  utl::map::Map segments;
  g1::vision::segmentObjects (tabletop_cloud_, segments, false);
  std::cout << std::endl;
  
  // Detect rotational objects
  std::vector<int> rotationalSegmentIds;
  std::vector<sym::RotationalSymmetry> objectSymmetries;
  g1::vision::rotationalObjectDetection(tabletop_cloud_, segments, table_plane_coefficients_, objectSymmetries, object_cylinders_, rotationalSegmentIds, false);
  // std::cout << std::endl;

  // // Fit boxes to the rest of the segments
  // std::vector<int> nonRotationalSegmentIds;
  // for (int i = 0; i < segments.size(); ++i)
  // {
  //   if (std::find(rotationalSegmentIds.begin(), rotationalSegmentIds.end(), i) == rotationalSegmentIds.end())
  //     nonRotationalSegmentIds.push_back(i);
  // }

  // g1::vision::fitBoundingBoxes(tabletop_cloud_, segments, table_plane_coefficients_, nonRotationalSegmentIds, object_boxes_, false);

  // // Get segment clouds
  // // TODO: get 'refined' segments
  // std::vector<pcl::PointCloud<PointNC>::Ptr> segmentClouds;
  // for (int i = 0; i < rotationalSegmentIds.size(); ++i)
  // {
  //   pcl::PointCloud<PointNC>::Ptr segmentCloud(new pcl::PointCloud<PointNC>);
  //   pcl::copyPointCloud<PointNC>(*tabletop_cloud_, segments[rotationalSegmentIds[i]], *segmentCloud);
  //   segmentClouds.push_back(segmentCloud);
  // }
  // for (int i = 0; i < nonRotationalSegmentIds.size(); ++i)
  // {
  //   pcl::PointCloud<PointNC>::Ptr segmentCloud(new pcl::PointCloud<PointNC>);
  //   pcl::copyPointCloud<PointNC>(*tabletop_cloud_, segments[nonRotationalSegmentIds[i]], *segmentCloud);
  //   segmentClouds.push_back(segmentCloud);
  // }

  // // Project segments

  // // cv::namedWindow("test", CV_WINDOW_AUTOSIZE);
  // // std::cout << segmentClouds.size() << std::endl;
  // // std::cout << rotationalSegmentIds.size() << std::endl;
  // // std::cout << nonRotationalSegmentIds.size() << std::endl;

  // bounding_boxes_2d_.resize(reconstruction_msg_.registered_views.size());
  // for (int i = 0; i < reconstruction_msg_.registered_views.size(); ++i)
  // {
  //   bounding_boxes_2d_[i].resize(segmentClouds.size());
  //   for (int j = 0; j < segmentClouds.size(); ++j)
  //   {
  //     Eigen::MatrixXf pose, K_e, d_e;
  //     pose = Eigen::MatrixXf::Map(reconstruction_msg_.registered_views[i].pose.data(), 4, 4);
  //     K_e = Eigen::MatrixXf::Map(reconstruction_msg_.registered_views[i].intrinsics_matrix.data(), 3, 3);
  //     d_e = Eigen::MatrixXf::Map(reconstruction_msg_.registered_views[i].distortion_coefficients.data(), reconstruction_msg_.registered_views[i].distortion_coefficients.size(), 1);
  //     cv::Mat K, d;
  //     cv::eigen2cv(K_e, K);
  //     cv::eigen2cv(d_e, d);
  //     Eigen::MatrixXf projPts = g1::vision::projectPointCloudToImageView(segmentClouds[j], K, d, pose.inverse());

  //     std::vector<int> bbCorners(4);
  //     g1::vision::boundingBox2DCorners(projPts, bbCorners[0], bbCorners[1], bbCorners[2], bbCorners[3]);


  //     // std::cout << projPts.rows() << " x " << projPts.cols() << std::endl;
  //     // for (int k = 0; k < 4; ++k) {
  //     //   std::cout << bbCorners[k] << "  ";
  //     // }
  //     // std::cout << std::endl;

  //     bounding_boxes_2d_[i][j] = bbCorners;
  //   }


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

  // }

    
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