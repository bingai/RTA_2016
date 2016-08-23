/*
 * =====================================================================================
 *
 *       Filename:  world_model.cpp
 *
 *    Description:  world model node to updates the states
 *
 *        Version:  1.0
 *        Created:  06/20/2016 06:01:09 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ren Mao (neroam), neroam@umd.edu
 *   Organization:  
 *
 * =====================================================================================
 */

#include <world_model/world_model.h>
#include <world_model_msgs/Object.h>
#include <world_model_msgs/Location.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// Service call
#include <world_model_msgs/UpdateStatesDummy.h>
#include <world_model_msgs/GetStatesDummy.h>
// #include <world_model_msgs/PointCloud2Service.h>
#include <world_model_msgs/GetStatesObjects.h>
#include <world_model_msgs/UpdateStatesObjects.h>
#include <world_model_msgs/QueryLocations.h>
#include <world_model_msgs/ReconstructionUpdate.h>
#include <world_model_msgs/ReconstructionQuery.h>
#include <world_model_msgs/ReconstructedPointCloudQuery.h>

// Libraries
// #include <g1_control/utilities.h>

// RVIZ visual tools
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <eigen_conversions/eigen_msg.h>
#include <XmlRpcException.h>

namespace g1
{

namespace common
{

class WorldModel::WorldModelImpl
{
public:

  WorldModelImpl(ros::NodeHandle &node_handle)
    : dummy_state_("Empty"),
    visual_tools_ (new rviz_visual_tools::RvizVisualTools("/base","/world_model_visualizer")),
    visual_tools_location_ (new rviz_visual_tools::RvizVisualTools("/base","/world_model_locations_visualizer"))
  {
    // Dummy 
    dummy_update_service_ = node_handle.advertiseService("/world_model/update_states_dummy", &WorldModelImpl::updateDummyStates, this);

    dummy_get_service_ = node_handle.advertiseService("/world_model/get_states_dummy", &WorldModelImpl::getDummyStates, this);

    // Service call to update world model
    reconstruction_update_service_ = node_handle.advertiseService("/world_model/update_reconstruction", &WorldModelImpl::updateReconstruction, this);

    reconstruction_query_service_  = node_handle.advertiseService("/world_model/get_reconstruction", &WorldModelImpl::getReconstruction, this);

    reconstructed_point_cloud_query_service_ = node_handle.advertiseService("/world_model/get_reconstructed_point_cloud", &WorldModelImpl::getReconstructedPointCloud, this);

    objects_update_service_ = node_handle.advertiseService("/world_model/update_states_objects", &WorldModelImpl::updateObjectsStates, this);

    // Service call to read world model
    objects_get_service_ = node_handle.advertiseService("/world_model/get_states_objects", &WorldModelImpl::getObjectsStates, this);

    // Service call to query locations
    locations_service_ = node_handle.advertiseService("world_model/query_locations", &WorldModelImpl::queryLocations, this);

    // dummy publisher
    publisher_ = node_handle.advertise<std_msgs::String>("/world_model/dummy_states", 1);

    // Scene cloud publisher
    // scene_cloud_publiher_ = node_handle.advertise<sensor_msgs::PointCloud2>("/world_model/scene_cloud", 1);

    node_handle_ = node_handle;

    visual_tools_->setLifetime(2);
    visual_tools_location_->setLifetime(2);

    ROS_INFO("World model initialized.");
  }

  ~WorldModelImpl()
  {
    saveScene();
  }

  // Public method for updating internal states. 
  // For example: smoothing, fusion, listening, transforming.
  void updateStates()
  {
//     updateArMarker("ar_marker_1");
  }

//   // Update the object states from tf if necessary.
//   void updateArMarker(const std::string &object_id)
//   {
//     //Try to update AR Marker
//     geometry_msgs::Pose object_pose;
//     if (g1::control::getPosefromTF(object_pose, tf_listener_,
//             "base", object_id)) {
//       world_model_msgs::Object object;
//       object.id = object_id;
// 
//       shape_msgs::SolidPrimitive primitive;
//       primitive.type = primitive.CYLINDER;
//       primitive.dimensions.resize(2);
//       primitive.dimensions[0] = 0.07;
//       primitive.dimensions[1] = 0.09;
// 
//       object.primitives.push_back(primitive);
//       object.primitive_poses.push_back(object_pose);
// 
//       manipulation_objects_[object_id] = object;
//     }
//   }

  // Query locations
  bool queryLocations(
      world_model_msgs::QueryLocations::Request  &req,
      world_model_msgs::QueryLocations::Response &res)
  {
    // Check operation
    if (req.operation == req.DELETE) {
      for (int i = 0; i < req.locations.size(); i++) {
        // Remove all locations with id starts with given string
        std::string id = req.locations[i].id;
        std::map<std::string, world_model_msgs::Location>::iterator it;
        for (it = interested_locations_.begin(); it != interested_locations_.end();) {
          if (it->first.find(id) == 0) {
            ROS_INFO_STREAM("Erase location : " << it->first);
            interested_locations_.erase(it++);
          }
          else {
            ++it;
          }
        }
      }  
    }
    else if (req.operation == req.UPDATE) {
      for (int i = 0; i < req.locations.size(); i++) {
        ROS_INFO_STREAM("Update Location: " << req.locations[i].id);
        interested_locations_[req.locations[i].id] = req.locations[i];
      }
    }
    else if (req.operation == req.READ) {
      if (req.location_id != "") {
        if (interested_locations_.count(req.location_id) > 0) {
            res.locations.push_back(interested_locations_[req.location_id]);
        }
      }
      else {
        //return all locations
        for (std::map<std::string, world_model_msgs::Location>::iterator it = interested_locations_.begin(); it != interested_locations_.end(); ++it ) {
          res.locations.push_back(it->second);
        }
      }
    }

    res.success = true;
    return true;
  }

  // Update the object states from tf if necessary.
  bool updateObjectsStates(
      world_model_msgs::UpdateStatesObjects::Request  &req,
      world_model_msgs::UpdateStatesObjects::Response &res)
  {
    // Check if it is remove operation
    if (req.operation == req.DELETE) {
      for (int i = 0; i < req.objects_info.size(); i++) {
        // Remove all objects with id starts with given string
        std::string id = req.objects_info[i].id;
        std::map<std::string, world_model_msgs::Object>::iterator it;
        for (it = manipulation_objects_.begin(); it != manipulation_objects_.end();) {
          if (it->first.find(id) == 0) {
            ROS_INFO_STREAM("Erase object : " << it->first);
            manipulation_objects_.erase(it++);
          }
          else {
            ++it;
          }
        }

        for (it = scene_objects_.begin(); it != scene_objects_.end();) {
          if (it->first.find(id) == 0) {
            ROS_INFO_STREAM("Erase object : " << it->first);
            scene_objects_.erase(it++);
          }
          else {
            ++it;
          }
        }
      }

      if (req.objects_info.size() < 1) {
        // Remove all objects from manipulation objects.
        ROS_INFO_STREAM("Erase all manipulation objects in the world model.");
        manipulation_objects_.clear();
      }  
    }
    else if (req.operation == req.UPDATE) {
      // Default operation is to update objects states
      for (int i = 0; i < req.objects_info.size(); i++) {
        if (req.objects_info[i].primitive_poses.size() > 0 && req.objects_info[i].primitives.size() > 0 ) {
          ROS_INFO_STREAM("Update Object: " << req.objects_info[i].id);
          if  ( req.objects_info[i].id.find("microwave") == 0   ||
                req.objects_info[i].id.find("table")     == 0   ||
                req.objects_info[i].id.find("fridge")    == 0
              ) {
            scene_objects_[req.objects_info[i].id] = req.objects_info[i];
          } 
          else {
            manipulation_objects_[req.objects_info[i].id] = req.objects_info[i];
          }

        }
      }
    }
    res.success = true;
    return true;
  }

  void publishObjectsTF()
  {
    for (std::map<std::string, world_model_msgs::Object>::iterator it = manipulation_objects_.begin(); 
          it != manipulation_objects_.end(); ++it) {
      
      geometry_msgs::Pose transformed_pose = it->second.primitive_poses[0];
      
      tf::Transform transform;
      transform.setOrigin(
        tf::Vector3(transformed_pose.position.x,
                    transformed_pose.position.y,
                    transformed_pose.position.z));
      tf::Quaternion orientation;
      tf::quaternionMsgToTF(transformed_pose.orientation, orientation);
      transform.setRotation(orientation);
      tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "world_model/" + it->second.id));
    }
  }

  void visualizeObjectsMap(const std::map<std::string, world_model_msgs::Object> & mp, const rviz_visual_tools::colors &color) {
    for (std::map<std::string, world_model_msgs::Object>::const_iterator it = mp.begin();
          it != mp.end(); ++it) {
//       ROS_INFO_STREAM(it->second);
      int idx = 0;
      for (int i = 0; i < it->second.primitives.size(); i++) {
        //Delete the marker
        // visualization_msgs::Marker reset_marker;
        // reset_marker.header.stamp = ros::Time::now();
        // reset_marker.header.frame_id = "base";
        // reset_marker.ns = it->second.id;
        // reset_marker.id = ++idx;
        // reset_marker.action = reset_marker.DELETE;
        // visual_tools_->publishMarker(reset_marker);
        ++idx;
        if (it->second.primitives[i].type == it->second.primitives[i].BOX) {
            visualization_msgs::Marker box_marker;
            box_marker.header.stamp = ros::Time::now();
            box_marker.header.frame_id = "base";
            box_marker.ns = it->second.id;
            box_marker.id = idx;
            box_marker.type = box_marker.CUBE;
            box_marker.pose = it->second.primitive_poses[i];
            box_marker.scale.x = it->second.primitives[i].dimensions[0]; 
            box_marker.scale.y = it->second.primitives[i].dimensions[1];
            box_marker.scale.z = it->second.primitives[i].dimensions[2];
            box_marker.color = visual_tools_->getColor(color);
            box_marker.lifetime = ros::Duration(2);
            
            box_marker.action = box_marker.ADD;
            visual_tools_->publishMarker(box_marker);
          
        } else if (it->second.primitives[i].type == it->second.primitives[i].CYLINDER) {
          
          // Extract object parameters
          visualization_msgs::Marker marker;
          marker.header.stamp = ros::Time::now();
          marker.header.frame_id = "base";
          marker.ns = it->second.id;
          marker.id = idx;
          marker.type = marker.CYLINDER;
          marker.pose = it->second.primitive_poses[i];
          marker.scale.x = it->second.primitives[i].dimensions[1] * 2; 
          marker.scale.y = it->second.primitives[i].dimensions[1] * 2;
          marker.scale.z = it->second.primitives[i].dimensions[0];
          marker.color = visual_tools_->getColor(color);
          marker.action = marker.ADD;
          marker.lifetime = ros::Duration(2);

          // Publish
          visual_tools_->publishMarker(marker);

        } else {
          std::cout << "Error: unknown shape should not appear!" << std::endl;
        //   visual_tools_->publishCone(
        //     it->second.primitive_poses[i],
        //     rviz_visual_tools::GREEN,
        //     it->second.primitives[i].dimensions[0],
        //     it->first
        //   );
        } 
      } 
    } 
  }

  void visualizeLocationsMap(const std::map<std::string, world_model_msgs::Location> & mp) {
    for (std::map<std::string, world_model_msgs::Location>::const_iterator it = mp.begin(); it != mp.end(); ++it) {
      // visualization_msgs::Marker reset_marker;
      // reset_marker.header.stamp = ros::Time::now();
      // reset_marker.header.frame_id = "base";
      // reset_marker.ns = it->second.id;
      // reset_marker.action = reset_marker.DELETE;
      // visual_tools_location_->publishMarker(reset_marker);

      // Extract object parameters
      Eigen::Affine3d pose;
      geometry_msgs::Pose location_pose;
      location_pose.position = it->second.position;
      location_pose.orientation.w = 1.0;
      tf::poseMsgToEigen(location_pose, pose);
      double height = 0.01;
      double radius = 0.2;

      // Publish
      visual_tools_location_->publishCylinder  ( pose, rviz_visual_tools::WHITE, height, radius, it->second.id);
    } 
  }

  void visualizeObjects() {
    // visual_tools_->deleteAllMarkers();
    // visual_tools_location_->deleteAllMarkers();
    visualizeObjectsMap(manipulation_objects_, rviz_visual_tools::GREEN);
    visualizeObjectsMap(scene_objects_, rviz_visual_tools::BLUE);
    visualizeLocationsMap(interested_locations_);
  }


  // Public method for initializing scene
  void initializeScene()
  {
    // Load scene objects
    XmlRpc::XmlRpcValue scene_list;
    if (!ros::param::get("/world_model/initial_scene/scene", scene_list)) {
      ROS_ERROR("Cannot load initial scene");
    }

    world_model_msgs::Object obj;
    ROS_ASSERT(scene_list.getType() == XmlRpc::XmlRpcValue::TypeArray); 
    for (int32_t i = 0; i < scene_list.size(); ++i) {
      try {
        obj.primitives.clear();
        obj.primitive_poses.clear();
        ROS_ASSERT(scene_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct); 
        obj.id = static_cast<std::string>(scene_list[i]["id"]);

        XmlRpc::XmlRpcValue primitives = scene_list[i]["primitives"];
        XmlRpc::XmlRpcValue primitive_poses = scene_list[i]["primitive_poses"];

        ROS_ASSERT(primitives.getType() == XmlRpc::XmlRpcValue::TypeArray); 
        ROS_ASSERT(primitive_poses.getType() == XmlRpc::XmlRpcValue::TypeArray); 
        ROS_ASSERT(primitive_poses.size() == primitives.size()); 

        for (int32_t j = 0; j < primitives.size(); ++j) {
          shape_msgs::SolidPrimitive primitive;
          primitive.type = static_cast<int>(primitives[j]["shape"]);
          for (int32_t k = 0; k < primitives[j]["dimensions"].size(); ++k) {
            primitive.dimensions.push_back(static_cast<double>(primitives[j]["dimensions"][k]));
          }

          ROS_ASSERT(primitive_poses[j].size() == 7); 
          geometry_msgs::Pose primitive_pose;
          primitive_pose.position.x = static_cast<double>(primitive_poses[j][0]);
          primitive_pose.position.y = static_cast<double>(primitive_poses[j][1]);
          primitive_pose.position.z = static_cast<double>(primitive_poses[j][2]);
          primitive_pose.orientation.x = static_cast<double>(primitive_poses[j][3]);
          primitive_pose.orientation.y = static_cast<double>(primitive_poses[j][4]);
          primitive_pose.orientation.z = static_cast<double>(primitive_poses[j][5]);
          primitive_pose.orientation.w = static_cast<double>(primitive_poses[j][6]);

          obj.primitives.push_back(primitive);
          obj.primitive_poses.push_back(primitive_pose);
        }
        scene_objects_[obj.id] = obj;  
        ROS_INFO_STREAM("Load object in the initial scene: " << obj.id);
      }
      catch (XmlRpc::XmlRpcException const e) {
        ROS_ERROR_STREAM("Load scene failed because " << e.getMessage());
      }
    }

    // Load manipulation objects
    if (!ros::param::get("/world_model/initial_scene/objects", scene_list)) {
      ROS_ERROR("Cannot load initial scene");
    }

    ROS_ASSERT(scene_list.getType() == XmlRpc::XmlRpcValue::TypeArray); 
    for (int32_t i = 0; i < scene_list.size(); ++i) {
      try {
        obj.primitives.clear();
        obj.primitive_poses.clear();
        ROS_ASSERT(scene_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct); 
        obj.id = static_cast<std::string>(scene_list[i]["id"]);

        XmlRpc::XmlRpcValue primitives = scene_list[i]["primitives"];
        XmlRpc::XmlRpcValue primitive_poses = scene_list[i]["primitive_poses"];

        ROS_ASSERT(primitives.getType() == XmlRpc::XmlRpcValue::TypeArray); 
        ROS_ASSERT(primitive_poses.getType() == XmlRpc::XmlRpcValue::TypeArray); 
        ROS_ASSERT(primitive_poses.size() == primitives.size()); 

        for (int32_t j = 0; j < primitives.size(); ++j) {
          shape_msgs::SolidPrimitive primitive;
          primitive.type = static_cast<int>(primitives[j]["shape"]);
          for (int32_t k = 0; k < primitives[j]["dimensions"].size(); ++k) {
            primitive.dimensions.push_back(static_cast<double>(primitives[j]["dimensions"][k]));
          }

          ROS_ASSERT(primitive_poses[j].size() == 7); 
          geometry_msgs::Pose primitive_pose;
          primitive_pose.position.x = static_cast<double>(primitive_poses[j][0]);
          primitive_pose.position.y = static_cast<double>(primitive_poses[j][1]);
          primitive_pose.position.z = static_cast<double>(primitive_poses[j][2]);
          primitive_pose.orientation.x = static_cast<double>(primitive_poses[j][3]);
          primitive_pose.orientation.y = static_cast<double>(primitive_poses[j][4]);
          primitive_pose.orientation.z = static_cast<double>(primitive_poses[j][5]);
          primitive_pose.orientation.w = static_cast<double>(primitive_poses[j][6]);

          obj.primitives.push_back(primitive);
          obj.primitive_poses.push_back(primitive_pose);
        }
        manipulation_objects_[obj.id] = obj;  
        ROS_INFO_STREAM("Load object in the initial scene: " << obj.id);
      }
      catch (XmlRpc::XmlRpcException const e) {
        ROS_ERROR_STREAM("Load scene failed because " << e.getMessage());
      }
    }

    // Load interested locations
    if (!ros::param::get("/world_model/initial_scene/locations", scene_list)) {
      ROS_ERROR("Cannot load initial scene");
    }

    world_model_msgs::Location location;
    ROS_ASSERT(scene_list.getType() == XmlRpc::XmlRpcValue::TypeArray); 
    for (int32_t i = 0; i < scene_list.size(); ++i) {
      try {
        ROS_ASSERT(scene_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct); 
        location.id = static_cast<std::string>(scene_list[i]["id"]);

        XmlRpc::XmlRpcValue position = scene_list[i]["position"];
        ROS_ASSERT(position.getType() == XmlRpc::XmlRpcValue::TypeArray); 
        ROS_ASSERT(position.size() == 3); 

        location.position.x = position[0];
        location.position.y = position[1];
        location.position.z = position[2];

        interested_locations_[location.id] = location;
        ROS_INFO_STREAM("Load location in the initial scene: " << location.id);
      }
      catch (XmlRpc::XmlRpcException const e) {
        ROS_ERROR_STREAM("Load scene failed because " << e.getMessage());
      }
    }

//     // Add table into the scene
//     world_model_msgs::Object table;
//     table.id = std::string("table");
// 
//     shape_msgs::SolidPrimitive primitive;
//     primitive.type = primitive.BOX;
//     primitive.dimensions.resize(3);
//     primitive.dimensions[0] = 0.8;
//     primitive.dimensions[1] = 1.6;
//     primitive.dimensions[2] = 1.0;
// 
//     geometry_msgs::Pose primitive_pose;
//     primitive_pose.orientation.w = 1.0;
//     primitive_pose.position.x = 0.72;
//     primitive_pose.position.y = 0;
//     primitive_pose.position.z = -0.69;
// 
//     table.primitives.push_back(primitive);
//     table.primitive_poses.push_back(primitive_pose);
//     scene_objects_[table.id] = table; 
  }

  XmlRpc::XmlRpcValue arrayToXmlValue(const XmlRpc::XmlRpcValue::ValueArray &array)
  {
    std::string xml = "<value><array><data>";
    for (int i = 0; i < array.size(); i++) {
      xml += array[i].toXml();
    }
    xml += "</data></array></value>";
    int offset = 0;
    return XmlRpc::XmlRpcValue(xml, &offset);
  }

  XmlRpc::XmlRpcValue structToXmlValue(XmlRpc::XmlRpcValue::ValueStruct &list)
  {
    std::string xml = "<value><struct>";
    for (std::map<std::string, XmlRpc::XmlRpcValue>::iterator it = list.begin(); it != list.end(); ++it) {
      xml += "<member><name>" + it->first + "</name>";
      xml += it->second.toXml();
      xml += "</member>";
    }
    xml += "</struct></value>";
    int offset = 0;
    return XmlRpc::XmlRpcValue(xml, &offset);
  }

  // Save scene to yaml config file
  void saveScene()
  {
    std::string file_path;
    if (!ros::param::get("/world_model/file_path", file_path)) {
      ROS_ERROR("Cannot save file path for initial scene");
      return;
    }

    // Set scene objects
    XmlRpc::XmlRpcValue::ValueArray scene_list;
    for (std::map<std::string, world_model_msgs::Object>::iterator it = scene_objects_.begin(); it != scene_objects_.end(); ++it ) {
      XmlRpc::XmlRpcValue::ValueStruct obj;
      obj["id"] = XmlRpc::XmlRpcValue(it->second.id);
      XmlRpc::XmlRpcValue::ValueArray primitives, primitive_poses;

      for (int i = 0; i < it->second.primitives.size(); i++) {
        // Primitives
        XmlRpc::XmlRpcValue::ValueStruct primitive;
        primitive["shape"] = XmlRpc::XmlRpcValue(static_cast<int>(it->second.primitives[i].type));
        XmlRpc::XmlRpcValue::ValueArray dimensions;
        for (int k = 0; k < it->second.primitives[i].dimensions.size(); k++) {
          dimensions.push_back(XmlRpc::XmlRpcValue(it->second.primitives[i].dimensions[k]));
        }
        primitive["dimensions"] = arrayToXmlValue(dimensions);
        primitives.push_back(structToXmlValue(primitive));

        // Primitive poses
        XmlRpc::XmlRpcValue::ValueArray primitive_pose;
        primitive_pose.resize(7);
        primitive_pose[0] = XmlRpc::XmlRpcValue(it->second.primitive_poses[i].position.x);
        primitive_pose[1] = XmlRpc::XmlRpcValue(it->second.primitive_poses[i].position.y);
        primitive_pose[2] = XmlRpc::XmlRpcValue(it->second.primitive_poses[i].position.z);
        primitive_pose[3] = XmlRpc::XmlRpcValue(it->second.primitive_poses[i].orientation.x);
        primitive_pose[4] = XmlRpc::XmlRpcValue(it->second.primitive_poses[i].orientation.y);
        primitive_pose[5] = XmlRpc::XmlRpcValue(it->second.primitive_poses[i].orientation.z);
        primitive_pose[6] = XmlRpc::XmlRpcValue(it->second.primitive_poses[i].orientation.w);
        primitive_poses.push_back(arrayToXmlValue(primitive_pose));
      }
      obj["primitives"] = arrayToXmlValue(primitives);
      obj["primitive_poses"] = arrayToXmlValue(primitive_poses);
      scene_list.push_back(structToXmlValue(obj));
    }

//     // Set manipulation objects
    XmlRpc::XmlRpcValue::ValueArray object_list;
    for (std::map<std::string, world_model_msgs::Object>::iterator it = manipulation_objects_.begin(); it != manipulation_objects_.end(); ++it ) {
      XmlRpc::XmlRpcValue::ValueStruct obj;
      obj["id"] = XmlRpc::XmlRpcValue(it->second.id);
      XmlRpc::XmlRpcValue::ValueArray primitives, primitive_poses;

      for (int i = 0; i < it->second.primitives.size(); i++) {
        // Primitives
        XmlRpc::XmlRpcValue::ValueStruct primitive;
        primitive["shape"] = XmlRpc::XmlRpcValue(static_cast<int>(it->second.primitives[i].type));
        XmlRpc::XmlRpcValue::ValueArray dimensions;
        for (int k = 0; k < it->second.primitives[i].dimensions.size(); k++) {
          dimensions.push_back(XmlRpc::XmlRpcValue(it->second.primitives[i].dimensions[k]));
        }
        primitive["dimensions"] = arrayToXmlValue(dimensions);
        primitives.push_back(structToXmlValue(primitive));

        // Primitive poses
        XmlRpc::XmlRpcValue::ValueArray primitive_pose;
        primitive_pose.resize(7);
        primitive_pose[0] = XmlRpc::XmlRpcValue(it->second.primitive_poses[i].position.x);
        primitive_pose[1] = XmlRpc::XmlRpcValue(it->second.primitive_poses[i].position.y);
        primitive_pose[2] = XmlRpc::XmlRpcValue(it->second.primitive_poses[i].position.z);
        primitive_pose[3] = XmlRpc::XmlRpcValue(it->second.primitive_poses[i].orientation.x);
        primitive_pose[4] = XmlRpc::XmlRpcValue(it->second.primitive_poses[i].orientation.y);
        primitive_pose[5] = XmlRpc::XmlRpcValue(it->second.primitive_poses[i].orientation.z);
        primitive_pose[6] = XmlRpc::XmlRpcValue(it->second.primitive_poses[i].orientation.w);
        primitive_poses.push_back(arrayToXmlValue(primitive_pose));
      }
      obj["primitives"] = arrayToXmlValue(primitives);
      obj["primitive_poses"] = arrayToXmlValue(primitive_poses);
      object_list.push_back(structToXmlValue(obj));
    }

    // Set locations
    XmlRpc::XmlRpcValue::ValueArray location_list;
    for (std::map<std::string, world_model_msgs::Location>::iterator it = interested_locations_.begin(); it != interested_locations_.end(); ++it ) {
      XmlRpc::XmlRpcValue::ValueStruct loc;
      loc["id"] = XmlRpc::XmlRpcValue(it->second.id);
      XmlRpc::XmlRpcValue::ValueArray position;
      position.resize(3);
      position[0] = XmlRpc::XmlRpcValue(it->second.position.x);
      position[1] = XmlRpc::XmlRpcValue(it->second.position.y);
      position[2] = XmlRpc::XmlRpcValue(it->second.position.z);
      loc["position"] = arrayToXmlValue(position);
      location_list.push_back(structToXmlValue(loc));
    }

    // Save to yaml
    ros::param::set("/world_model/initial_scene/scene", arrayToXmlValue(scene_list));
    ros::param::set("/world_model/initial_scene/objects", arrayToXmlValue(object_list));
    ros::param::set("/world_model/initial_scene/locations", arrayToXmlValue(location_list));
    std::string cmd = "rosparam dump " + file_path + " /world_model/initial_scene";
    system(cmd.c_str());
  }


  // Public method for publishing all messages.
  void publishStates()
  {
    publishDummyStates();
    publishReconstructedClouds();
    publishObjectsTF();
    visualizeObjects();    
    //ROS_INFO("Publish done.");
  }

  // Handle publishing dummy states
  void publishDummyStates()
  {
    std_msgs::String msg;
    msg.data = dummy_state_ + " from world model";
    publisher_.publish(msg);
  }

  // Publish reconstructed scene pointcloud to rviz
  void publishReconstructedClouds()
  {
    for (std::map<std::string, ros::Publisher>::iterator it = reconstructed_cloud_publishers_.begin(); it != reconstructed_cloud_publishers_.end(); ++it) {
      it->second.publish(reconstructions_[it->first].model_point_cloud);
    }
  }

  // Update the scene reconstruction
  bool updateReconstruction ( world_model_msgs::ReconstructionUpdate::Request  &req,
                              world_model_msgs::ReconstructionUpdate::Response &res)
  {
    if (req.operation == req.DELETE) {
      if (req.id.empty()) {
        reconstructions_.clear();
        reconstructed_cloud_publishers_.clear();
        res.success = true;
        ROS_INFO("Deleted all reconstruction objects from the world model.");
      } else {
        reconstructed_cloud_publishers_.erase(req.id);
        res.success = reconstructions_.erase(req.id) > 0;
        ROS_INFO("Deleted \"%s\" reconstruction object from the world model.", req.id.c_str());
      }
    } else {
      // Default to req.UPDATE
      reconstructions_[req.id] = req.reconstruction;
      res.success = true;
      reconstructed_cloud_publishers_[req.id] = node_handle_.advertise<sensor_msgs::PointCloud2>("/world_model/" + req.id, 1);
      ROS_INFO("Reconstruction object for \"%s\" received by the world model.", req.id.c_str());
    }
    return true;
  }

  // Send scene reconstruction results to client
  bool getReconstruction  ( world_model_msgs::ReconstructionQuery::Request  &req,
                            world_model_msgs::ReconstructionQuery::Response &res)
  {
    if (reconstructions_.find(req.id) == reconstructions_.end()) {
      res.success = false;
      ROS_INFO("Reconstruction object for \"%s\" not found in the world model.", req.id.c_str());
    } else {
      res.reconstruction = reconstructions_[req.id];
      res.success = true;
      ROS_INFO("Query for \"%s\" reconstruction object succeeded.", req.id.c_str());
    }
    return true;
  }

  bool getReconstructedPointCloud ( world_model_msgs::ReconstructedPointCloudQuery::Request &req,
                                    world_model_msgs::ReconstructedPointCloudQuery::Response &res)
  {
    if (reconstructions_.find(req.id) == reconstructions_.end()) {
      res.success = false;
      ROS_INFO("Reconstructed cloud for \"%s\" not found in the world model.", req.id.c_str());
    } else {
      res.point_cloud = reconstructions_[req.id].model_point_cloud;
      res.success = true;
      ROS_INFO("Query for \"%s\" reconstructed cloud succeeded.", req.id.c_str());
    }
    return true;
  }

  // Handle dummy states updates
  bool updateDummyStates(
      world_model_msgs::UpdateStatesDummy::Request  &req,
      world_model_msgs::UpdateStatesDummy::Response &res)
  {
    dummy_state_ = req.dummy_msg;
    res.success = true;
    ROS_INFO_STREAM("Updated world model states by dummy service: " << dummy_state_);
    return true;
  }

  // Handle dummy states access request
  bool getDummyStates(
      world_model_msgs::GetStatesDummy::Request &req,
      world_model_msgs::GetStatesDummy::Response &res)
  {
    res.dummy_msg = dummy_state_ + " from world model";
    ROS_INFO("Got world model states by dummy service.");
    return true;
  }

  // Handle objects states access request
  bool getObjectsStates(
      world_model_msgs::GetStatesObjects::Request &req,
      world_model_msgs::GetStatesObjects::Response &res
      )
  {
    if (req.object_id != "") {
      if (req.manipulatable) {
        if (manipulation_objects_.count(req.object_id) > 0) {
          res.objects.push_back(manipulation_objects_[req.object_id]);
        }
      }
      else {
        if (scene_objects_.count(req.object_id) > 0) {
          res.objects.push_back(scene_objects_[req.object_id]);
        }
      }
    }
    else {
      //return all objects
      if (req.manipulatable) {
        for (std::map<std::string, world_model_msgs::Object>::iterator it = manipulation_objects_.begin(); it != manipulation_objects_.end(); ++it ) {
          res.objects.push_back(it->second);
        }
      }
      else {
        for (std::map<std::string, world_model_msgs::Object>::iterator it = scene_objects_.begin(); it != scene_objects_.end(); ++it ) {
          res.objects.push_back(it->second);
        }
      }
    }
    return true;
  }

private:
  ros::NodeHandle node_handle_;

  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;

  // publishers
  ros::Publisher publisher_;

  // Publishers for reconstructed clouds
  std::map<std::string, ros::Publisher> reconstructed_cloud_publishers_;

  // For visualizing things in rviz
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_, visual_tools_location_;  
 
  // service for updating world model
  ros::ServiceServer dummy_update_service_;
  ros::ServiceServer reconstruction_update_service_;
  ros::ServiceServer reconstruction_query_service_;
  ros::ServiceServer reconstructed_point_cloud_query_service_;
  ros::ServiceServer objects_update_service_;
  ros::ServiceServer locations_service_;

  // service for reading world model
  ros::ServiceServer dummy_get_service_;
  ros::ServiceServer objects_get_service_;

  // world model stored states
  std::string dummy_state_;
  std::map<std::string, reconstruction_msgs::Reconstruction> reconstructions_;
  std::map<std::string, world_model_msgs::Object> scene_objects_;
  std::map<std::string, world_model_msgs::Object> manipulation_objects_;
  std::map<std::string, world_model_msgs::Location> interested_locations_;
};


WorldModel::WorldModel(ros::NodeHandle &node_handle)
{
  impl_ = new WorldModelImpl(node_handle);
}

WorldModel::~WorldModel()
{
  delete impl_;
}

void WorldModel::updateStates()
{
  impl_->updateStates();
}

void WorldModel::publishStates()
{
  impl_->publishStates();
}

void WorldModel::initializeScene()
{
  impl_->initializeScene();
}

}
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "g1_world_model", ros::init_options::AnonymousName);

  ros::NodeHandle node_handle;
  ros::Rate loop_rate(10);

  g1::common::WorldModel world_model(node_handle);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  world_model.initializeScene();

  while (ros::ok()) {
    world_model.updateStates();
    world_model.publishStates();
    loop_rate.sleep();
  }

  return 0;
};

