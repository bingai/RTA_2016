#include <ros/ros.h>


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
#include <world_model_msgs/UpdateStatesObjects.h>
#include <world_model_msgs/ImageCoordinate.h>
#include <world_model_msgs/QueryLocations.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/CameraInfo.h>


class UpdateObjectPosition {

private:
  ros::ServiceServer object_get_service_;
  ros::Subscriber sub_;
  ros::ServiceClient location_client_;

  bool capture;
  std::vector<sensor_msgs::PointCloud2ConstPtr> pcdQue;
  tf::TransformListener *tf_listener;
  Eigen::Vector3f object_pos;
  bool pos_update;
  bool clicknew;
  int row;
  int col;
  std::string location_id;

  void callback(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    if (!capture) {
      return;
    }
    if (cloud->width * cloud->height == 0) {
      return;
    }

    if (pcdQue.size() == 0) {
      pcdQue.push_back(cloud);
    } else {
      pcdQue[0] = cloud;
    } 
  }



  bool readTFTransform(const std::string& target_frame, const std::string& source_frame,
    const ros::Time &time, const double & timeout, Eigen::Affine3d & T_Eigen) {
    tf_listener->waitForTransform(target_frame, source_frame, time, ros::Duration(timeout) );
    tf::StampedTransform transform;
    std::cout << "start tf" << std::endl;

    try {
      tf_listener->lookupTransform (target_frame, source_frame, time, transform);
    } catch (tf::LookupException &e) {
      ROS_ERROR ("%s", e.what ());
      return false;
    } catch (tf::ExtrapolationException &e) {
      ROS_ERROR ("%s", e.what ());
      return false;
    }
    std::cout << "Finish read tf" << std::endl;

    tf::transformTFToEigen (transform, T_Eigen );
     std::cout << T_Eigen.matrix() << std::endl;
    return true;
  }

  bool updateWMLocations (std::string id, Eigen::Vector3f pos) {
    
    world_model_msgs::QueryLocations srv;
    srv.request.operation = srv.request.UPDATE;
    world_model_msgs::Location loc;
    loc.id = id;
    loc.position.x = pos[0];
    loc.position.y = pos[1];
    loc.position.z = pos[2];

    srv.request.locations.push_back(loc);

    if (location_client_.call(srv))
    {
      if (srv.response.success) {
        ROS_INFO_STREAM("Update the location");
      } else {
        ROS_INFO_STREAM("world_model location service error");
        return false;
      }

    }
    else {
      ROS_INFO_STREAM("\n\nWorld model server down\n\n");
      return false;
    }
    return true;
  }

public:
  UpdateObjectPosition(ros::NodeHandle &node_handle):pos_update(false), capture(false),
    clicknew(false){
    object_get_service_ = node_handle.advertiseService("/interact/update_3d_position", &UpdateObjectPosition::getObjectsStates, this);
    sub_ = node_handle.subscribe("/camera/depth_registered/points", 1, &UpdateObjectPosition::callback, this);

    tf::TransformListener listener(ros::Duration(10));
    row = -1;
    col = -1;

    location_client_ = node_handle.serviceClient<world_model_msgs::QueryLocations>("world_model/query_locations");
    if (!location_client_.waitForExistence(ros::Duration(2))) {
      ROS_ERROR("Cannot connect to the location service");
    }

    tf_listener = new tf::TransformListener();

    ROS_INFO("Update object states initialized.");
  }


  void publishObjectTF() {
    if (!pos_update) return;
    static tf::TransformBroadcaster br;
    tf::Transform microTF;
    microTF.setOrigin(tf::Vector3(object_pos[0], object_pos[1], object_pos[2]) );
    tf::Quaternion microQ;
    microQ.setRPY(0, 0, 0);
    microTF.setRotation(microQ);
    // br.sendTransform(tf::StampedTransform(microTF, ros::Time::now(), "/camera_rgb_optical_frame", "/object"));
    br.sendTransform(tf::StampedTransform(microTF, ros::Time::now(), "/base", "/object"));

  }

  bool getObjectsStates(world_model_msgs::ImageCoordinate::Request & request,
    world_model_msgs::ImageCoordinate::Response & response) {
    
    std::cout << "Two ImageCoordinate service called" << std::endl;
    capture = true;
    bool finished = false;
    row = request.row;
    col = request.col;
    location_id = request.location_id;
    response.success = true;
    clicknew = true;
    std::cout << row << "  " << col << std::endl;

    return true;
  }

  void run() {
    capture = true;
    if (row > 0 && col > 0 && clicknew) {
      if (pcdQue.size() == 0) {
        ROS_ERROR("no point cloud in the queue. Wait for a momement");
        ros::Duration(1.0).sleep();
        return;
      }
      sensor_msgs::PointCloud2ConstPtr cloud = pcdQue[0];
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(*cloud, *cloud_in);
    
      // std::cout << cloud->header.frame_id << std::endl;
      Eigen::Affine3d T_Eigen;
      // pcl::transformPointCloud(*cloud_in, *cloud_tf, T_Eigen.cast<float>());
      
      if (!pcl::isFinite(cloud_in->points[row * cloud_in->width + col])) {
        ROS_ERROR("NAN Wait for a momement");
        ros::Duration(1.0).sleep();
        return;
      }
      object_pos = cloud_in->points[row * cloud_in->width + col].getVector3fMap();

      if (!readTFTransform("/base", cloud->header.frame_id, cloud->header.stamp, 3.0, T_Eigen) ) {
        std::cout << "tf issue" << std::endl;
        return;
      }
      float x = static_cast<float> (T_Eigen(0, 0) * object_pos(0) + T_Eigen(0, 1) * object_pos(1) + T_Eigen(0, 2) * object_pos(2) + T_Eigen(0, 3));
      float y = static_cast<float> (T_Eigen(1, 0) * object_pos(0) + T_Eigen(1, 1) * object_pos(1) + T_Eigen(1, 2) * object_pos(2) + T_Eigen(1, 3));
      float z = static_cast<float> (T_Eigen(2, 0) * object_pos(0) + T_Eigen(2, 1) * object_pos(1) + T_Eigen(2, 2) * object_pos(2) + T_Eigen(2, 3));
      object_pos = Eigen::Vector3f(x, y, z); 
      // write to the world model about the new loation
      updateWMLocations(location_id, object_pos);
      //
      clicknew = false;
      pos_update = true;
      // updateWorldStates(p3d);
    }

    
    capture = false;
  }


};

 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "update_position", ros::init_options::AnonymousName);

  ros::NodeHandle node_handle;
  ros::Rate loop_rate(10);

  UpdateObjectPosition upobjs(node_handle);

  ros::AsyncSpinner spinner(1);
  spinner.start();


  while (ros::ok()) {
    upobjs.run();
    // upobjs.publishObjectTF();
    loop_rate.sleep();
  }

  return 0;
};

