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
#include <world_model_msgs/GetStatesObjects.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <graph_filter/GraphFilter.h>


#include <hand_tracker_2d/HandBBox.h>

class ObjectChoice {

private:
  ros::ServiceServer object_get_service_;
  ros::Subscriber sub_;
  ros::ServiceClient fetch_client_;
  ros::ServiceClient update_client_;


  bool capture;
  std::vector<sensor_msgs::PointCloud2ConstPtr> pcdQue;
  tf::TransformListener *tf_listener;
  Eigen::Vector3f object_pos;
  bool pos_update;
  bool clicknew;
  int row;
  int col;
  std::string object_id;
  ros::Publisher bbox_pub_;

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
     // std::cout << T_Eigen.matrix() << std::endl;
    return true;
  }


 bool chooseWMObjects (std::string id, Eigen::Vector3f pos) {
    
    world_model_msgs::GetStatesObjects srv;
    srv.request.object_id = "";
    srv.request.manipulatable = true;
    if (!fetch_client_.call(srv))
    {
      ROS_INFO_STREAM("\n\nWorld model server down\n\n");
      return false;
    }


    std::vector<world_model_msgs::Object> objects = srv.response.objects;
    // Figure out the object user clicks;
    float minDist = 1000.0;
    int min_index = -1;
    for (int i = 0; i < objects.size(); i++) {
      geometry_msgs::Point p = objects[i].primitive_poses[0].position;
      Eigen::Vector3f centroid(p.x, p.y, p.z);
      float dist = (centroid - pos).norm();
      if (dist < minDist) {
        minDist = dist;
        min_index = i;
      }
    }

    world_model_msgs::Object obj = objects[min_index];

    world_model_msgs::UpdateStatesObjects del_srv;
    del_srv.request.objects_info.resize(1);
    del_srv.request.objects_info[0] = obj;

    del_srv.request.operation = del_srv.request.DELETE;
    if (update_client_.call(del_srv))
    {
      ROS_INFO_STREAM("Update World Model Success: ");
    }
    else {
      ROS_ERROR("Failed to update world model.");
      // return false;
    }
    std::cout << "object id " << object_id << std::endl; 

    world_model_msgs::UpdateStatesObjects up_srv;
    up_srv.request.objects_info.resize(1);
    obj.id = object_id;
    up_srv.request.objects_info[0] = obj;
    up_srv.request.operation = up_srv.request.UPDATE;
    if (update_client_.call(up_srv))
    {
      ROS_INFO_STREAM("Update World Model Success: ");
      return true;
    }
    else {
      ROS_ERROR("Failed to update world model.");
      return false;
    }
 

    return true;
  }
 
public:
  ObjectChoice(ros::NodeHandle &node_handle):pos_update(false), capture(false),
    clicknew(false){
    object_get_service_ = node_handle.advertiseService("/interact/object_choice", &ObjectChoice::getObjectsStates, this);
    sub_ = node_handle.subscribe("/camera/depth_registered/points", 1, &ObjectChoice::callback, this);
    // sub_ = node_handle.subscribe("/remote/camera/depth_registered/points", 1, &ObjectChoice::callback, this);

    tf::TransformListener listener(ros::Duration(10));
    row = -1;
    col = -1;

    fetch_client_ = node_handle.serviceClient<world_model_msgs::GetStatesObjects>("/world_model/get_states_objects");
    if (!fetch_client_.waitForExistence(ros::Duration(2))) {
      ROS_ERROR("Cannot connect to the location service");
    }


    update_client_ = node_handle.serviceClient<world_model_msgs::UpdateStatesObjects>("/world_model/update_states_objects");
    if (!fetch_client_.waitForExistence(ros::Duration(2))) {
      ROS_ERROR("Cannot connect to the location service");
    }

    bbox_pub_ = node_handle.advertise<hand_tracker_2d::HandBBox>("/interact/hand_location", 1);


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
    object_id = request.location_id;
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
      chooseWMObjects(object_id, object_pos);
      //
      clicknew = false;
      pos_update = true;
      // updateWorldStates(p3d);
    }

    
    capture = false;
  }


  bool point2objects(std::string id) {
    capture = true;
    GraphFilter gf;
    if (pcdQue.size() == 0) {
      ROS_ERROR("no point cloud in the queue. Wait for a momement");
      capture = true;
      ros::Duration(1.0).sleep();
      return false;
    }

    sensor_msgs::PointCloud2ConstPtr cloud = pcdQue[0];
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud, *cloud_in);
  
    Eigen::Affine3d T_Eigen;
    
    if (!readTFTransform("/base", cloud->header.frame_id, cloud->header.stamp, 3.0, T_Eigen) ) {
      std::cout << "tf issue" << std::endl;
      return false;
    }

    gf.initialize();
    gf.setInputCloudAndTransform(cloud_in, T_Eigen);
    gf.setTimeStamp(cloud->header.stamp);
    gf.generateGraph();
    gf.constraintsFiltering();
    std::vector<std::pair<Eigen::Vector3f, 
      Eigen::Vector3f> > rays = gf.getRays();     

    if (rays.size() == 0) {
      std::cout << "No finger detected at this frame " << std::endl;
      return false;
    }


    std::vector<hand_tracker_2d::HandBBox> boxes = gf.getBBoxes();
    for (int i = 0; i < boxes.size(); i++) {
      bbox_pub_.publish(boxes[i]);
    }


    world_model_msgs::GetStatesObjects srv;
    srv.request.object_id = "";
    srv.request.manipulatable = true;
    if (!fetch_client_.call(srv))
    {
      ROS_INFO_STREAM("\n\nWorld model server down\n\n");
      return false;
    }

    
    std::vector<world_model_msgs::Object> objects = srv.response.objects;
    if (objects.size() == 0) {
      std::cout << "No objects in world model " << std::endl;
      return false;
    }

    float minDist = 1000.0;
    int min_index = -1;
    for (int i = 0; i < objects.size(); i++) {
      geometry_msgs::Point p = objects[i].primitive_poses[0].position;
      Eigen::Vector3f centroid(p.x, p.y, p.z);
      float dist = (centroid - rays[0].first).dot(rays[0].second);
      float gap = (centroid - rays[0].first - dist * rays[0].second).norm();
      std::cout << "Debug " << gap << std::endl;

      if (gap < minDist) {
        minDist = gap;
        min_index = i;
      }
    }
    
    if (min_index == -1) {
      std::cout << "point index is  -1 !!!!!" << std::endl;
      min_index = 0;
    }

    static tf::TransformBroadcaster br;
    tf::Transform microTF;
    microTF.setOrigin(tf::Vector3(objects[min_index].primitive_poses[0].position.x, 
      objects[min_index].primitive_poses[0].position.y, 
      objects[min_index].primitive_poses[0].position.z) );

    tf::Quaternion microQ(objects[min_index].primitive_poses[0].orientation.x,
      objects[min_index].primitive_poses[0].orientation.y,
      objects[min_index].primitive_poses[0].orientation.z,
      objects[min_index].primitive_poses[0].orientation.w);
    microTF.setRotation(microQ);
    std::stringstream ss;
    ss << id;
    std::cout << "****\n" << ss.str() << "\n****" << std::endl;
    br.sendTransform(tf::StampedTransform(microTF, ros::Time::now(), "/base", ss.str()));

    capture = false;
    return true;
  }



};

 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_choice", ros::init_options::AnonymousName);

  ros::NodeHandle node_handle;
  ros::Rate loop_rate(10);

  ObjectChoice objsc(node_handle);

  ros::AsyncSpinner spinner(1);
  spinner.start();


  while (ros::ok()) {
    objsc.run();
    // upobjs.publishObjectTF();
    objsc.point2objects("pointing");
    loop_rate.sleep();
  }

  return 0;
};

