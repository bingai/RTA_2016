#include <ros/ros.h>


// PCL
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
// #include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

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

#include <eigen_conversions/eigen_msg.h>
#include <vtkRenderWindow.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <memory>

class world_projection {

private:
  ros::ServiceClient fetch_client_;
  ros::ServiceClient update_client_;

  tf::TransformListener *tf_listener;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

  
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

 
public:
  world_projection(ros::NodeHandle &node_handle):
    viewer(new pcl::visualization::PCLVisualizer ("3D Viewer")){
   
    // tf::TransformListener listener(ros::Duration(10)); 
    fetch_client_ = node_handle.serviceClient<world_model_msgs::GetStatesObjects>("/world_model/get_states_objects");
    if (!fetch_client_.waitForExistence(ros::Duration(2))) {
      ROS_ERROR("Cannot connect to the location service");
    }


    update_client_ = node_handle.serviceClient<world_model_msgs::UpdateStatesObjects>("/world_model/update_states_objects");
    if (!fetch_client_.waitForExistence(ros::Duration(2))) {
      ROS_ERROR("Cannot connect to the location service");
    }

   
    tf_listener = new tf::TransformListener();

    ROS_INFO("world_projection initialized.");
  }






  bool project() {

    world_model_msgs::GetStatesObjects srv;
    srv.request.object_id = "";
    srv.request.manipulatable = true;
    if (!fetch_client_.call(srv))
    {
      ROS_INFO_STREAM("\n\nWorld model server down\n\n");
      return false;
    }

    
    std::vector<world_model_msgs::Object> objects = srv.response.objects;

    srv.request.object_id = "";
    srv.request.manipulatable = false;
    if (!fetch_client_.call(srv))
    {
      ROS_INFO_STREAM("\n\nWorld model server down\n\n");
      return false;
    }
    
    objects.insert(objects.end(), 
      srv.response.objects.begin(), 
      srv.response.objects.end() );

    if (objects.size() == 0) {
      std::cout << "No objects in world model " << std::endl;
      return false;
    }

    // if (!readTFTransform("/base", cloud->header.frame_id, cloud->header.stamp, 3.0, T_Eigen) ) {
    //   std::cout << "tf issue" << std::endl;
    //   return false;
    // }
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(0.5);
    viewer->initCameraParameters();
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    Eigen::Vector3f focus(-0.707107, 0, -0.707107);
    Eigen::Vector3f pos(5.25724, -0.684991, 1.52054);
    Eigen::Vector3f up(-0.354892, -0.0549771, 0.933289);    
    Eigen::Vector3f vdir = (focus - pos).normalized();
    vdir.normalize();
    viewer->setCameraPosition(pos(0), pos(1), pos(2), 
      vdir(0), vdir(1), vdir(2),
      up(0), up(1), up(2)  
      );
      
    for (int i = 0; i < objects.size(); i++) {
      for (int j = 0; j < objects[i].primitives.size(); j++) {
        Eigen::Vector3d trans;
        tf::pointMsgToEigen (objects[i].primitive_poses[j].position, trans);
        Eigen::Quaterniond rotation;
        tf::quaternionMsgToEigen (objects[i].primitive_poses[j].orientation, rotation);
        
        std::cout << objects[i].id << std::endl;

        if (objects[i].primitives[j].type == objects[i].primitives[j].BOX) {
          viewer->addCube(trans.cast<float>(), rotation.cast<float>(), 
            objects[i].primitives[j].dimensions[0],
            objects[i].primitives[j].dimensions[1], 
            objects[i].primitives[j].dimensions[2], objects[i].id);
     
          viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR, 0.1, 0.3, 0.2 /*R,G,B*/, 
            objects[i].id);
        
        } else if (objects[i].primitives[j].type == objects[i].primitives[j].CYLINDER) {
            pcl::ModelCoefficients cylinder_coeff;
            cylinder_coeff.values.resize (7);    // We need 7 values
            
            Eigen::Vector3f zdir = rotation.matrix().col(2).cast<float>();
            Eigen::Vector3f top = 
              trans.cast<float>() + zdir * objects[i].primitives[j].dimensions[0];
            
            cylinder_coeff.values[3] = trans(0);
            cylinder_coeff.values[4] = trans(1);
            cylinder_coeff.values[5] = trans(2);
            cylinder_coeff.values[0] = top(0);
            cylinder_coeff.values[1] = top(1);
            cylinder_coeff.values[2] = top(2);

            cylinder_coeff.values[6] = objects[i].primitives[j].dimensions[1];
            viewer->addCylinder(cylinder_coeff, objects[i].id);
        }


      }      
    }


    viewer->spin();
    viewer->saveScreenshot("screenshot.png"); 

    vtkSmartPointer<vtkRenderWindow> render = viewer->getRenderWindow();
    std::unique_ptr<uchar> pixels(
      render->GetRGBACharPixelData( 0, 0, render->GetSize()[0] - 1, 
        render->GetSize()[1] - 1, 1 ) );
    
    // Rendered Image to cv::Mat
    cv::Mat image = cv::Mat(render->GetSize()[1], render->GetSize()[0], CV_8UC4, &pixels.get()[0] );
    cv::cvtColor( image, image, cv::COLOR_RGBA2BGRA );
    cv::flip( image, image, 0 );

    cv::imshow("aaa",image);
    cv::waitKey(0);
    imwrite( "Gray_Image.jpg", image);

    return true;
  }



};

 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "world_projection", ros::init_options::AnonymousName);

  ros::NodeHandle node_handle;
  ros::Rate loop_rate(10);

  world_projection wp(node_handle);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  wp.project();

  // while (ros::ok()) {
  //   wp.project();
  //   loop_rate.sleep();
  // }

  return 0;
};

