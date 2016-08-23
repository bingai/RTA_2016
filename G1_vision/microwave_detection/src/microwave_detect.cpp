
#include <microwave_detection/mircowave_detect.h>


// Microwave::Microwave(ros::NodeHandle& nh_):viewer2("Simple") {
Microwave::Microwave(ros::NodeHandle& nh_) {
  tf_listener = new tf::TransformListener();
  
  // subl = nh_.subscribe("/rta_openni/point_cloud_color", 1, &Microwave::callback, this);

  subl = nh_.subscribe("/camera/depth_registered/points", 1, &Microwave::callback, this);

  // subl = nh_.subscribe("/remote/camera/depth_registered/points", 1, &Microwave::callback, this);

  update_client_ = nh_.serviceClient<world_model_msgs::UpdateStatesObjects>("/world_model/update_states_objects");
  if (!update_client_.waitForExistence(ros::Duration(2))) {
    ROS_ERROR("Cannot connect to the service");
  }
  ROS_INFO("Microwave vision initialized.");  

} 

Microwave::~Microwave(){

}


void Microwave::ConvertPCLCloud2Image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, 
                                                cv::Mat_<cv::Vec3b> &image)
{
  unsigned pcWidth = pcl_cloud->width;
  unsigned pcHeight = pcl_cloud->height;
  unsigned position = 0;
  
  image = cv::Mat_<cv::Vec3b>(pcHeight, pcWidth);

  for (unsigned row = 0; row < pcHeight; row++) {
    for (unsigned col = 0; col < pcWidth; col++) {
      cv::Vec3b &cvp = image.at<cv::Vec3b> (row, col);
      position = row * pcWidth + col;
      const pcl::PointXYZRGB &pt = pcl_cloud->points[position];

      cvp[2] = pt.r;
      cvp[1] = pt.g;
      cvp[0] = pt.b;
    }
  }
}


void Microwave::ConvertPCLCloud2ColorSeg(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr &in,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out) {
 
 	out.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    out->points.resize(in->width*in->height);
    out->is_dense = false;


    std::map<int, float> label2color;
    for(unsigned i=0; i<in->points.size(); i++) {
      if (label2color.find(in->points[i].label) == label2color.end()) {
      	label2color[in->points[i].label] = MicrowaveRect::GetRandomColor();
      } 
    }
    label2color[255]=1.2f;


    for (unsigned row = 0; row < in->height; row++) {
      for (unsigned col = 0; col < in->width; col++) {
        int idx = row * in->width + col;
        pcl::PointXYZRGBL &pt = in->points[idx];
        pcl::PointXYZRGB &npt = out->points[idx];
        npt.x = pt.x;
        npt.y = pt.y;
        npt.z = pt.z;
        npt.rgb = label2color[pt.label];
      }
    }


 }

void Microwave::callback(const sensor_msgs::PointCloud2ConstPtr& cloud) {
  // std::cout << "call" << std::endl;
  if (!moveDone) {
    return;
  }
  // freshNum--;
  // std::cout << "callback" << std::endl;

  if (cloud->width * cloud->height == 0) {
    return;
  }

  if (pcdQue.size() == 0) {
    pcdQue.push_back(cloud);
  } else {
    pcdQue[0] = cloud;
  } 
}


bool Microwave::detectMicrowave() {
  moveDone = false;
  if (pcdQue.size() == 0) {
    ROS_ERROR("no point cloud in the queue");
    moveDone = true;

    return false;
  }

  sensor_msgs::PointCloud2ConstPtr cloud = pcdQue[0];
  moveDone = true;
  
  std::cout << cloud->header.frame_id << std::endl;
  //TO DO: CHECK IF TF IS CORRECT !!!!
  tf_listener->waitForTransform("/base", cloud->header.frame_id, cloud->header.stamp, ros::Duration(3.0));
  tf::StampedTransform transform;
  try {
    tf_listener->lookupTransform ("/base", cloud->header.frame_id, cloud->header.stamp, transform);
  } catch (tf::LookupException &e) {
    ROS_ERROR ("%s", e.what ());
    return false;
  } catch (tf::ExtrapolationException &e) {
    ROS_ERROR ("%s", e.what ());
    return false;
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud, *cloud1);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector< int > index;
  pcl::removeNaNFromPointCloud (*cloud1, *cloud2, index);
 
  std::cout << "get point cloud" << cloud2->points.size() << " " << cloud2->width << std::endl;

  Eigen::Affine3d T_Eigen;
  tf::transformTFToEigen (transform, T_Eigen );
  std::cout << "transform " << T_Eigen.matrix() << std::endl;



  pcl::PointCloud<pcl::Normal>::Ptr cloud_n (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> norm_est;
  norm_est.setSearchMethod(tree);
  norm_est.setRadiusSearch(0.03);
  norm_est.setInputCloud(cloud1);
  norm_est.compute(*cloud_n);
  std::cout << "after norm " << std::endl;

  pcl::concatenateFields(*cloud1, *cloud_n, *cloud_f);

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_trans (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    
  pcl::transformPointCloudWithNormals(*cloud_f, *cloud_trans, T_Eigen.cast<float>());

  if (cloud_trans->width * cloud_trans->height == 0) {
  	return false;
  }

  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pcl_cloud_labeled(new pcl::PointCloud<pcl::PointXYZRGBL>);
  pcl::copyPointCloud (*cloud_trans, *pcl_cloud_labeled);
  
  mcr.initialize();
  mcr.setInputAndOrigCloud(cloud_trans, cloud_f);
  mcr.setTransform(T_Eigen);
  // mcr.setUseBoxDetect(true);
  mcr.findMicrowave();

  std::vector<std::vector<int> > planePointIndices = mcr.planePointIndices;
  // if (mcr.handleIndices.size() > 0) {
   // mcr.visualization();
  // }


  // std::vector<int> activeSurface;
  //   for (unsigned i = 0; i < planePointIndices.size (); i++) {
  //     // std::cout << activeSurface[i] << std::endl;
  //     for (unsigned j = 0; j < planePointIndices[i].size (); j++) {
  //       pcl_cloud_labeled->points[planePointIndices[i][j]].label = i+1;
  //     }
  //   }


  // std::vector<int> activeSurface;
  // for (int k = 0; k < mcr.component.size(); k++) {
  //   activeSurface =  mcr.frontDoor;
  //   for (unsigned i = 0; i < activeSurface.size (); i++) {
  //     // std::cout << activeSurface[i] << std::endl;
  //     for (unsigned j = 0; j < planePointIndices[activeSurface[i]].size (); j++) {
  //       pcl_cloud_labeled->points[planePointIndices[activeSurface[i]][j]].label = k+1;
  //     }
  //   }
  // }

  std::cout << "handle points number " <<mcr.handleIndices.size() << std::endl;
  // for (int i = 0; i < mcr.handleIndices.size(); i++) {
  //   pcl_cloud_labeled->points[mcr.handleIndices[i]].label = 1;
  // }
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
  // ConvertPCLCloud2ColorSeg(pcl_cloud_labeled, cloud_colored);
 
  // // viewer2.showCloud(cloud_colored);
  
  // Eigen::Vector3f c(0, 0, 0);
  // for (int i = 0; i < mcr.handleIndices.size(); i++) {
  //   c = c + cloud_trans->points[mcr.handleIndices[i]].getVector3fMap();
  // }
  // c = c / mcr.handleIndices.size();
  // handle_centroid = c;

  if (mcr.handleIndices.size()) {
    // static tf::TransformBroadcaster br;
    // tf::Transform microTF;
    // microTF.setOrigin(tf::Vector3(c[0], c[1], c[2]) );
    // tf::Quaternion microQ;
    // frontNormal = mcr.frontNormal;
    // float yaw = atan2(mcr.frontNormal[1], mcr.frontNormal[0]); 
    // microQ.setRPY(0, 0, yaw);
    // microTF.setRotation(microQ);
    // br.sendTransform(tf::StampedTransform(microTF, cloud->header.stamp, "/base", "/microwave"));
    // // microQ.setRPY(3.14, 3.14, yaw);
    // // microQ.setRPY(0, -1.57, 3.14);

    // Eigen::Vector3f x(0, 0,-1);
    // Eigen::Vector3f z(-1 *mcr.frontNormal[0], -1*mcr.frontNormal[1], 0);
    // z.normalize();
    // Eigen::Vector3f a(1,0,1);
    // a.normalize();
    // Eigen::Vector3f b = (x+z);
    // b.normalize();
    // frontOrientation = frontOrientation.FromTwoVectors(a, b);
    return true;

  } else {
    return false;
  }

}

bool Microwave::updateWorldStates() {
  world_model_msgs::UpdateStatesObjects srv;
  // pcl::toROSMsg(*reconstruct_cloud_, srv.request.cloud);
  if (!mcr.generateMessage(srv)) {
    ROS_ERROR("Failed to generate microwave info.");
    return false;
  }
  if (update_client_.call(srv))
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

geometry_msgs::Pose Microwave::generateView(float x_dist, float y_dist, float z_dist) {
  Eigen::Vector3f pos = mcr.handle_centroid + x_dist*mcr.frontNormal
    + y_dist*mcr.sideNormal + z_dist*mcr.topNormal;
 
  std::cout << x_dist << " " << y_dist << " " << z_dist << std::endl;
  // std::cout << y_dist << "\n\n" << pos << "\n\n" << mcr.handle_centroid << std::endl; 
  Eigen::Vector3f z = (mcr.handle_centroid - pos).normalized();
  Eigen::Vector3f y(0, 1, 0);
  y.normalize();
  Eigen::Vector3f x = y.cross(z).normalized();
  y = z.cross(x).normalized();

  Eigen::Matrix3f rotation;
  rotation.col(0) = x;
  rotation.col(1) = y;
  rotation.col(2) = z;

  // Eigen::JacobiSVD<Eigen::Matrix3f> svd(rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
  // rotation = svd.matrixU()*(svd.matrixV().transpose());
  // Eigen::Matrix3f U = svd.matrixU(), Vt = svd.matrixV().transpose();
  // Eigen::Matrix3f tmp = U * Vt;
  // Eigen::Matrix3f S = Eigen::Matrix3f::Identity();


  Eigen::Quaternionf quad = Eigen::Quaternionf(rotation);
  return g1::control::constructPose(
      g1::control::constructPoint(pos[0], pos[1], pos[2]),
      g1::control::constructQuat(quad.x(), quad.y(), quad.z(), quad.w())
  );
}

void Microwave::addValidView(std::vector<geometry_msgs::Pose>& validviews, 
  geometry_msgs::Pose pose, g1::control::Executor *motion_controller) {
  
  motion_controller->reset();
  
  for (int i = 0 ; i < validviews.size(); i++) {
    std::vector<geometry_msgs::Pose> prev_poses(1, validviews[i]);
    boost::shared_ptr<g1::control::Move> prev_move_ptr(new g1::control::Move("Observe pose"));
    prev_move_ptr->setTargetPoses("right", prev_poses);
    motion_controller->addActionTarget(prev_move_ptr);
  }

  std::vector<geometry_msgs::Pose> target_poses(1, pose);
  boost::shared_ptr<g1::control::Move> new_move_ptr(new g1::control::Move("Observe pose"));
  new_move_ptr->setTargetPoses("right", target_poses);
  motion_controller->addActionTarget(new_move_ptr);
 
  if (!motion_controller->plan()) {
    ROS_ERROR("A viewpoint is not reachable");
  } else {
    validviews.push_back(pose);
  }
}

std::vector<geometry_msgs::Pose> Microwave::runViewPlan(
  g1::control::Executor *motion_controller) {

  std::vector<geometry_msgs::Pose> validviews;
  moveDone = true;
  geometry_msgs::Pose target_pose = g1::control::constructPose(
      g1::control::constructPoint(0.322, -0.697, 0.250),
      g1::control::constructQuat(-0.393, 0.722, -0.034, 0.568)
    );

  moveDone = false;

  std::vector<geometry_msgs::Pose> target_poses(1, target_pose);
  boost::shared_ptr<g1::control::Move> move_ptr(new g1::control::Move("Observe pose"));
  move_ptr->setTargetPoses("right", target_poses);

  motion_controller->addActionTarget(move_ptr);
  if (!motion_controller->run()) {
    ROS_ERROR("Execution Failed. Aborting Microwave Detection.");
    return validviews;
  }
  // freshNum = 10;
  moveDone = true;
  ros::Duration(1).sleep();

  while (!detectMicrowave()) {
    ros::Duration(1).sleep(); 
  }

  // plan viewpoints for microwave reconstruction
  for (float y = -0.2; y < 0.4; y += 0.1) {
    target_pose = generateView(0.34, y, 0.35);
    addValidView(validviews, target_pose, motion_controller);

  }
 
  std::cout << "planning finished. valid views: " << validviews.size() <<std::endl;
  // target_poses.clear();
  // motion_controller->reset();
  // for (int i = 0 ; i < validviews.size(); i++) {
  //   std::vector<geometry_msgs::Pose> poses(1, validviews[i]);
  //   boost::shared_ptr<g1::control::Move> move_ptr(new g1::control::Move("Observe pose"));
  //   move_ptr->setTargetPoses("right", poses);
  //   motion_controller->addActionTarget(move_ptr);
  // }

  // moveDone = false;
  // if (!motion_controller->run()) {
  //   ROS_ERROR("Execution faliture");
  // }

  return validviews;

}

void Microwave::runWithPointCloud(const  pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr &cloud) {
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_cp = cloud->makeShared();
  mcr.initialize();
  mcr.setInputCloud(cloud_cp);
  // mcr.setUseBoxDetect(true);
  mcr.findMicrowave();
  mcr.visualization();
  std::cout << "handle points number " <<mcr.handleIndices.size() << std::endl;

}

void Microwave::run(g1::control::Executor *motion_controller) {
  // ros::Rate rate(30);

  moveDone = true;
  // geometry_msgs::Pose target_pose = g1::control::constructPose(
  //     g1::control::constructPoint(0.362, 0.498, 0.222),
  //     g1::control::constructQuat(0.336, 0.749, -0.232, 0.522)
  //   );

  geometry_msgs::Pose target_pose = g1::control::constructPose(
      g1::control::constructPoint(0.323, 0.470, 0.243),
      g1::control::constructQuat(0.218, 0.837, -0.151, 0.479)
    );

  // geometry_msgs::Pose target_pose = g1::control::constructPose(
  //     g1::control::constructPoint(0.373, 0.398, 0.219),
  //     g1::control::constructQuat(0.363, 0.770, -0.218, 0.476)
  //   );
  // Monday
  // geometry_msgs::Pose target_pose = g1::control::constructPose(
  //     g1::control::constructPoint(0.410, 0.561, 0.309),
  //     g1::control::constructQuat(0.231, 0.841, -0.143, 0.467)
  //   );

  moveDone = false;

  std::vector<geometry_msgs::Pose> target_poses(1, target_pose);
  boost::shared_ptr<g1::control::Move> move_ptr(new g1::control::Move("Observe pose"));
  move_ptr->setTargetPoses("left", target_poses);

  motion_controller->addActionTarget(move_ptr);
  if (!motion_controller->run()) {
    ROS_ERROR("Execution Failed. Aborting Microwave Detection.");
    return ;
  }
  // freshNum = 10;
  moveDone = true;
  ros::Duration(3).sleep();

  while (!detectMicrowave()) {
    ros::Duration(1).sleep();
  }
  // mcr.visualization();


  // Eigen::Vector3f pos = handle_centroid + 0.02*frontNormal;

  // target_poses.clear();
  // target_poses.push_back(
  //   g1::control::constructPose(
  //     g1::control::constructPoint(pos[0], pos[1]+0.03, pos[2]),
  //     g1::control::constructQuat(frontOrientation.x(), frontOrientation.y(), 
  //       frontOrientation.z(), frontOrientation.w())
  //   )
  // );

  // move_ptr.reset(new g1::control::Move("move to microwave"));
  // move_ptr->setTargetPoses("right", target_poses);

  // motion_controller->addActionTarget(move_ptr);

  // moveDone = false;
  // if (!motion_controller->run()) {
  //   ROS_ERROR("Execution Failed. Aborting Microwave Detection.");
  //   return ;
  // }
  // // freshNum = 10;
  // moveDone = true;
 


  // std::cout << frontOrientation.x() <<" " <<  frontOrientation.y()
  //   << " " << frontOrientation.z() << "  " <<  frontOrientation.w()
  //   << std::endl;
  // while (ros::ok()) {
  //   ros::Duration(1).sleep();
  //   // detectMicrowave();
  //   detectMicrowave();
  // }


}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "microwave_detect");	
  std::cout << "start microwave detect" << std::endl;
  ros::NodeHandle nh;
  Microwave ct(nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  g1::control::Executor::Options opt(nh, "left", "electric", "reflex");
  g1::control::Executor motion_controller(opt);

  motion_controller.resetArms();
  ct.run(&motion_controller);
  // ct.runViewPlan(& motion_controller);
  
  // std::string cloudFilename = "/home/arclab/arc_ws/auto_view.pcd";
  // pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr sceneCloud (
  //   new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  // if (pcl::io::loadPCDFile(cloudFilename, *sceneCloud))
  // {
  //   std::cout << "Could not load pointcloud from file:" << std::endl;
  //   std::cout << cloudFilename << std::endl;
  //   return -1;
  // }

  // ct.runWithPointCloud(sceneCloud);

  if (ct.updateWorldStates()) {
    ROS_INFO("Updated microwave in world states.");
  }


}