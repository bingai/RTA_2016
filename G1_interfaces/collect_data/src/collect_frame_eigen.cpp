
#include <fstream>
#include <collect_data/collect_frame_eigen.h>



CollectData::CollectData(ros::NodeHandle& nh_)
{
  tf_listener = new tf::TransformListener();
  // subl = nh_.subscribe("/camera/depth_registered/points", 1, &CollectData::callback, this);
  subl = nh_.subscribe("/remote_camera/depth_registered/points", 1, &CollectData::callback, this);

  // viewer.reset(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  // mcr(*viewer);
} 

CollectData::~CollectData(){

}


void CollectData::ConvertPCLCloud2Image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, 
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


void CollectData::callback(const sensor_msgs::PointCloud2ConstPtr& cloud) {
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


bool CollectData::collect() {
  moveDone = false;
  if (pcdQue.size() == 0) {
    ROS_ERROR("no point cloud in the queue");
    moveDone = true;

    return false;
  }

  sensor_msgs::PointCloud2ConstPtr cloud = pcdQue[0];
  moveDone = true;

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


  Eigen::Affine3d T_Eigen;
  tf::transformTFToEigen (transform, T_Eigen );
  
  Eigen::Matrix4d M = T_Eigen.matrix();

  static int counter = 0;
  std::cout << "eigen " << M << std::endl;
  std::ofstream eigenfile;
  std::stringstream sse;
  sse << "/home/arclab/arc_ws/data/" << "eigen" << counter << ".txt";

  eigenfile.open (sse.str().c_str());
  eigenfile << M;
  eigenfile.close();

  std::stringstream ss;
  ss << "/home/arclab/arc_ws/data/" << filename << counter++ << ".pcd";
  pcl::io::savePCDFile(ss.str(), *cloud1, true);


}


geometry_msgs::Pose CollectData::generateView(float x_dist, float y_dist, 
  float z_dist, Eigen::Vector3f centroid, Eigen::Vector3f xdir, 
  Eigen::Vector3f ydir, Eigen::Vector3f zdir) {

  Eigen::Vector3f pos = centroid + x_dist*xdir + y_dist*ydir + z_dist*zdir;
 
  std::cout << pos.transpose() << std::endl;
 
  Eigen::Vector3f z = (centroid - pos).normalized();
  Eigen::Vector3f y(0, 1, 0);
  y.normalize();
  Eigen::Vector3f x = y.cross(z).normalized();
  y = z.cross(x).normalized();

  Eigen::Matrix3f rotation;
  rotation.col(0) = x;
  rotation.col(1) = y;
  rotation.col(2) = z;

  Eigen::Quaternionf quad = Eigen::Quaternionf(rotation);
  return g1::control::constructPose(
      g1::control::constructPoint(pos[0], pos[1], pos[2]),
      g1::control::constructQuat(quad.x(), quad.y(), quad.z(), quad.w())
  );
}

void CollectData::addValidView(std::vector<geometry_msgs::Pose>& validviews, 
  geometry_msgs::Pose pose, g1::control::Executor *motion_controller) {
  
  motion_controller->reset();
  
  for (int i = 0 ; i < validviews.size(); i++) {
    std::vector<geometry_msgs::Pose> prev_poses(1, validviews[i]);
    boost::shared_ptr<g1::control::Move> prev_move_ptr(new g1::control::Move("Observe pose"));
    prev_move_ptr->setTargetPoses("left", prev_poses);
    motion_controller->addActionTarget(prev_move_ptr);
  }

  std::vector<geometry_msgs::Pose> target_poses(1, pose);
  boost::shared_ptr<g1::control::Move> new_move_ptr(new g1::control::Move("Observe pose"));
  new_move_ptr->setTargetPoses("left", target_poses);
  motion_controller->addActionTarget(new_move_ptr);
 
  if (!motion_controller->plan()) {
    ROS_ERROR("A viewpoint is not reachable");
  } else {
    validviews.push_back(pose);
  }
}


bool loadEigen(std::string eigenfile,Eigen::Affine3d& T){
  int cols = 0, rows = 0;
  double buff[16];

  // Read numbers from file into buffer.
  ifstream infile;
  infile.open(eigenfile.c_str());
  std::cout << "in eigen load " << eigenfile << std::endl;
  while (! infile.eof())
  {
    std::string line;
    std::getline(infile, line);
    // std::cout << "line " << line << std::endl;
    int temp_cols = 0;
    std::stringstream stream(line);
    while(! stream.eof()) {
        stream >> buff[cols*rows+temp_cols++];
    }

    if (temp_cols == 0)
        continue;

    if (cols == 0)
        cols = temp_cols;

    rows++;
  }


  infile.close();

  rows;

  // Populate matrix with numbers.
  Eigen::Matrix4d result;
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
        result(i,j) = buff[ cols*i+j ];
    }
  }

  T = result;
  return true;
}

void CollectData::setCloudCaptureFlag(bool flag) {
  this->moveDone = flag;
}


void CollectData::startCaptureData(g1::control::Executor *motion_controller) {
  std::vector<geometry_msgs::Pose> validviews;
  Eigen::Vector3f centroid(0.957, 0.167, -0.160);
  Eigen::Vector3f xdir(1, 0, 0);  
  Eigen::Vector3f ydir(0, 1, 0);
  Eigen::Vector3f zdir(0, 0, 1);  
  // for (float y = 0.3; y < 0.9; y += 0.1) {
  //   geometry_msgs::Pose target_pose = generateView(-0.45, y, 0.2,
  //      centroid, xdir, ydir, zdir
  //   );

  //   addValidView(validviews, target_pose, motion_controller);

  // }
 
  // std::cout << "planning finished. valid views: " << validviews.size() <<std::endl;
  // motion_controller->reset();
  // for (int i = 0 ; i < validviews.size(); i++) {
  //   std::vector<geometry_msgs::Pose> poses(1, validviews[i]);
  //   boost::shared_ptr<g1::control::Move> move_ptr(new g1::control::Move("Observe pose"));
  //   move_ptr->setTargetPoses("left", poses);
  //   motion_controller->addActionTarget(move_ptr);
  // }

  // moveDone = false;
  // if (!motion_controller->run()) {
  //   ROS_ERROR("Execution faliture");
  // }

  for (float x = -0.55; x < -0.3; x += 0.1) {
    for (float z = 0.5; z < 0.9; z+= 0.1) {
      for (float y = 0.0; y < 0.5; y += 0.1) {
        validviews.clear();
        geometry_msgs::Pose target_pose = generateView(x, y, z,
           centroid, xdir, ydir, zdir
        );

        addValidView(validviews, target_pose, motion_controller);
        std::cout << "planning finished. valid views: " << validviews.size() <<std::endl;
        if (validviews.size() == 0) {
          continue;
        }
        motion_controller->reset();
        std::vector<geometry_msgs::Pose> poses(1, validviews[0]);
        boost::shared_ptr<g1::control::Move> move_ptr(new g1::control::Move("Observe pose"));
        move_ptr->setTargetPoses("left", poses);
        motion_controller->addActionTarget(move_ptr);
        moveDone = false;

        if (!motion_controller->run()) {
          ROS_ERROR("Execution faliture");
        }
        moveDone = true;
       ros::Duration(1).sleep(); 

        collect();

     }

    }
  }



}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "data collection");	
  std::cout << "start data collection" << std::endl;
  ros::NodeHandle nh;
  CollectData ct(nh);
  ct.showParts = 0;
  pcl::console::parse_argument (argc, argv, "-p", ct.showParts);

  ct.filename = "graph_test_data";
  pcl::console::parse_argument (argc, argv, "-name", ct.filename);

  ros::AsyncSpinner spinner(1);
  spinner.start();


  g1::control::Executor::Options opt(nh, "left", "electric", "reflex");
  g1::control::Executor motion_controller(opt);

  motion_controller.resetArms();
  srand (time(NULL));
 //------------------------------------------------------//
  ct.setCloudCaptureFlag(false);
  ct.startCaptureData(&motion_controller);

  // ----------------------------------------------------//
  // std::string id = "1";
  // if (argc > 1) {
  //   id = argv[1];
  // }
  
  // std::stringstream ss;
  // ss <<"/home/lwt1104/pcd/extest" << id << ".pcd";
  // std::string cloudFilename = ss.str();
  // ss.str("");
  // ss <<"/home/lwt1104/pcd/eigen" << id <<".txt";
  // std::string eigenfile = ss.str();
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr sceneCloud (
  //   new pcl::PointCloud<pcl::PointXYZRGB>);
  // if (pcl::io::loadPCDFile(cloudFilename, *sceneCloud))
  // {
  //   std::cout << "Could not load pointcloud from file:" << std::endl;
  //   std::cout << cloudFilename << std::endl;
  //   return -1;
  // }
  
  // Eigen::Affine3d T;
  // loadEigen(eigenfile, T);
  // ct.runWithPointCloud(sceneCloud, T);
  // ----------------------------------------------------//  

  // while (ros::ok()) {
  //   ct.setCloudCaptureFlag(true);
  //   ros::Duration(1).sleep();
  //   ct.collect();
  // }



}
