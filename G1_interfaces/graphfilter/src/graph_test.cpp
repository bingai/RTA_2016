
#include <fstream>
#include <graph_filter/graph_test.h>
#include <vtkPolyLine.h>

#include <hand_tracker_2d/HandBBox.h>

// GraphTest::GraphTest(ros::NodeHandle& nh_):viewer2("Simple") {
GraphTest::GraphTest(ros::NodeHandle& nh_): mcr()
{
  tf_listener = new tf::TransformListener();
  subl = nh_.subscribe("/camera/depth_registered/points", 1, &GraphTest::callback, this);
  // subl = nh_.subscribe("/remote/camera/depth_registered/points", 1, &GraphTest::callback, this);

  // viewer.reset(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  // mcr(*viewer);

  bbox_pub_ = nh_.advertise<hand_tracker_2d::HandBBox>("/interact/hand_location", 1);

} 

GraphTest::~GraphTest(){

}


void GraphTest::ConvertPCLCloud2Image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, 
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


void GraphTest::ConvertPCLCloud2ColorSeg(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr &in,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out) {
 
 	out.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    out->points.resize(in->width*in->height);
    out->is_dense = false;


    std::map<int, float> label2color;
    for(unsigned i=0; i<in->points.size(); i++) {
      if (label2color.find(in->points[i].label) == label2color.end()) {
      	label2color[in->points[i].label] = GraphFilter::GetRandomColor();
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

void GraphTest::callback(const sensor_msgs::PointCloud2ConstPtr& cloud) {
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


bool GraphTest::detectGraphTest() {
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

  // static int counter = 3;
  // if (counter == 3) {
  //   std::cout << "eigen " << M << std::endl;
  //   std::ofstream eigenfile;
  //   std::stringstream sse;
  //   sse << "/home/arclab/arc_ws/" << "eigen" << counter << ".txt";

  //   eigenfile.open (sse.str());
  //   eigenfile << M;
  //   eigenfile.close();

  //   std::stringstream ss;
  //   ss << "/home/arclab/arc_ws/" << filename << counter++ << ".pcd";
  //   pcl::io::savePCDFile(ss.str(), *cloud1, true);
  // }

  // pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pcl_cloud_labeled(new pcl::PointCloud<pcl::PointXYZRGBL>);
  
  mcr.initialize();

  mcr.setInputCloudAndTransform(cloud1, T_Eigen);
  mcr.setTimeStamp(cloud->header.stamp);
  mcr.generateGraph();
  mcr.constraintsFiltering();

  std::vector<hand_tracker_2d::HandBBox> boxes = mcr.getBBoxes();
  for (int i = 0; i < boxes.size(); i++) {
    std::cout << "**** " << i << std::endl;
    bbox_pub_.publish(boxes[i]);
  }
  // mcr.cloudView(showParts);
  // mcr.visualization();
  // mcr.setUseBoxDetect(true);
  // mcr.findMicrowave();

  std::vector<std::vector<int> > planePointIndices = mcr.planePointIndices;
  

}


void GraphTest::runWithPointCloud(const  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
    Eigen::Affine3d& T_Eigen) {

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cp = cloud->makeShared();
  mcr.initialize();

  mcr.setInputCloudAndTransform(cloud_cp, T_Eigen);
  mcr.generateGraph();
  mcr.visualization();
  std::cout << "handle points number " <<mcr.handleIndices.size() << std::endl;

}

bool loadEigen(std::string eigenfile,Eigen::Affine3d& T){
  int cols = 0, rows = 0;
  double buff[16];

  // Read numbers from file into buffer.
  ifstream infile;
  infile.open(eigenfile);
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

void GraphTest::setCloudCaptureFlag(bool flag) {
  this->moveDone = flag;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "microwave_detect");	
  std::cout << "start microwave detect" << std::endl;
  ros::NodeHandle nh;
  GraphTest ct(nh);
  ct.showParts = 0;
  pcl::console::parse_argument (argc, argv, "-p", ct.showParts);

  ct.filename = "graph_test_data";
  pcl::console::parse_argument (argc, argv, "-name", ct.filename);

  ros::AsyncSpinner spinner(1);
  spinner.start();


  // ct.filename = "extest";
  // motion_controller.resetArms();
  // ct.run(&motion_controller);
  // ct.runViewPlan(& motion_controller);
  srand (time(NULL));

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
  while (ros::ok()) {
    ct.setCloudCaptureFlag(true);
    ros::Duration(0.5).sleep();
    ct.detectGraphTest();
  }



}
