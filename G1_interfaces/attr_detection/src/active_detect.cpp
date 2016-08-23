
#include <fstream>
#include <attr_detection/active_detect.h>
#include <vtkPolyLine.h>
#include <graph_filter/GraphFilter.h>
#include <hand_tracker_2d/HandBBox.h>
#include <attr_detection/AttributeProcess.h>

// ActiveDetect::ActiveDetect(ros::NodeHandle& nh_):viewer2("Simple") {
ActiveDetect::ActiveDetect(ros::NodeHandle& nh_):ap()
{
  tf_listener = new tf::TransformListener();
  subl = nh_.subscribe("/camera/depth_registered/points", 1, &ActiveDetect::callback, this);

  tf_pub = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGBNormal>  > (
    "/detect/visual", 1); 
  // subl = nh_.subscribe("/remote/camera/depth_registered/points", 1, &ActiveDetect::callback, this);

  // viewer.reset(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  // mcr(*viewer);


} 

ActiveDetect::~ActiveDetect(){

}


void ActiveDetect::ConvertPCLCloud2Image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, 
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


void ActiveDetect::ConvertPCLCloud2ColorSeg(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr &in,
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

void ActiveDetect::callback(const sensor_msgs::PointCloud2ConstPtr& cloud) {
  // std::cout << "call" << std::endl;
  if (!capture) {
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


void ActiveDetect::setCloudCaptureFlag(bool flag) {
  this->capture = flag;
}


bool ActiveDetect::readTFTransform(const std::string& target_frame, const std::string& source_frame,
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

bool ActiveDetect::addOneView() {
  while (pcdQue.size() == 0) {
    ROS_ERROR("no point cloud in the queue. Wait for a momement");
    capture = true;
    ros::Duration(1.0).sleep();
  }

  sensor_msgs::PointCloud2ConstPtr cloud = pcdQue[0];
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud, *cloud_in);

  Eigen::Affine3d T_Eigen;
  
  if (!readTFTransform("/base", cloud->header.frame_id, cloud->header.stamp, 3.0, T_Eigen) ) {
    std::cout << "tf issue" << std::endl;
    return false;
  }


  ap.initialize();
  ap.setInputCloudAndTransform(cloud_in, T_Eigen);
  ap.setTimeStamp(cloud->header.stamp);
  ap.run(tf_pub);
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "active_detect");	
  std::cout << "start active detect" << std::endl;
  ros::NodeHandle nh;


  ros::AsyncSpinner spinner(1);
  spinner.start();
  ActiveDetect active_detect(nh);

  srand (time(NULL));

  while (ros::ok()) {
    active_detect.setCloudCaptureFlag(true);
    ros::Duration(0.5).sleep();
    active_detect.addOneView();
    // ct.detectActiveDetect();
  }



}
