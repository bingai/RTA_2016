#include <attr_detection/AttributeProcess.h>

#include <graph_filter/ZAdaptiveNormals.hh>
#include <graph_filter/GraphUtilities.h>
#include <graph_filter/GraphUtilities.h>

#include <pcl_ros/point_cloud.h>
// AttributeProcess::AttributeProcess():viewer(new pcl::visualization::PCLVisualizer),
//   cloud_viewer("simple") {
AttributeProcess::AttributeProcess() {
   // tf_pub = node_handle.advertise<pcl::PointCloud<pcl::PointXYZRGBNormal>  > (
   //   "/detect/visual", 1); 
   initialize();
}

AttributeProcess::~AttributeProcess() {
}


// AttributeProcess::AttributeProcess(const pcl::visualization::PCLVisualizer & pv):viewer(pv){
//   initialize();
// }

void AttributeProcess::initialize() {
  tableIndices.clear();
  planePointIndices.clear();
  planecoeffs.clear();
  nbgh_matrix.clear();
}

void AttributeProcess::generateGraph() {
  segmentPlane();
  computeNeighborsOrganized();
}


void AttributeProcess::ConvertPCLCloud2ColorSeg(const std::vector<int>& planeIndices,
       pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &out) {

  out.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  // out->points.resize(input_->points.size());
  out->is_dense = false;
  float color = Graph::Util::GetRandomColor();
  
  for (int i = 0; i < planeIndices.size(); i++) {
    for (int j = 0; j < planePointIndices[planeIndices[i]].size(); j++ ) {
      pcl::PointXYZRGBNormal pt = 
        input_->points[planePointIndices[planeIndices[i] ][j]];
      // pt.rgb = color;
      out->points.push_back(pt);
    }
  }
  out->width = out->points.size();
  out->height = 1;
}



void AttributeProcess::setInputCloudAndTransform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_,
  const Eigen::Affine3d & T_Eigen) {
  this->T_Eigen = T_Eigen;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = cloud_;
  pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
  ZAdaptiveNormals::Parameter za_param;
  za_param.adaptive = true;
  ZAdaptiveNormals nor (za_param);
  nor.setInputCloud (cloud);
  nor.compute ();
  nor.getNormals (normal);

  // pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  // pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> norm_est;
  // norm_est.setSearchMethod(tree);
  // norm_est.setRadiusSearch(0.04);
  // norm_est.setInputCloud(cloud);
  // norm_est.compute(*normal);

  orig_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::concatenateFields(*cloud, *normal, *orig_cloud_);
 
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_trans (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::transformPointCloudWithNormals(*orig_cloud_, *cloud_trans, T_Eigen.cast<float>());
  this->input_ = cloud_trans;
}

void AttributeProcess::setTimeStamp(ros::Time stamp_) {
  frame_time_stamp = stamp_;
}

void AttributeProcess::segmentPlane() {
  segmentPlaneOrganized();
}

void AttributeProcess::segmentPlaneOrganized() {
  FindPlanes::Parameter param;
    param.adaptive = true;
    param.thrAngle = 0.3;
    // int detail = 1;
    // if(detail == 1) {
    //   param.epsilon_c = 0.58;
    //   param.omega_c = -0.002;
    // } else if (detail == 2) {
    //   param.epsilon_c = 0.62;
    //   param.omega_c = 0.0;
    // } 
    param.epsilon_c = 0.62;
    param.omega_c = -0.003;

    param.minPoints = 200;
    FindPlanes fp (param);
    fp.setInputCloud (orig_cloud_);
    fp.setOrigCloud(orig_cloud_);
    fp.setPixelCheck (true, 5);
    planePointIndices = fp.compute ();
    planecoeffs = fp.planecoeffs;
    transformSurfaceNormals();
    std::cout << "exist segmentPlane organized. # planes: " 
      << planePointIndices.size() << std::endl;
}


void AttributeProcess::transformSurfaceNormals() {
   // do the transform from camera frame to robot base frame
  for (unsigned i = 0; i < planecoeffs.size(); i++) {
    // std::cout << planecoeffs[i] << std::endl;
    Eigen::Matrix<float, 3, 1> nt (planecoeffs[i].normal[0], 
      planecoeffs[i].normal[1], planecoeffs[i].normal[2]);
    planecoeffs[i].normal[0] = static_cast<float> (T_Eigen (0, 0) * nt.coeffRef (0) + T_Eigen(0, 1) * nt.coeffRef (1) + T_Eigen(0, 2) * nt.coeffRef (2));
    planecoeffs[i].normal[1]= static_cast<float> (T_Eigen(1, 0) * nt.coeffRef (0) + T_Eigen(1, 1) * nt.coeffRef (1) + T_Eigen(1, 2) * nt.coeffRef (2));
    planecoeffs[i].normal[2] = static_cast<float> (T_Eigen(2, 0) * nt.coeffRef (0) + T_Eigen(2, 1) * nt.coeffRef (1) + T_Eigen(2, 2) * nt.coeffRef (2));
  }
}



void AttributeProcess::computeNeighborsOrganized() {

  double d_max = 0.02;
  cv::Mat_<int> patches(input_->height, input_->width);
  patches.setTo(-1);
  for(int i=0; i< planePointIndices.size(); i++) {
    for(int j=0; j<planePointIndices[i].size(); j++) {
      patches.at<int>( planePointIndices[i][j]/ input_->width, 
        planePointIndices[i][j] % input_->width) = i;
    }
  }

  int numPlanesDected = planePointIndices.size();
  nbgh_matrix.clear();
  nbgh_matrix.resize(numPlanesDected);
  for(unsigned i=0; i<numPlanesDected; i++) {
    std::vector<bool> temp(numPlanesDected, false);
    nbgh_matrix[i] = temp;
  }

  std::cout << "before for in neighbor" << std::endl;

  for(int row=1; row<patches.rows; row++) { 
    for(int col=1; col<patches.cols; col++) {
      if(patches.at<int>(row, col) == -1) {
        continue;
      }

      std::vector<std::pair<int, int> > ngbrs = Graph::Util::imageLeftUpNgbr(
        std::pair<int, int>(col, row), patches.cols, patches.rows
      );

      for (int i = 0; i < ngbrs.size(); i++) {
        int ncol = ngbrs[i].first, nrow = ngbrs[i].second;
        if (patches.at<int>(nrow, ncol) == -1 || 
          patches.at<int>(row, col) == patches.at<int>(nrow, ncol)) {
          continue;
        }
        int pos_0 = row*patches.cols+col;
        int pos_1 = nrow*patches.cols+ncol;
        double dis = (input_->points[pos_0].getVector3fMap() - 
          input_->points[pos_1].getVector3fMap() ).norm();
        if( dis < d_max) {
          nbgh_matrix[patches.at<int>(nrow, ncol)][patches.at<int>(row, col)] = true;
          nbgh_matrix[patches.at<int>(row, col)][patches.at<int>(nrow, ncol)] = true;
        }
        
      }

      
    }
  }
  std::cout << "exist OrganizedNeighbor" << std::endl;

}

void AttributeProcess::findTabletop() {
  //find clusters of planes that orientation is vertical
  int numOfPlanes = planePointIndices.size();
  std::vector<bool> visited(numOfPlanes, false);
  std::vector<std::vector<int> > clusters;
  for (int i = 0; i < numOfPlanes; i++) {
    if (visited[i] || fabs(planecoeffs[i].getNormalVector3fMap().dot(
      Eigen::Vector3f(0,0,1) ) ) < 0.97) {
      continue;
    }

    std::vector<int> que;
    int top = 0;
    que.push_back(i);
    visited[i] = true;
    while (top < que.size() ) {
      int pi = que[top];
      top++;
      for (int j = 0; j < nbgh_matrix[pi].size(); j++) {
        if (!nbgh_matrix[pi][j] || visited[j]) {
          continue;
        }
        if ( fabs(planecoeffs[j].getNormalVector3fMap().dot(
          Eigen::Vector3f(0,0,1) ) ) < 0.97) {
          continue;
        }


        que.push_back(j);
        visited[j] = true;
      }
    }
    clusters.push_back(que);
  }
  
  std::cout << "num of flat planes " << clusters.size() << std::endl;
  // find cluster index
  int tableIndex = -1;
  int tablePoints = 0;
  for (int i = 0; i < clusters.size(); i++) {
    int numPoint = 0;
    for (int j = 0; j < clusters[i].size(); j++) {
      numPoint += planePointIndices[clusters[i][j] ].size();
    }
    if (numPoint > tablePoints) {
      tablePoints = numPoint;
      tableIndex = i;
    }
  }

  tableIndices = clusters[tableIndex];
}

void AttributeProcess::visualizeTable(ros::Publisher& tf_pub) {

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr color(
    new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  ConvertPCLCloud2ColorSeg(tableIndices, color);

  color->header.frame_id = "/base";
  // std::cout << tableIndices.size() << " color size " << color->points.size() << std::endl;
  color->header.stamp = input_->header.stamp;
  tf_pub.publish(color);

}

void AttributeProcess::getTableTopObjects() {

}

void AttributeProcess::run(ros::Publisher& tf_pub) {
  generateGraph();
  findTabletop();
  if (tableIndices.size() == 0) {
    return;
  }

  visualizeTable(tf_pub);

}