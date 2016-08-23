#include <graph_filter/GraphFilter.h>
#include <namaris/utilities/pcl_typedefs.hpp>
#include <namaris/algorithms/region_growing_normal_variation/region_growing_normal_variation.hpp>
// #include <sisyphus/box_detection.hpp>

#include <graph_filter/ZAdaptiveNormals.hh>
#include <graph_filter/GraphUtilities.h>

// GraphFilter::GraphFilter():viewer(new pcl::visualization::PCLVisualizer),
//   cloud_viewer("simple") {
GraphFilter::GraphFilter() {
   initialize();
}

GraphFilter::~GraphFilter() {
}


// GraphFilter::GraphFilter(const pcl::visualization::PCLVisualizer & pv):viewer(pv){
//   initialize();
// }

void GraphFilter::initialize() {
  use_box = false;
  handleIndices.clear();
  boxes.clear();
  rays.clear();
}

void GraphFilter::generateGraph() {
  segmentPlane();
  // computeNeighbors();
}

void GraphFilter::setUseBoxDetect(bool use_box_) {
  use_box = use_box_;
}

void GraphFilter::computeCentroid(std::map<int,  pcl::PointXYZ>& centroid) {
  for (int i=0; i < planePointIndices.size(); i++) {
    float x = 0, y = 0, z = 0;
    int num = 0;
    for (int j = 0; j < planePointIndices[i].size(); j++) {
      pcl::PointXYZRGBNormal& p = input_->points[planePointIndices[i][j]]; 
      if (!pcl::isFinite(p)) {continue;}
      x += p.x;
      y += p.y;
      z += p.z;
      num++;
    }
    x /= num;
    y /= num;
    z /= num;
    pcl::PointXYZ tt(x,y,z);
    centroid.insert(std::pair<int, pcl::PointXYZ>(i, tt));
  }
}

void GraphFilter::computePatchNorm(std::map<int, pcl::Normal>& PatchMeanN) {
  if (input_->isOrganized ()) {
    for (int i=0; i < planePointIndices.size(); i++) {
      PatchMeanN.insert(std::pair<int, pcl::Normal>(i, planecoeffs[i]));
    }
  } else {
    for (int i=0; i < planePointIndices.size(); i++) {
      float n_x = 0, n_y = 0, n_z = 0;
      for (int j = 0; j < planePointIndices[i].size(); j++) {
        pcl::PointXYZRGBNormal& n = input_->points[planePointIndices[i][j]];
        if (!pcl::isFinite(n)) {continue;}
        n_x += n.normal_x;
        n_y += n.normal_y;
        n_z += n.normal_z;
      }
      float normalizeF = sqrt(n_x*n_x + n_y*n_y + n_z*n_z);
      n_x /= normalizeF;
      n_y /= normalizeF;
      n_z /= normalizeF;
      PatchMeanN.insert(std::pair<int, pcl::Normal>(i, pcl::Normal(n_x, n_y, n_z)));
    }

  }


}


void GraphFilter::setPointPlaneLabel() {
  for (int i = 0; i < planePointIndices.size(); i++) {
   	for (int j = 0; j < planePointIndices[i].size(); j++) {
       // pointPlaneLabel.insert(std::pair<int, int>(planePointIndices[i][j], i));
       pointPlaneLabel[planePointIndices[i][j]] = i;
   	}
  }
}

bool GraphFilter::tablePlaneUnaryConditionFunction(const PointNC& p, const Eigen::Vector3f &gravity_vector, const float gravity_angle_thresh) {
  Eigen::Vector3f normal = p.getNormalVector3fMap();
  // return (normal.dot(-gravity_vector) > std::cos(gravity_angle_thresh)) || (std::abs(normal.dot(gravity_vector)) < std::sin(gravity_angle_thresh));
  return pcl::isFinite(p);
}

bool GraphFilter::tablePlaneBinaryConditionFunction(const PointNC& p0, const PointNC& p1, float dist) {
  Eigen::Vector3f n0 = p0.getNormalVector3fMap();
  Eigen::Vector3f n1 = p1.getNormalVector3fMap();
  if (n0.dot(n1) > 0.94 || dist < 0.02) {
  	return true;
  }
}


void GraphFilter::setSearcher() {
  if (input_->isOrganized ()) {
    searcher_.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZRGBNormal> ());
  }
  else {
    searcher_.reset (new pcl::search::KdTree<pcl::PointXYZRGBNormal> ());
  }

  // indices_->resize(input_->size());
  // for (size_t pointId = 0; pointId < indices_->size(); pointId++) {
  //   (*indices_)[pointId] = pointId;
  // }

  searcher_->setInputCloud (input_);  

}

void GraphFilter::printNeighbors() {
  for(unsigned i=0; i<nbgh_matrix.size(); i++) {
	for (unsigned j = 0; j < nbgh_matrix[i].size(); j++) {
		std::cout << (nbgh_matrix)[i][j] << "\t";
    }
   std::cout << std::endl;
  }	
}


void GraphFilter::setTransform(const Eigen::Affine3d & T_Eigen) {
  this->T_Eigen = T_Eigen;
}


void GraphFilter::setInputCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_) {
  if (cloud_->points.size() == 0) {
    std::cout << "Error empty point cloud" << std::endl;
    return ;
  }
  std::vector<int> index;
  this->input_.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::removeNaNFromPointCloud(*cloud_, *this->input_, index);
}


void GraphFilter::setInputAndOrigCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_,
         pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr orig_cloud_){
    if (cloud_->points.size() == 0 || orig_cloud_->points.size() == 0) {
      std::cout << "Error empty point cloud" << std::endl;
      return ;
    }
    this->input_ = cloud_;
    this->orig_cloud_ = orig_cloud_;
}




void GraphFilter::findConnectedComponent() {
  std::vector<std::vector<int> > component;
  std::vector<int> surfaceState(planePointIndices.size(), 1);
  for  (unsigned i = 0; i < surfaceState.size(); i++) {

    if (surfaceState[i] != 1) {  // 0 means background; 1 denotes foreground but unknown componet; 2 means determined component
      continue;
    }
    std::vector<int> v;
    v.clear();
    std::queue<int> que;
    que.push(i);
    surfaceState[i] = 2;
    while (!que.empty()) {
      int ele = que.front();
      que.pop();
      v.push_back(ele);
      // find ele's neighbor's
      for (unsigned j = 0; j < surfaceState.size(); j++) {
        if (surfaceState[j] != 1) {
          continue;
        }
        // if (relations[ele][j] || relations[j][ele]) {
        if (nbgh_matrix[ele][j]) {
          que.push(j);
          surfaceState[j] = 2;
        }
      }
    }
    component.push_back(v); 

  }

  for (int i = 0; i < component.size(); i++) {
     for (int j = 0; j < component[i].size(); j++) {
       std::cout << component[i][j] << "\t";
     }
     std::cout << std::endl;
   }
  this->component = component;
}

void GraphFilter::computeNeighborsOrganized() {
  
  double z_max = 0.03;
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

  for(int row=1; row<patches.rows; row++) { 
    for(int col=1; col<patches.cols; col++) {
      if(patches.at<int>(row, col) != -1) {
        if(patches.at<int>(row-1, col) != -1) {
          if(patches.at<int>(row, col) != patches.at<int>(row-1, col)) {
            int pos_0 = row*orig_cloud_->width+col;
            int pos_1 = (row-1)*orig_cloud_->width+col;
            double dis = fabs(orig_cloud_->points[pos_0].z - orig_cloud_->points[pos_1].z);
            if( dis < z_max) {
              nbgh_matrix[patches.at<int>(row-1, col)][patches.at<int>(row, col)] = true;
              nbgh_matrix[patches.at<int>(row, col)][patches.at<int>(row-1, col)] = true;
            }
          }
        }
        if(patches.at<int>(row, col-1) != -1) {
          if(patches.at<int>(row, col) != patches.at<int>(row, col-1)) {
            int pos_0 = row*orig_cloud_->width+col;
            int pos_1 = (row)*orig_cloud_->width+col - 1;
            double dis = fabs(orig_cloud_->points[pos_0].z - orig_cloud_->points[pos_1].z);

            if( dis < z_max) {
              nbgh_matrix[patches.at<int>(row, col-1)][patches.at<int>(row, col)] = true;
              nbgh_matrix[patches.at<int>(row, col)][patches.at<int>(row, col-1)] = true;
            }
          }
        }
        if(patches.at<int>(row-1, col-1) != -1) {
          if(patches.at<int>(row, col) != patches.at<int>(row-1, col-1)) {
            int pos_0 = row*orig_cloud_->width+col;
            int pos_1 = (row-1)*orig_cloud_->width+col - 1;
            double dis = fabs(orig_cloud_->points[pos_0].z - orig_cloud_->points[pos_1].z);
            if( dis < z_max) {
              nbgh_matrix[patches.at<int>(row-1, col-1)][patches.at<int>(row, col)] = true;
              nbgh_matrix[patches.at<int>(row, col)][patches.at<int>(row-1, col-1)] = true;
            }
          }
        }
      }
    }
  }
  std::cout << "exist OrganizedNeighbor" << std::endl;

}

void GraphFilter::computeNeighborsUnOrganized() {
  setSearcher();
  setPointPlaneLabel();

  int numPlanesDected = planePointIndices.size();
  nbgh_matrix.clear();
  nbgh_matrix.resize(numPlanesDected);
  for(unsigned i=0; i<numPlanesDected; i++) {
  	std::vector<bool> temp(numPlanesDected, false);
    nbgh_matrix[i] = temp;
  }

  // #pragma omp parallel for
  for (int i = 0; i < planePointIndices.size(); i++) {
   	for (int j = 0; j < planePointIndices[i].size(); j++) {
   	  std::vector<int> nn_indices;
      std::vector<float> nn_distances;	
      int num_neighbors_found = searcher_->radiusSearch(input_->points[planePointIndices[i][j]], 
      	  0.05, nn_indices, nn_distances);

      int label_p = pointPlaneLabel[planePointIndices[i][j]];
      for (int k = 0; k < nn_indices.size(); k++) {
      	if (pointPlaneLabel.find(nn_indices[k]) == pointPlaneLabel.end()) {
      		continue;
      	}
        int label_n = pointPlaneLabel[nn_indices[k]]; 
      	(nbgh_matrix)[label_n][label_p] = true;
      	(nbgh_matrix)[label_p][label_n] = true;
      }
   	}
  }

}
void GraphFilter::computeNeighbors() {
  if (input_->isOrganized()) {
  	computeNeighborsOrganized();
  } else {
  	computeNeighborsUnOrganized();
  }
}

void GraphFilter::segmentPlaneUnOrganized() {
	Eigen::Vector3f gravityVector = Eigen::Vector3f(0, 0, 1).normalized();
	// Parameters
	float gravityVectorAngleThreshold = pcl::deg2rad(30.0);         // Maximum allowed difference between a table plane point normal and gravity vector
	float normalVariationThreshold    = pcl::deg2rad(10.0)*100;   // Maximum allowed normal difference between adjacent points (in rad/m)
	int minTablePlanePoints = 800;

	// Create unary condition function
	boost::function<bool (const PointNC&)> tablePlaneUnaryFunction =
	boost::bind(&GraphFilter::tablePlaneUnaryConditionFunction, this, _1, gravityVector, gravityVectorAngleThreshold);


	// Create region growing segmentation object
	alg::RegionGrowingNormalVariation<PointNC> rg;
	rg.setInputCloud(input_);
	rg.setConsistentNormals(true);
	rg.setNumberOfNeighbors(5);
	rg.setSearchRadius(0.01);
	rg.setNormalVariationThreshold(normalVariationThreshold);
	rg.setMinValidBinaryNeighborsFraction(0.7);
    // rg.setMinValidUnaryNeighborsFraction(0.01);
	rg.setUnaryConditionFunction(tablePlaneUnaryFunction);
	// rg.setBinaryConditionFunction(tablePlaneBinaryFunction);
	rg.setMinSegmentSize(minTablePlanePoints);
	rg.segment(planePointIndices);	
}
void GraphFilter::segmentPlaneOrganized() {
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

}
void GraphFilter::segmentPlane() {
  if (input_->isOrganized()) {
  	segmentPlaneOrganized();
  } else {
  	segmentPlaneUnOrganized();
  }

}

void GraphFilter::transformSurfaceNormals() {
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

void GraphFilter::ConvertPCLCloud2ColorSeg(const std::vector<int>& indices,
       pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &out) {

  out.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  out->points.resize(input_->points.size());
  out->is_dense = false;
  float color = GetRandomColor();

  for (int i = 0; i < indices.size(); i++) {
    pcl::PointXYZRGBNormal &pt = input_->points[indices[i]];
    pcl::PointXYZRGBNormal &npt = out->points[indices[i]];
    npt = pt;
    npt.rgb = color;
  }

}


void GraphFilter::ConvertPCLCloud2ColorSeg(const std::vector<std::vector<int> >& component,
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &out) {
 
 	out.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    out->points.resize(input_->points.size());
    out->is_dense = false;

	for (int i = 0; i < component.size(); i++) {
      float color = GetRandomColor();

      for (int j = 0; j < component[i].size(); j++) {

        for (int k = 0; k < planePointIndices[component[i][j]].size(); k++) {
           pcl::PointXYZRGBNormal &pt = input_->points[planePointIndices[component[i][j]][k]];
           pcl::PointXYZRGBNormal &npt = out->points[planePointIndices[component[i][j]][k]];
           npt = pt;
           npt.rgb = color;
        }
      }

    }

 }



void GraphFilter::cloudView(int remain) {
  static  boost::shared_ptr<pcl::visualization::CloudViewer>  cloud_viewer( 
    new pcl::visualization::CloudViewer("simple") );
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr color_planecloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  std::vector<std::vector<int> > planes;
  if (remain) {
    std::cout <<"Parts ";
    pcl::IndicesPtr ind(new std::vector<int>);
    for (int i = 0; i < planePointIndices.size(); i++) {
      if (scene2sub[i].size() > 0) {
      // if (scene2sub[i].find(0) !=scene2sub[i].end() ) {
        std::cout << i <<" : " <<  nbgh_list[i].size() << "|| \t";
        ind->insert(ind->end(), planePointIndices[i].begin(),
          planePointIndices[i].end());
      }
    }
    std::cout <<std::endl;

    pcl::ExtractIndices<pcl::PointXYZRGBNormal> eifilter (true); // Initializing with true will allow us to extract the removed indices
    eifilter.setInputCloud (input_);
    eifilter.setIndices (ind);
    eifilter.filter (*color_planecloud);    

  } else {
    for (int i = 0; i < planePointIndices.size(); i++) {
      std::vector<int> temp(1, i);
      planes.push_back(temp);
    }    
    ConvertPCLCloud2ColorSeg(planes, color_planecloud);
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(
    new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*color_planecloud, *color_cloud);
  cloud_viewer->showCloud(color_cloud);
}

void GraphFilter::visualization() {
  static boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  // viewer->setBackgroundColor(0, 0, 0);
  // viewer->addCoordinateSystem(0.3);
  // viewer->initCameraParameters();
  // viewer->setCameraPosition(0,0,0,0,0,1,0,-1,0);
  viewer->removeAllPointClouds();
  viewer->removeAllShapes();
  viewer->removeCoordinateSystem();
// visualize node
  std::cout << "number of surfaces: " << planePointIndices.size() << std::endl;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr color_planecloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  std::vector<std::vector<int> > planes;
  for (int i = 0; i < planePointIndices.size(); i++) {
    std::vector<int> temp(1, i);
    planes.push_back(temp);
  }

  ConvertPCLCloud2ColorSeg(planes, color_planecloud);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb33(color_planecloud);
  viewer->addPointCloud<pcl::PointXYZRGBNormal>(color_planecloud, rgb33, "cloud_front");

  // visualize edge
  std::stringstream ss;
  for (int i = 0; i < nbgh_matrix.size(); i ++) {
    for (int j = 0; j < nbgh_matrix[i].size(); j++) {
      if (!nbgh_matrix[i][j]) continue;
      ss.str("");
      ss << i <<"-"<< j;
      viewer->addLine(input_->points[planePointIndices[i][0]],
        input_->points[planePointIndices[j][0]],
        ss.str()
      );

      viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2.5, ss.str());
    }
  }

  // vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
  // vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
  // vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();

  // //Iterate through all adjacent points, and add a center point to adjacent point pair
  // for (int i = 0; i < nbgh_matrix.size(); i ++) {
  //   for (int j = 0; j < nbgh_matrix[i].size(); j++) {
  //     if (!nbgh_matrix[i][j]) continue;
 
  //     points->InsertNextPoint (input_->points[planePointIndices[i][0]].getVector3fMap());
  //     points->InsertNextPoint (input_->points[planePointIndices[j][0]].getVector3fMap());
  //  }
  // }
  // // Create a polydata to store everything in
  // vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();
  // // Add the points to the dataset
  // polyData->SetPoints (points);
  // polyLine->GetPointIds  ()->SetNumberOfIds(points->GetNumberOfPoints ());
  // for(unsigned int i = 0; i < points->GetNumberOfPoints (); i++)
  //   polyLine->GetPointIds ()->SetId (i,i);
  // cells->InsertNextCell (polyLine);
  // // Add the lines to the dataset
  // polyData->SetLines (cells);
  // viewer->addModelFromPolyData (polyData,supervoxel_name);

  viewer->spin();


}


void GraphFilter::setInputCloudAndTransform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_,
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

 boost::shared_ptr<Constraints> GraphFilter::chooseConstraints(
   std::set<boost::shared_ptr<Constraints> >& c_set) {
  std::set<boost::shared_ptr<Constraints> >::iterator it = c_set.begin();
  boost::shared_ptr<Constraints> c = *it;
  c_set.erase(it);

  return c;
}

boost::shared_ptr<Constraints> GraphFilter::chooseConstraints(
  std::vector<boost::shared_ptr<Constraints> > &c_order) {

  std::vector<boost::shared_ptr<Constraints> >::iterator it = c_order.begin();
  boost::shared_ptr<Constraints> c = *it;
  c_order.erase(it);
  return c;
}


void GraphFilter::initializeMatching() {
  sub2scene.clear();
  scene2sub.clear();

  int subTotalIds = 2;
  std::vector<int> scenelist(planePointIndices.size());
  std::vector<int> sublist(subTotalIds);

  std::iota(std::begin(sublist), std::end(sublist), 0);
  std::iota(std::begin(scenelist), std::end(scenelist), 0);

  for (int i = 0; i < subTotalIds; i++) {
    std::vector<int> ts(scenelist);
    sub2scene[i] = std::unordered_set<int>(ts.begin(), ts.end())  ;
  }

  for (int i = 0; i < planePointIndices.size(); ++i) {
    std::vector<int> ts(sublist);
    scene2sub[i] = std::set<int>(ts.begin(), ts.end() ) ;
  }

  nbgh_list.resize(planePointIndices.size());
 
}

void GraphFilter::constraintsFiltering() {
  int count = 0;
  int num_constrains = 8;
  // std::set<boost::shared_ptr<Constraints> > c_set;
  std::vector<boost::shared_ptr<Constraints> > c_set;
  Centroid centroid(input_, planePointIndices);
  Pca pcav(input_, planePointIndices);

  initializeMatching();
  // initiliaze constraints by insertion
  std::map<int, int> sz_rej0 = {{0, 1}};
  std::vector<float> dims_constraint {0.3, 0.70, 0, 0.15};
  std::vector<float> loc_finger_constraint {-1000, 1000, -1000, 1000, -0.22, 0.3};
  std::vector<float> loc_arm_constraint {-1000, 1000, -1000, 1000, -0.1, 1};

  std::map<int, int> sz_rej1 = {{1, 1}};
  std::set<int> fd0;
  fd0.insert(1);
  std::set<int> fd1;
  fd1.insert(0);
  std::map<int, std::set<int> > subDest0;
  subDest0[0] = fd0;
  subDest0[1] = fd1;
  std::map<int, int> fa_rej = {{0, 1}, {1, 1}};
  // c_set.insert(boost::shared_ptr<Constraints>(new SizeConstraints(input_,
  //   planePointIndices, dims_constraint, sz_rej0) ) );
  // c_set.insert(boost::shared_ptr<Constraints>(new LocationConstraints(input_,
  //   planePointIndices, sz_rej1) ) );
  // c_set.insert(boost::shared_ptr<Constraints>(new FingerConstraints(input_,
  //   planePointIndices, sz_rej1) ) );


  c_set.push_back(boost::shared_ptr<Constraints>(new SizeConstraints(input_,
    planePointIndices, dims_constraint, pcav, sz_rej0) ) );
  c_set.push_back(boost::shared_ptr<Constraints>(new LocationConstraints(input_,
    planePointIndices, centroid, loc_arm_constraint, sz_rej0) ) );
  c_set.push_back(boost::shared_ptr<Constraints>(new LocationConstraints(input_,
    planePointIndices, centroid, loc_finger_constraint, sz_rej1) ) );
  c_set.push_back(boost::shared_ptr<Constraints>(new FingerConstraints(input_,
    planePointIndices, pcav,sz_rej1) ) );

  boost::shared_ptr<ArmFingerConstraints> armfingerC(new ArmFingerConstraints(
        nbgh_list, scene2sub, subDest0, fa_rej, input_, planePointIndices,
        centroid, pcav
      ) );
  armfingerC->setTransform(T_Eigen);
  c_set.push_back(armfingerC);

 
  while (!c_set.empty()) {
    // choose a constraint
    boost::shared_ptr<Constraints> ct = chooseConstraints(c_set);
    // c_set.erase(ct);
    std::string cname = ct->getClassName();
    std::vector<int> v_list = ct->getVerticeList();

    // std::cout << "after get vlist" << std::endl;
    
    for (std::vector<int>::iterator it = v_list.begin(); it != v_list.end(); ++it) {
      for (std::unordered_set<int>::iterator sit = sub2scene[*it].begin();
        sit != sub2scene[*it].end();) {
        
        int sceneid = *sit, subid = *it;
        // if (cname.compare("ArmFingerConstraints") == 0) {
        // std::cout << "scene " << sceneid << " sub " << subid << std::endl;
        // // std::cout << planecoeffs[sceneid] << std::endl;
        // }
        bool pass = ct->passLabel(sceneid, subid);
        // if (cname.compare("ArmFingerConstraints") == 0) {
        //   std::cout << "after passLabel" << std::endl;
        // }

        if (!pass) {
          ++sit;
          sub2scene[subid].erase(sceneid);
          scene2sub[sceneid].erase(subid);
        } else {
          ++sit;
        }

      }
    }
    
    // std::cout << "after for loop" << std::endl;

  }
  
  // std::cout << "loop done " << std::endl;

  //find components/ results
  //start from sub id 0 to sub id 1
  int fcount = 0;
  std::vector<int> fingers;
  std::vector<int> arms;
  std::vector<Eigen::Vector3f> tips;
  std::vector<int> tips_ind;

  for (std::unordered_set<int>::iterator sit = sub2scene[0].begin();
    sit != sub2scene[0].end(); ++sit) { 
    Eigen::Vector3f tip (1,1,1);
    float max_dist = -1000000.0;
    int fingerid = -1;
    int arm_id = -1;
    int tip_index = -1;
    for (std::set<int>::iterator nit = nbgh_list[*sit].begin();
      nit != nbgh_list[*sit].end(); ++ nit) {
      if (scene2sub[*nit].find(1) == scene2sub[*nit].end()) {
        continue;
      }
      Eigen::Vector3f ftip;
      float dist;
      int ffid;
      armfingerC->getDistAndTip(*sit, *nit, ffid, dist);
      ftip = input_->points[ffid].getVector3fMap();
      // armfingerC->getDistAndTip(*sit, *nit, ftip, dist);

      if (dist > max_dist) {
        tip = ftip;
        max_dist = dist;
        fingerid = *nit;
        arm_id = *sit;
        tip_index = ffid;
      }
    }
    
    if (fingerid > 0) {
      bool replace = false;
      for (int i = 0; i < fingers.size(); i++) {
        if ( (tips[i]-tip).norm() < 0.07 ) {
           replace = true;
           if (tip[2] < tips[i][2]) {

             fingers[i] = fingerid;
             tips[i] = tip;
             tips_ind[i] = tip_index;
             arms[i] = arm_id;
           }
        }
      }
      if (!replace) {
        fingers.push_back(fingerid);
        tips.push_back(tip);
        arms.push_back(arm_id);
        tips_ind.push_back(tip_index);
      }
    }

  }

  for (int i = 0; i < fingers.size(); i++) {
      static tf::TransformBroadcaster br;
      tf::Transform microTF;
      microTF.setOrigin(tf::Vector3(tips[i][0], tips[i][1], tips[i][2]) );
      tf::Quaternion microQ;
      microQ.setRPY(0, 0, 0);
      microTF.setRotation(microQ);
      std::stringstream ss;
      ss << "/finger" << fcount++;
      std::cout << ss.str() << std::endl;
      br.sendTransform(tf::StampedTransform(microTF, ros::Time::now(), "/base", ss.str()));
      // br.sendTransform(tf::StampedTransform(microTF, frame_time_stamp, "/base", ss.str()));

     
      // boxes.push_back(generateBBox(armfingerC->getEndPoint(arms[i], fingers[i]),
      //  fingers[i]) );
      boxes.push_back(generateBBoxFinger(tips_ind[i], 
        armfingerC->getPrincipalDirOrig(arms[i]) ) );
      rays.push_back(std::pair<Eigen::Vector3f, Eigen::Vector3f>(
        input_->points[tips_ind[i] ].getVector3fMap(),
        armfingerC->getPrincipalDir(arms[i]) ) );

  }




}

std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > GraphFilter::getRays() {
  return rays;
}

boost::shared_ptr<Constraints> GraphFilter::getConstraints(int cind, 
    Centroid& centroid, Pca& pcav, const Eigen::Affine3d & T_Eigena,
    std::vector<std::set<int> > & nbgh_listc
){
  std::map<int, int> sz_rej0 = {{0, 1}};
  std::vector<float> dims_constraint {0.3, 0.70, 0, 0.15};
  std::vector<float> loc_finger_constraint {-1000, 1000, -1000, 1000, -0.22, 0.3};
  std::vector<float> loc_arm_constraint {-1000, 1000, -1000, 1000, -0.1, 1};

  std::map<int, int> sz_rej1 = {{1, 1}};
  std::set<int> fd0;
  fd0.insert(1);
  std::set<int> fd1;
  fd1.insert(0);
  std::map<int, std::set<int> > subDest0;
  subDest0[0] = fd0;
  subDest0[1] = fd1;
  std::map<int, int> fa_rej = {{0, 1}, {1, 1}};
  
  switch (cind) {
    case 0:
      return boost::shared_ptr<Constraints>(new SizeConstraints(input_,
        planePointIndices, dims_constraint, pcav, sz_rej0) );
    case 1:
      return boost::shared_ptr<Constraints>(new LocationConstraints(input_,
        planePointIndices, centroid, loc_arm_constraint, sz_rej0) );
    case 2:
      return boost::shared_ptr<Constraints>(new LocationConstraints(input_,
         planePointIndices, centroid, loc_finger_constraint, sz_rej1) );
    case 3:
      return boost::shared_ptr<Constraints>(new FingerConstraints(input_,
        planePointIndices, pcav,sz_rej1) );

    case 4:{
       boost::shared_ptr<ArmFingerConstraints> armfingerC(new ArmFingerConstraints(
          nbgh_listc, scene2sub, subDest0, fa_rej, input_, planePointIndices,
          centroid, pcav
        ) );
       armfingerC->setTransform(T_Eigen);
      return armfingerC;
           }
    default: { 
      std::cout << "!!!ERROR, WRONG CONSTRAINTS INDEX!!!" << std::endl;
      return NULL;
             }
  }
  return NULL;
}


hand_tracker_2d::HandBBox GraphFilter::generateBBoxFinger(
  int tip_ind, Eigen::Vector3f dir) {
  hand_tracker_2d::HandBBox result;
  float ax = tip_ind % input_->width;
  float ay = tip_ind / input_->width;

  Eigen::Vector3f p = orig_cloud_->points[tip_ind].getVector3fMap();
  Eigen::Vector2f dir2d ( (p(0) + dir(0)) / (p(2) + dir(2)),
    (p(1) + dir(1)) / (p(2) + dir(2)) );
  dir2d = -1 * dir2d;
  dir2d.normalize();
  Eigen::Vector2f dir2dv(1, -1 * dir2d(0) /dir2d(1));
  dir2dv.normalize();

  std::vector<float> xs;
  std::vector<float> ys;
  int width = 30, height = 80;
  xs.push_back(ax + dir2dv(0) * width);
  xs.push_back(ax - dir2dv(0) * width);
  xs.push_back(ax + dir2dv(0) * width + dir2d(0) * height);
  xs.push_back(ax - dir2dv(0) * width + dir2d(0) * height);


  ys.push_back(ay + dir2dv(1) * width);
  ys.push_back(ay - dir2dv(1) * width);
  ys.push_back(ay + dir2dv(1) * width + dir2d(1) * height);
  ys.push_back(ay - dir2dv(1) * width + dir2d(1) * height);


  result.stamp = frame_time_stamp;
  result.box.resize(4);
  result.box[0] = Graph::Util::getVectorsMin(xs);
  result.box[1] = Graph::Util::getVectorsMin(ys);
  result.box[2] = Graph::Util::getVectorsMax(xs);
  result.box[3] = Graph::Util::getVectorsMax(ys);

  return result;
}


hand_tracker_2d::HandBBox GraphFilter::generateBBox(int arm_end_ind, int fid) {
  hand_tracker_2d::HandBBox result;
  int ax = arm_end_ind % input_->width;
  int ay = arm_end_ind / input_->width;


  int xmin = 20000, ymin = 20000;
  int xmax = -1, ymax = -1;

  for (int i = 0; i < planePointIndices[fid].size(); i++) {
    int index = planePointIndices[fid][i];
    int x = index % input_->width;
    int y = index / input_->width;
    if (x < xmin) {
      xmin = x;
    } else if (x > xmax) {
      xmax = x;
    }

    if (y < ymin) {
      ymin = y;
    } else if (y > ymax){
      ymax = y;
    }
  }
  
  std::cout << "[ " << xmin << " , " << xmax << " ] , ["  << ymin 
    << " , " << ymax << " ]" << std::endl;
  int cx = (ax + (xmin + xmax) / 2) / 2;
  int cy = (ay + (ymin + ymax) / 2) / 2;
  result.stamp = frame_time_stamp;
  result.box.resize(4);
  result.box[0] = cx - 30;
  result.box[1] = cy - 40;
  result.box[2] = cx + 30;
  result.box[3] = cy + 40;

  // result.box[0] = xmin - 10;
  // result.box[1] = ymin;
  // result.box[2] = xmax + 10;
  // result.box[3] = ymax;

  return result;
}


std::vector<hand_tracker_2d::HandBBox> GraphFilter::getBBoxes() {
  return boxes;
}


void GraphFilter::setTimeStamp(ros::Time stamp_) {
  frame_time_stamp = stamp_;
}

void GraphFilter::OptimizeConstraintOrder(std::vector<boost::shared_ptr<Constraints> >&
          c_order ) {
  std::vector<boost::shared_ptr<Constraints> > orig_order(c_order);
  c_order.clear();
  while (orig_order.size() != 0) {
    int indM = 0;
    float reward_cost = -1;
    for (int i = 0; i < orig_order.size(); i++) {
      float reward, cost; 
      //getTrainReward(orig_order[i], reward, cost);
      if (reward / cost > reward_cost) {
        reward_cost = reward / cost;
        indM = i;
      }
    
    }
    c_order.push_back(orig_order[indM]);
    orig_order.erase(orig_order.begin() + indM);
  }
}

