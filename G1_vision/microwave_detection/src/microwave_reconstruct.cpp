#include <microwave_detection/microwave_reconstruct.h>
// #include <namaris/utilities/pcl_typedefs.hpp>
// #include <namaris/algorithms/region_growing_normal_variation/region_growing_normal_variation.hpp>
#include <sisyphus/box_detection.hpp>


MicrowaveRect::MicrowaveRect() {
   initialize();
}
MicrowaveRect::~MicrowaveRect() {

}

void MicrowaveRect::initialize() {
  use_box = false;
  handleIndices.clear();
}
void MicrowaveRect::findMicrowave() {
  if (!use_box) {
    segmentPlane();
    computeRelation();
  } else {
    findMicrowave_box();
  }
}

void MicrowaveRect::setUseBoxDetect(bool use_box_) {
  use_box = use_box_;
}

bool MicrowaveRect::generateMessage(world_model_msgs::UpdateStatesObjects & srv) {
  if (handleIndices.size() == 0) {
    std::cout << "Hanlde not detected. Error!" << std::endl;
    return false;
  }
  srv.request.objects_info.resize(3);

  world_model_msgs::Object microwave_handle;
  microwave_handle.id = "microwave_handle";
  microwave_handle.primitives.push_back(setSolidPrimitiveBox(handle_dimension));
  microwave_handle.primitive_poses.push_back(setGeometryMsgs(handle_centroid,
    microwave_pose));
  srv.request.objects_info[0] = microwave_handle;

  world_model_msgs::Object microwave_door;
  microwave_door.id = "microwave_door";
  microwave_door.primitives.push_back(setSolidPrimitiveBox(door_dimension));
  microwave_door.primitive_poses.push_back(setGeometryMsgs(door_centroid,
    microwave_pose));
  // microwave_door.primitives.push_back(setSolidPrimitiveBox(door_dimension));
  // microwave_door.primitive_poses.push_back(setGeometryMsgs(door_open_centroid,
  //   door_open_pose));
  srv.request.objects_info[1] = microwave_door;

  world_model_msgs::Object microwave_frame;
  microwave_frame.id = "microwave_frame";
  
  // top, bottom, right, left 
  microwave_frame.primitives.resize(4);
  microwave_frame.primitive_poses.resize(4);
  microwave_frame.primitives[0] = setSolidPrimitiveBox(top_dimension);
  microwave_frame.primitive_poses[0] =setGeometryMsgs(top_centroid, microwave_pose);
  microwave_frame.primitives[1] = setSolidPrimitiveBox(bottom_dimension);
  microwave_frame.primitive_poses[1] =setGeometryMsgs(bottom_centroid, microwave_pose);
  microwave_frame.primitives[2] = setSolidPrimitiveBox(right_side_dimension);
  microwave_frame.primitive_poses[2] =setGeometryMsgs(right_side_centroid, microwave_pose);
  microwave_frame.primitives[3] = setSolidPrimitiveBox(left_side_dimension);
  microwave_frame.primitive_poses[3] =setGeometryMsgs(left_side_centroid, microwave_pose);   
  srv.request.objects_info[2] = microwave_frame;
  return true;

}

shape_msgs::SolidPrimitive 
  MicrowaveRect::setSolidPrimitiveBox(Eigen::Vector3f dims) {
  shape_msgs::SolidPrimitive sp;
  sp.type = sp.BOX;
  sp.dimensions.push_back(dims[0]);
  sp.dimensions.push_back(dims[1]);
  sp.dimensions.push_back(dims[2]);
  return sp;
}

geometry_msgs::Pose MicrowaveRect::setGeometryMsgs(Eigen::Vector3f c,
  Eigen::Quaternionf q) {
  geometry_msgs::Pose pose;
  pose.position.x = c[0];
  pose.position.y = c[1];
  pose.position.z = c[2];
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

void MicrowaveRect::computeCentroid(std::map<int,  pcl::PointXYZ>& centroid) {
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

void MicrowaveRect::computePatchNorm(std::map<int, pcl::Normal>& PatchMeanN) {
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

void MicrowaveRect::setPointPlaneLabel() {
  for (int i = 0; i < planePointIndices.size(); i++) {
   	for (int j = 0; j < planePointIndices[i].size(); j++) {
       // pointPlaneLabel.insert(std::pair<int, int>(planePointIndices[i][j], i));
       pointPlaneLabel[planePointIndices[i][j]] = i;
   	}
  }
}

bool MicrowaveRect::tablePlaneUnaryConditionFunction(const PointNC& p, const Eigen::Vector3f &gravity_vector, const float gravity_angle_thresh) {
  Eigen::Vector3f normal = p.getNormalVector3fMap();
  // return (normal.dot(-gravity_vector) > std::cos(gravity_angle_thresh)) || (std::abs(normal.dot(gravity_vector)) < std::sin(gravity_angle_thresh));
  return pcl::isFinite(p);
}

bool MicrowaveRect::tablePlaneBinaryConditionFunction(const PointNC& p0, const PointNC& p1, float dist) {
  Eigen::Vector3f n0 = p0.getNormalVector3fMap();
  Eigen::Vector3f n1 = p1.getNormalVector3fMap();
  if (n0.dot(n1) > 0.94 || dist < 0.02) {
  	return true;
  }
}


void MicrowaveRect::setSearcher() {
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

void MicrowaveRect::printNeighbors() {
  for(unsigned i=0; i<nbgh_matrix.size(); i++) {
	for (unsigned j = 0; j < nbgh_matrix[i].size(); j++) {
		std::cout << (nbgh_matrix)[i][j] << "\t";
    }
   std::cout << std::endl;
  }	
}


void MicrowaveRect::locateBox(std::vector<int> & surfaceState, FitSize& fs,
	std::map<int, pcl::Normal>& PatchMeanN) {
  component.clear();	
  for  (unsigned i = 0; i < planePointIndices.size(); i++) {
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
        if (relations[ele][j] || relations[j][ele]) {
          que.push(j);
          surfaceState[j] = 2;
        }
      }
    }
    component.push_back(v); 

  }
  std::cout << "component before valid" << component.size() << std::endl;

  std::vector<int> validComponent;
  int frontSurfaceId = -1;
  double frontSurfaceArea = 0;
  for (unsigned i = 0; i < component.size(); i++) {
     bool hasHorizonal = false, hasVertical = false;
    if (fs.fitCubeSize(component[i])) {
      for (unsigned j= 0; j < component[i].size(); j++) {        
      std::cout << "z " << PatchMeanN[component[i][j]] << std::endl;
        if (PatchMeanN[component[i][j]].normal_z   > 0.97) {
          hasHorizonal = true;
        } else if (fabs(PatchMeanN[component[i][j]].normal_z) < 0.13){
          hasVertical = true;
        }
      }
    }
   if (hasHorizonal && hasVertical) {
    validComponent.push_back(i);
   }
  }
  
  std::vector<std::vector<int> > orig_component = component;
  component.clear();
  component.resize(validComponent.size());

  for (int i = 0; i < validComponent.size(); i++) {
    component[i] = (orig_component[validComponent[i]]);
  }
  std::cout << "component " << component.size() << std::endl;
}


void MicrowaveRect::FindHandleBox(FitSize & fs, std::map<int, pcl::Normal>& PatchMeanN) {
  for (int i = 0; i < component.size(); i++) {
    // front door and other surfaces is assigned in locateFrontDoor
    locateFrontDoor(fs, i, PatchMeanN);
    findHandle(fs);
    if (handleIndices.size() > 0) {
      component_box_id = i;
      return ;
    }
  }
}

void MicrowaveRect::locateFrontDoor(FitSize & fs, int box_id,
  std::map<int, pcl::Normal>& PatchMeanN) {
  int frontSurfaceId = -1;
  int topSurfaceId = -1;

  double frontSurfaceArea = 0;
  double topSurfaceArea = 0;
  std::vector<std::vector<int> > faces;
  std::cout << "overall size " << component[box_id].size() << std::endl;
    for (unsigned j= 0; j < component[box_id].size(); j++) {

      if (fabs(PatchMeanN[component[box_id][j]].normal_z) < 0.08 ||
      	fabs(PatchMeanN[component[box_id][j]].normal_z) > 0.97) {
        bool integrate = false;
        pcl::Normal n = PatchMeanN[component[box_id][j]];

        for (int k = 0; k < faces.size(); k++) {
          pcl::Normal n0 = PatchMeanN[faces[k][0]]; 
          if (n.normal_x * n0.normal_x + n.normal_y * n0.normal_y +
            n.normal_z * n0.normal_z > 0.95) {
    		     faces[k].push_back(component[box_id][j]);
    		     integrate = true;
    		     break;
          }
        }
        if (!integrate) {
          std::vector<int> temp;
      	  temp.push_back(component[box_id][j]);
          faces.push_back(temp);
        }

      }
    }
  std::cout << "after combining planes" << std::endl;
  for (int i = 0; i < faces.size(); i++) {
    std::vector<float> box3d = fs.get3DBox(faces[i]);
    if (fabs(PatchMeanN[faces[i][0]].normal_z) < 0.08) {
      double height = box3d[5]-box3d[4];
      double width = std::sqrt((box3d[1]-box3d[0])*(box3d[1]-box3d[0]) + (box3d[3]-box3d[2])*(box3d[3]-box3d[2]));
      std::cout << " face size " << faces[i].size() 
      << " front area " << height << " " << width << " " << height*width << std::endl;
      if (height * width > frontSurfaceArea) {
        frontSurfaceId = i;
        frontSurfaceArea = height * width;
      }  	
    } else {
      double height = box3d[1]-box3d[0];
      double width = box3d[3]-box3d[2];
      if (height * width > topSurfaceArea) {
        topSurfaceId = i;
        topSurfaceArea = height * width;
      }  	
    }

  }
  std::cout<< frontSurfaceId << " after faces " << topSurfaceId<< std::endl;
  if (frontSurfaceId >= 0 && topSurfaceId >= 0) {
    frontDoor = faces[frontSurfaceId];
    topFace = faces[topSurfaceId];
    frontNormal = PatchMeanN[faces[frontSurfaceId][0]].getNormalVector3fMap();
    topNormal = PatchMeanN[faces[topSurfaceId][0]].getNormalVector3fMap();
    sideNormal = topNormal.cross(frontNormal);
  }
}


void MicrowaveRect::fitCuboid() {
  if (frontDoor.size() == 0 || topFace.size() == 0) {
    std::cout << "No front door or top face found!" << std::endl;
    return;
  }

  std::vector<int> allFaces;
  allFaces.insert(allFaces.end(), frontDoor.begin(), frontDoor.end());
  allFaces.insert(allFaces.end(), topFace.begin(), topFace.end());
  //Get the plane equations for all six faces
  pcl::PointXYZRGBNormal right_p, left_p, top_p, bottom_p, front_p, back_p;
  double left_dist = std::numeric_limits<double>::max(), 
    right_dist = std::numeric_limits<double>::lowest(),
    bottom_dist = std::numeric_limits<double>::max(), 
    top_dist = std::numeric_limits<double>::lowest(),
    back_dist = std::numeric_limits<double>::max(), 
    front_dist = std::numeric_limits<double>::lowest();

  for (int i = 0; i < allFaces.size(); i++) {
    for (int j = 0; j < planePointIndices[allFaces[i]].size(); j++) {
      double dist_right = pcl::pointToPlaneDistanceSigned(input_->points[planePointIndices[allFaces[i]][j]],
        sideNormal[0], sideNormal[1], sideNormal[2], 0);

      double dist_top = pcl::pointToPlaneDistanceSigned(input_->points[planePointIndices[allFaces[i]][j]],
        topNormal[0], topNormal[1], topNormal[2], 0);

      double dist_front = pcl::pointToPlaneDistanceSigned(input_->points[planePointIndices[allFaces[i]][j]],
        frontNormal[0], frontNormal[1], frontNormal[2], 0);


      if (dist_right < left_dist) {
         left_dist = dist_right;
         left_p = input_->points[planePointIndices[allFaces[i]][j]];
      } else if (dist_right > right_dist) {
         right_dist = dist_right;
         right_p = input_->points[planePointIndices[allFaces[i]][j]];
      }

      if (dist_top < bottom_dist) {
         bottom_dist = dist_top;
         bottom_p = input_->points[planePointIndices[allFaces[i]][j]];
      } else if (dist_top > top_dist) {
         top_dist = dist_top;
         top_p = input_->points[planePointIndices[allFaces[i]][j]];
      }

      if (dist_front < back_dist) {
         back_dist = dist_front;
         back_p = input_->points[planePointIndices[allFaces[i]][j]];
      } 
      // else if (dist_front > front_dist) {
      //    front_dist = dist_front;
      //    front_p = input_->points[planePointIndices[allFaces[i]][j]];
      // }

    }
  }

  Eigen::Vector3f front_mean(0,0,0);
  float points_count = 0.0;
  for (int i = 0; i < frontDoor.size(); i++) {
    points_count += planePointIndices[frontDoor[i] ].size();
    for (int j = 0; j < planePointIndices[frontDoor[i]].size(); j++) {
       front_mean = front_mean + input_->points[planePointIndices[frontDoor[i]][j] ].getVector3fMap();
    }
  }
 front_mean = front_mean / points_count;

  float distv = top_dist - bottom_dist;
  if (distv < 0.3) {
    bottom_p = top_p;
    Eigen::Vector3f p = top_p.getVector3fMap() - 0.3 * topNormal;
    bottom_p.x = p[0];
    bottom_p.y = p[1];
    bottom_p.z = p[2];

  }
  std::cout << "vertical " << distv << std::endl;


  frontPlane = Eigen::Vector4f (frontNormal[0], frontNormal[1], frontNormal[2],
    -1*frontNormal.dot(front_mean ));

  backPlane = Eigen::Vector4f (frontNormal[0], frontNormal[1], frontNormal[2],
    -1*frontNormal.dot(back_p.getVector3fMap() ));

  topPlane=Eigen::Vector4f(topNormal[0], topNormal[1], topNormal[2],
    -1*topNormal.dot(top_p.getVector3fMap() ));

  bottomPlane = Eigen::Vector4f(topNormal[0], topNormal[1], topNormal[2],
    -1*topNormal.dot(bottom_p.getVector3fMap() ));

  rightPlane = Eigen::Vector4f(sideNormal[0], sideNormal[1], sideNormal[2],
    -1*sideNormal.dot(right_p.getVector3fMap() ));

  leftPlane=Eigen::Vector4f(sideNormal[0], sideNormal[1], sideNormal[2],
    -1*sideNormal.dot(left_p.getVector3fMap() ));

  pcl::threePlanesIntersection(frontPlane, topPlane, rightPlane, front_top_right);

  pcl::threePlanesIntersection(backPlane, topPlane, rightPlane, back_top_right);

  pcl::threePlanesIntersection(frontPlane, bottomPlane, rightPlane, front_bottom_right);

  pcl::threePlanesIntersection(frontPlane, topPlane, leftPlane, front_top_left);

}



void MicrowaveRect::findHandle(FitSize& fs) {
  if (frontDoor.size() == 0 || topFace.size() == 0) {
    std::cout << "No front door or top face found!" << std::endl;
    handleIndices.clear();
    return;
  }
  // figure out constraint surface coefficients
  Eigen::Vector3f f_p = (input_->points[planePointIndices[frontDoor[0] ][0]]).getVector3fMap();
  double f_d = -1 * (f_p.dot(frontNormal));
  Eigen::Vector3f t_p = (input_->points[planePointIndices[topFace[0] ][0]]).getVector3fMap();
  double t_d = -1 * (t_p.dot(topNormal));

  // get the distance of microwave under each direction
  // top_dist is the largest distance from top face. It is a negative value
  double top_dist = 1;
  for (int i = 0; i < frontDoor.size(); i++) {
    for (int j = 0; j < planePointIndices[frontDoor[i]].size(); j++) {
      double dist_p = pcl::pointToPlaneDistanceSigned(input_->points[planePointIndices[frontDoor[i]][j]],
        topNormal[0], topNormal[1], topNormal[2], t_d);
      if (dist_p < top_dist) {
       	 top_dist = dist_p;
      } 
    }
  }

  pcl::PointXYZRGBNormal r_p;
  double left_dist = std::numeric_limits<double>::max(), 
    right_dist = std::numeric_limits<double>::lowest();
  for (int i = 0; i < topFace.size(); i++) {
    for (int j = 0; j < planePointIndices[topFace[i]].size(); j++) {
      double dist_p = pcl::pointToPlaneDistanceSigned(input_->points[planePointIndices[topFace[i]][j]],
        sideNormal[0], sideNormal[1], sideNormal[2], 0);
      if (dist_p < left_dist) {
       	 left_dist = dist_p;
      }
      if (dist_p > right_dist) {
       	 right_dist = dist_p;
       	 r_p = input_->points[planePointIndices[topFace[i]][j]];
      }
    }
  }
  double s_d = -1 * r_p.getVector3fMap().dot(sideNormal);
  double side_dist = left_dist - right_dist;

  handleIndices.clear();
  for (int i = 0; i < input_->points.size(); i++) {
    double dist = pcl::pointToPlaneDistanceSigned(input_->points[i],frontNormal[0], frontNormal[1],
    	frontNormal[2], f_d);
    if (dist < 0.015 || dist > 0.1) {
      continue;
    }
    dist = pcl::pointToPlaneDistanceSigned(input_->points[i],topNormal[0], topNormal[1],
    	topNormal[2], t_d);
    if (dist > 0 || dist < top_dist) {
    	continue;
    }

    dist = pcl::pointToPlaneDistanceSigned(input_->points[i],sideNormal[0], sideNormal[1],
    	sideNormal[2], s_d);
    if (dist > 0 || dist < side_dist) {
    	continue;
    }
    handleIndices.push_back(i);
  }
  
  boost::shared_ptr<std::vector<int> > ind(new std::vector<int>(handleIndices));
  int valid_count = 0;
  for (int i = 0; i < handleIndices.size(); i++) {
    if (pcl::isFinite(input_->points[handleIndices[i]])) {
      valid_count++;
    }
  }

  std::cout << "valid count" << valid_count << std::endl;
  if (valid_count == 0) {
    std::cout << "No points in the hull" << std::endl;
    handleIndices.clear();
    return ;
  }
  

  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBNormal> ec;
  ec.setInputCloud(input_);
  ec.setIndices(ind);
  ec.setClusterTolerance(0.01);
  ec.setMinClusterSize(200);
  ec.setMaxClusterSize(25000);
  
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(
  	new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  tree->setInputCloud(input_, ind);

  ec.setSearchMethod(tree);

  std::vector<pcl::PointIndices> cluster_ind;
  ec.extract(cluster_ind);
  int count = 0;

  std::cout <<"cluster number" << cluster_ind.size() << std::endl;
  for (int i = 0; i < cluster_ind.size(); i++) {
  	if (fs.isHandleShape(cluster_ind[i].indices)) {
       handleIndices = cluster_ind[i].indices;
       count++;
  	}
  }
  if (count == 0) {
    handleIndices.clear();
  }
  std::cout << count << " positives" << std::endl;
}

bool MicrowaveRect::convexBox(const pcl::PointXYZ& c0, const pcl::PointXYZ& c1,
        const pcl::Normal& n0, const pcl::Normal& n1) {
  Eigen::Vector3f p0(c0.x, c0.y, c0.z);
  Eigen::Vector3f p1(c1.x, c1.y, c1.z);

  Eigen::Vector3f d0(n0.normal_x, n0.normal_y, n0.normal_z);
  Eigen::Vector3f d1(n1.normal_x, n1.normal_y, n1.normal_z);
  
  // std::cout << "convexBox " << p0 << "\n " << p1 << " \n" << p0 - p1 << " endconvex" << std::endl;

  float orig = (p0 - p1).norm();
  for (int i = 0; i < 4; i++) {
  	float alpha = 0.03 + alpha * i;
    float dist0 = (p0 - p1 - alpha * d1).norm();
    if (dist0 < orig) {
    	return false;
    }
    float dist1 = (p0 + alpha * d0 - p1).norm();
    if (dist1 < orig) {
    	return false;
    }

  }

  return true;

}



void MicrowaveRect::setTransform(const Eigen::Affine3d & T_Eigen) {
  this->T_Eigen = T_Eigen;
}


void MicrowaveRect::setInputCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_) {
  if (cloud_->points.size() == 0) {
    std::cout << "Error empty point cloud" << std::endl;
    return ;
  }
  std::vector<int> index;
  this->input_.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::removeNaNFromPointCloud(*cloud_, *this->input_, index);
}


void MicrowaveRect::setInputAndOrigCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_,
         pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr orig_cloud_){
    if (cloud_->points.size() == 0 || orig_cloud_->points.size() == 0) {
      std::cout << "Error empty point cloud" << std::endl;
      return ;
    }
    this->input_ = cloud_;
    this->orig_cloud_ = orig_cloud_;
}

void MicrowaveRect::computeRelation() {
  FitSize fs;
  fs.setInputCloud(input_);
  fs.setSurfaceIndices(planePointIndices);

  std::map<int, pcl::Normal> PatchMeanN;
  std::vector<int> surfaceState(planePointIndices.size(), 0);

  std::map<int, pcl::PointXYZ> centroid;

  std::map<int, int> sizeFit;

  
#pragma omp parallel sections
  {

    #pragma omp section
    {
      computeNeighbors();
    }
   
    #pragma omp section 
    {
      fs.compute(sizeFit);
    } 
    #pragma omp section 
    {

      computeCentroid(centroid);
    }      


    #pragma omp section 
    {
     computePatchNorm(PatchMeanN);
    }

  } // end parallel sections
  
  // printNeighbors();
  relations.clear();
  relations.resize(planePointIndices.size());
  
  for(unsigned i=0; i<planePointIndices.size(); i++) {
  	std::vector<bool> temp(planePointIndices.size(), false);
    relations[i] = temp;
  }

#pragma omp parallel for
  for(int i=0; i< planePointIndices.size(); i++) {
    for(int j=i+1; j<planePointIndices.size(); j++) {
      int p0 = i;
      int p1 = j;
      
      if (!nbgh_matrix[i][j]) {
      	continue;
      }
      if (!sizeFit[p0] || !sizeFit[p1]) {
        continue;
      }
      // std::cout << "pass size" << p0 << " " << p1 << std::endl;
      pcl::Normal n0 = PatchMeanN[p0];
      pcl::Normal n1 = PatchMeanN[p1];
      double angle = acos(n0.normal_x * n1.normal_x + n0.normal_y * n1.normal_y + n0.normal_z * n1.normal_z);
      double innpro = fabs(n0.normal_x * n1.normal_x + n0.normal_y * n1.normal_y + n0.normal_z * n1.normal_z);
      if (innpro >= 0.15) { continue; }
      std::cout << angle << "  " << n0 << " " << n1 <<" " << std::endl;
      
      pcl::PointXYZ c0 = centroid[p0];
      pcl::PointXYZ c1 = centroid[p1];

      if (c0.z < -0.4 || c1.z < -0.4 || c0.z > 0.4 || c1.z > 0.4) {
        continue;
      }

      if (!convexBox(c0, c1, n0, n1)) {
        continue;
      }

      surfaceState[p0] = 1;
      surfaceState[p1] = 1;
      
      relations[p0][p1] = true;
      relations[p1][p0] = true;
      std::cout << p0 << "  " << p1 << std::endl;
      
    }
  }

std::cout << "before microwave " << std::endl;
  locateBox(surfaceState, fs, PatchMeanN);
// std::cout << "before frontdoor " << std::endl;
//   locateFrontDoor(fs, PatchMeanN);
// std::cout << "before handle " << std::endl;

//   findHandle(fs);
  if (component.size() > 0) {
    FindHandleBox(fs, PatchMeanN);
    fitCuboid();
    microwaveParts();
  }
}

void MicrowaveRect::microwaveParts() {
  if (handleIndices.size() == 0) {
    return;
  }
  //pickup a point in the handle and get its side plane
  handle_centroid = Eigen::Vector3f (0,0,0);
  for (int i = 0; i < handleIndices.size(); i++) {
    handle_centroid += input_->points[handleIndices[i]].getVector3fMap();
    // float h_iter = pcl::pointToPlaneDistanceSigned(input_->points[handleIndices[i]],
    //   frontPlane[0], frontPlane[1], frontPlane[2], frontPlane[3]);
    // if (h_iter > handle_height) {
    //   handle_height = h_iter;
    // }
  }

  handle_centroid /= handleIndices.size();
  pcl::PointXYZ hc(handle_centroid[0],handle_centroid[1],handle_centroid[2]);
  float handle_height = pcl::pointToPlaneDistanceSigned(hc,
        frontPlane[0], frontPlane[1], frontPlane[2], frontPlane[3]);


  handleSidePlane = Eigen::Vector4f(sideNormal[0], sideNormal[1], sideNormal[2],
    -1 * (sideNormal.dot(handle_centroid )) );
  std::cout << "handle_height" << handle_height << std::endl;
  if (handle_height <= 0) {
    std::cout << "Error: handle height cannot be negative: " << handle_height << std::endl;
    return ;
  }

  pcl::threePlanesIntersection(frontPlane, topPlane, handleSidePlane, handle_top);
  pcl::threePlanesIntersection(frontPlane, bottomPlane, handleSidePlane, handle_bottom);

  // pose of microwave
  Eigen::Vector3f x = frontNormal;
  x[2] = 0;
  x.normalize();
  Eigen::Vector3f z(0, 0, 1);
  Eigen::Vector3f y = z.cross(x);
  Eigen::Matrix3f rotation; 
  std::cout << x << "\n\n" << y << std::endl;
  rotation << x , y , z;
  rotation.col(0) = x;
  rotation.col(1) = y;
  rotation.col(2) = z;
  std::cout << "rotation " << rotation << std::endl;
  // rotation = rotation.transpose();

  // std::cout << "rotation matrix " << rotation << std::endl;
  microwave_pose = Eigen::Quaternionf(rotation);
  // std::cout << microwave_pose << std::endl;
  // x (width) y (depth) z (height) of microwave
  float mx = (front_top_right - back_top_right).norm();
  float my = (front_top_right - front_top_left).norm();
  float mz = (front_top_right - front_bottom_right).norm();

  float top_thickness = 0.04, bottom_thickness = 0.065,
    left_thickness = 0.055, door_thickness = 0.035, handle_width = 0.025;

  //centroid of right part
  right_side_centroid = (handle_top + front_bottom_right - front_top_right
    + back_top_right - front_top_right + front_top_right) / 2 ;
  right_side_dimension = Eigen::Vector3f(mx, 
    (front_top_right - handle_top).norm(), mz);
 
  top_centroid = (front_top_left + back_top_right - top_thickness*topNormal) / 2;
  top_dimension = Eigen::Vector3f(mx, my, top_thickness);
 

  bottom_centroid = (front_top_left + back_top_right + 
    bottom_thickness*topNormal) / 2 + front_bottom_right - front_top_right;
  bottom_dimension = Eigen::Vector3f(mx, my, bottom_thickness);


  bottom_centroid = (front_top_left + back_top_right + 
    bottom_thickness*topNormal) / 2 + front_bottom_right - front_top_right;
  bottom_dimension = Eigen::Vector3f(mx, my, bottom_thickness);


  left_side_centroid = (front_top_left + back_top_right - front_top_right 
    + front_bottom_right - front_top_right + front_top_left
    + left_thickness*sideNormal) / 2;
  left_side_dimension = Eigen::Vector3f(mx, left_thickness, mz);

  door_centroid = (front_top_left + front_bottom_right - front_top_right 
     + handle_top - door_thickness*frontNormal ) / 2;
  door_dimension = Eigen::Vector3f(door_thickness, 
    (handle_top - front_top_left).norm(), mz);

  handle_centroid = (handle_top + handle_bottom + handle_height * topNormal) / 2;
  handle_dimension = Eigen::Vector3f(handle_height, handle_width,(handle_top-handle_bottom).norm());
  
  door_open_centroid = (front_top_left + front_bottom_right - front_top_right +
    front_top_left) / 2 + frontNormal * (handle_top - front_top_left).norm() / 2;

  Eigen::Matrix3f rotation_open;
  rotation_open.col(0) = sideNormal;
  rotation_open.col(1) = -1 * frontNormal;
  rotation_open.col(2) = topNormal;
  door_open_pose = Eigen::Quaternionf(rotation_open);

}



void MicrowaveRect::findConnectedComponent() {
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

void MicrowaveRect::computeNeighborsOrganized() {
  
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

void MicrowaveRect::computeNeighborsUnOrganized() {
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
void MicrowaveRect::computeNeighbors() {
  if (input_->isOrganized()) {
  	computeNeighborsOrganized();
  } else {
  	computeNeighborsUnOrganized();
  }
}

void MicrowaveRect::segmentPlaneUnOrganized() {
	Eigen::Vector3f gravityVector = Eigen::Vector3f(0, 0, 1).normalized();
	// Parameters
	float gravityVectorAngleThreshold = pcl::deg2rad(30.0);         // Maximum allowed difference between a table plane point normal and gravity vector
	float normalVariationThreshold    = pcl::deg2rad(10.0)*100;   // Maximum allowed normal difference between adjacent points (in rad/m)
	int minTablePlanePoints = 800;

	// Create unary condition function
	boost::function<bool (const PointNC&)> tablePlaneUnaryFunction =
	boost::bind(&MicrowaveRect::tablePlaneUnaryConditionFunction, this, _1, gravityVector, gravityVectorAngleThreshold);


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
void MicrowaveRect::segmentPlaneOrganized() {
	FindPlanes::Parameter param;
    param.adaptive = true;
    param.thrAngle = 0.3;
    int detail = 2;
    if(detail == 1) {
      param.epsilon_c = 0.58;
      param.omega_c = -0.002;
    } else if (detail == 2) {
      param.epsilon_c = 0.62;
      param.omega_c = 0.0;
    } 
    param.minPoints = 200;
  	FindPlanes fp (param);
    fp.setInputCloud (orig_cloud_);
    fp.setOrigCloud(orig_cloud_);
    fp.setPixelCheck (true, 5);
    planePointIndices = fp.compute ();
    planecoeffs = fp.planecoeffs;

    transformSurfaceNormals();

}
void MicrowaveRect::segmentPlane() {
  if (input_->isOrganized()) {
  	segmentPlaneOrganized();
  } else {
  	segmentPlaneUnOrganized();
  }

}

void MicrowaveRect::transformSurfaceNormals() {
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

void MicrowaveRect::ConvertPCLCloud2ColorSeg(const std::vector<int>& indices,
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


void MicrowaveRect::ConvertPCLCloud2ColorSeg(const std::vector<std::vector<int> >& component,
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


void MicrowaveRect::findMicrowave_box() {
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_in;
  cloud_in = coordinateRangeClipPointCloud<pcl::PointXYZRGBNormal>(input_, true, -2, 2, -2, 2, 0.001, 1.8);
  cloud_in = downSamplePointCloud<pcl::PointXYZRGBNormal>(cloud_in, 0.005);


  // std::vector<Box<pcl::PointXYZRGBNormal> > boxes = detectBoxes<pcl::PointXYZRGBNormal>(cloud_in, 1000, 0.02, 0.01, false);
  // std::cout << "number of boxes found: " << boxes.size() << std::endl;
  // for (int i = 0; i < boxes.size(); ++i) {
  //   // Eigen::Affine3f aff;
  //   // aff.matrix() = boxes[i].pose;
  //   frontNormal = boxes[i].pose.block<3,1>(0,0);
  //   sideNormal = boxes[i].pose.block<3,1>(0,1);
  //   topNormal = boxes[i].pose.block<3,1>(0,2);

  //   Eigen::Vector3f box_centroid = boxes[i].pose.block<3,1>(0,3);
  //   Eigen::Vector3f front_p = box_centroid + boxes[i].size(0)/2*frontNormal;
  //   Eigen::Vector3f back_p = box_centroid - boxes[i].size(0)/2*frontNormal;
  //   Eigen::Vector3f top_p = box_centroid + boxes[i].size(2)/2*topNormal;
  //   Eigen::Vector3f bottom_p = box_centroid - boxes[i].size(2)/2*topNormal;
  //   Eigen::Vector3f right_p = box_centroid + boxes[i].size(1)/2*sideNormal;
  //   Eigen::Vector3f left_p = box_centroid - boxes[i].size(1)/2*sideNormal;

  //   frontPlane = Eigen::Vector4f (frontNormal[0], frontNormal[1], frontNormal[2],
  //     -1*frontNormal.dot(front_p ));

  //   backPlane = Eigen::Vector4f (frontNormal[0], frontNormal[1], frontNormal[2],
  //     -1*frontNormal.dot(back_p));

  //   topPlane=Eigen::Vector4f(topNormal[0], topNormal[1], topNormal[2],
  //     -1*topNormal.dot(top_p));

  //   bottomPlane = Eigen::Vector4f(topNormal[0], topNormal[1], topNormal[2],
  //     -1*topNormal.dot(bottom_p ));

  //   rightPlane = Eigen::Vector4f(sideNormal[0], sideNormal[1], sideNormal[2],
  //     -1*sideNormal.dot(right_p));

  //   leftPlane=Eigen::Vector4f(sideNormal[0], sideNormal[1], sideNormal[2],
  //     -1*sideNormal.dot(left_p));

  //   pcl::threePlanesIntersection(frontPlane, topPlane, rightPlane, front_top_right);

  //   pcl::threePlanesIntersection(backPlane, topPlane, rightPlane, back_top_right);

  //   pcl::threePlanesIntersection(frontPlane, bottomPlane, rightPlane, front_bottom_right);

  //   pcl::threePlanesIntersection(frontPlane, topPlane, leftPlane, front_top_left);
  
  //   component_box_id = i;
  // }  
}

void MicrowaveRect::visualization() {
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(0.3);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0,0,0,0,0,1,0,-1,0);
  viewer->removeAllPointClouds();
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb2(input_);
  viewer->addPointCloud<pcl::PointXYZRGBNormal>(input_, rgb2, "cloud");
  // viewer->addPointCloudNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> (input_, input_, 100, 0.05, "normals");
  viewer->spin();

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr color_planecloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  // color_planecloud = input_;
  std::vector<std::vector<int> > planes;
  for (int i = 0; i < planePointIndices.size(); i++) {
    std::vector<int> temp(1, i);
    planes.push_back(temp);
  }
  // planes.push_back(frontDoor);
    // std::cout << "frontDoor size" << frontDoor.size() << std::endl;
  // planes.push_back(topFace);

  ConvertPCLCloud2ColorSeg(planes, color_planecloud);

   

   //  ConvertPCLCloud2ColorSeg(handleIndices, color_planecloud);
   //  viewer->removeAllPointClouds();
   //  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb23(color_planecloud);
   // viewer->addPointCloud<pcl::PointXYZRGBNormal>(color_planecloud, rgb23, "cloud");
  // viewer->spin();

  //   planes.clear();
  //   planes.push_back(frontDoor);
  //   std::cout << "frontDoor size" << frontDoor.size() << std::endl;
  //   // planes.push_back(topFace);

  //   ConvertPCLCloud2ColorSeg(planes, color_planecloud);   

  //   // ConvertPCLCloud2ColorSeg(component, color_planecloud);
  //   viewer->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb33(color_planecloud);
    viewer->addPointCloud<pcl::PointXYZRGBNormal>(color_planecloud, rgb33, "cloud_front");


    pcl::PointXYZ front_top_right_p(front_top_right[0],front_top_right[1],front_top_right[2]);
  viewer->addSphere(front_top_right_p, 0.01, 1.0, 1.0, 1.0, "front_top_right");

    pcl::PointXYZ front_bottom_right_p(front_bottom_right[0],front_bottom_right[1],front_bottom_right[2]);
  viewer->addSphere(front_bottom_right_p, 0.01, 1.0, 1.0, 1.0, "front_bottom_right");

    pcl::PointXYZ back_top_right_p(back_top_right[0],back_top_right[1],back_top_right[2]);
  viewer->addSphere(back_top_right_p, 0.01, 1.0, 1.0, 1.0, "back_top_right");

    pcl::PointXYZ front_top_left_p(front_top_left[0],front_top_left[1],front_top_left[2]);
  viewer->addSphere(front_top_left_p, 0.01, 1.0, 1.0, 1.0, "front_top_left");

    pcl::PointXYZ handle_top_p(handle_top[0],handle_top[1],handle_top[2]);
  viewer->addSphere(handle_top_p, 0.01, 1.0, 1.0, 1.0, "handle_top");

    pcl::PointXYZ handle_centroid_p(handle_centroid[0],handle_centroid[1],handle_centroid[2]);
  viewer->addSphere(handle_centroid_p, 0.01, 1.0, 1.0, 1.0, "handle_centroid");


  pcl::PointXYZ handle_bottom_p(handle_bottom[0],handle_bottom[1],handle_bottom[2]);
  viewer->addSphere(handle_bottom_p, 0.01, 1.0, 1.0, 1.0, "handle_bottom");

  // viewer->addCube(right_side_centroid, microwave_pose, right_side_dimension[0],
  //   right_side_dimension[1], right_side_dimension[2], "right_side");

  // viewer->addCube(top_centroid, microwave_pose, top_dimension[0],
  //   top_dimension[1], top_dimension[2], "top_side");

  // viewer->addCube(bottom_centroid, microwave_pose, bottom_dimension[0],
  //   bottom_dimension[1], bottom_dimension[2], "bottom_side");

  // viewer->addCube(left_side_centroid, microwave_pose, left_side_dimension[0],
  //   left_side_dimension[1], left_side_dimension[2], "left_side");


  // viewer->addCube(door_centroid, microwave_pose, door_dimension[0],
  //   door_dimension[1], door_dimension[2], "door_side");

  // viewer->addCube(handle_centroid, microwave_pose, handle_dimension[0],
  //   handle_dimension[1], handle_dimension[2], "handle");



  viewer->spin();




  // viewer->spin();


  //   pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  //   std::vector<std::vector<int> > temp;
  //   temp.push_back(frontDoor);
  //   ConvertPCLCloud2ColorSeg(temp, color_cloud);
  //   viewer->removeAllPointClouds();
  //   pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb3(color_cloud);
  // viewer->addPointCloud<pcl::PointXYZRGBNormal>(color_cloud, rgb3, "cloud");
  // viewer->spin();
 
  //   temp.clear();
  //   temp.push_back(topFace);
  //   ConvertPCLCloud2ColorSeg(temp, color_cloud);
  //   viewer->removeAllPointClouds();
  //   rgb3.setInputCloud(color_cloud);
  // viewer->addPointCloud<pcl::PointXYZRGBNormal>(color_cloud, rgb3, "cloud");
  // viewer->spin();

  //   ConvertPCLCloud2ColorSeg(handleIndices, color_cloud);
  //   viewer->removeAllPointClouds();
  //   rgb3.setInputCloud(color_cloud);
  // viewer->addPointCloud<pcl::PointXYZRGBNormal>(color_cloud, rgb3, "cloud");
  // viewer->spin();



}

