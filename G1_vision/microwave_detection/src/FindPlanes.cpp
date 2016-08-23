

#include <microwave_detection/FindPlanes.h>


/********************** FindPlanes ************************
 * Constructor/Destructor
 */
FindPlanes::FindPlanes(Parameter p)
{
  setParameter(p);
  pixel_check = false;
  max_neighbours = 4;
  max_nneighbours = 2*max_neighbours;
  
  srt_curvature.resize(20);
}

FindPlanes::~FindPlanes()
{
}

/************************** PRIVATE ************************/

void FindPlanes::CreatePatchImage()
{

  patches = cv::Mat_<int>(cloud->height, cloud->width);
  
  patches.setTo(0);

  for(unsigned i=0; i<planes.size(); i++) {
    for(unsigned j=0; j<planes[i].size(); j++) {
      int row = planes[i][j] / cloud->width;
      int col = planes[i][j] % cloud->width;
      patches(row, col) = i+1;   // plane 1,2,...,n
    }
  }
}

// /**
//  * Count number of neighbouring pixels (8-neighbourhood)
//  * @param nb Maximum neighbouring pixels
//  * @param nnb Maximum neighbouring pixels with neighbouring neighbors
//  * @param nnb_inc Increment value for neighbouring neighbors
//  */
void FindPlanes::CountNeighbours(std::vector< std::vector<int> > &reassign_idxs,
                                             int nb, int nnb, int nnb_inc)
{
  reassign_idxs.clear();
  reassign_idxs.resize(planes.size());
  
  #pragma omp parallel for
  for(int row=1; row<patches.rows-1; row++) {
    for(int col=1; col<patches.cols-1; col++) {
      
      int nb_counter = 0;
      int nnb_counter = 0;
      bool neighbors[8] = {false, false, false, false, false, false, false, false};
      if(patches(row, col) == 0) continue;
      if(patches(row, col) == patches(row, col-1)) {nb_counter++; neighbors[0] = true;}
      if(patches(row, col) == patches(row-1, col-1)) {nb_counter++; neighbors[1] = true;}
      if(patches(row, col) == patches(row-1, col)) {nb_counter++; neighbors[2] = true;}
      if(patches(row, col) == patches(row-1, col+1)) {nb_counter++; neighbors[3] = true;}
      if(patches(row, col) == patches(row, col+1)) {nb_counter++; neighbors[4] = true;}
      if(patches(row, col) == patches(row+1, col+1)) {nb_counter++; neighbors[5] = true;}
      if(patches(row, col) == patches(row+1, col)) {nb_counter++; neighbors[6] = true;}
      if(patches(row, col) == patches(row+1, col-1)) {nb_counter++; neighbors[7] = true;}
        
      for(unsigned i=0; i<8; i++) {
        int j = i+1;
        if(j > 7) j=0;
        if(neighbors[i] == true && neighbors[j] == true)
          nnb_counter += nnb_inc;
      }

      if(nb_counter <= nb && (nb_counter+nnb_counter) <= nnb)
        if(mask[row*patches.cols + col] == 0) {
          #pragma omp critical 
          {
            mask[row*patches.cols + col] = 1;
            reassign_idxs[patches(row, col) - 1].push_back(row*patches.cols + col);
          }
        }
    }
  }
}

// /**
//  * Reasign points to neighbouring patches
//  * We allow the inlier distance to assign points to other patches
//  */
bool FindPlanes::ReasignPoints(std::vector< std::vector<int> > &reassign_idxs)
{
  bool ready = false;
  bool assigned = false;
  bool have_assigned = false;
  while(!ready) {
    assigned = false;
    
    for(int i=(int)planes.size()-1; i>=0 ; i--) {
      std::vector<int> not_assigned_indexes;
      for(int j=0; j< (int) reassign_idxs[i].size(); j++) {
        int idx = reassign_idxs[i][j];
        unsigned row = reassign_idxs[i][j] / cloud->width;
        unsigned col = reassign_idxs[i][j] % cloud->width;
        if(row >= 0 && row < cloud->height && col >= 0 && col < cloud->width) {
          int surounding[planes.size()];
          for(unsigned s=0; s<planes.size(); s++)
            surounding[s] = 0;
        
          for(unsigned v=row-1; v<=row+1; v++) {
            for(unsigned u=col-1; u<=col+1; u++) {
              if(v!=row || u!=col) {
                int a_idx = v*cloud->width + u;
            
                float dist = 1.0;
                if(!isnan(cloud->points[a_idx].z))
                  dist = (cloud->points[idx].getVector3fMap() - cloud->points[a_idx].getVector3fMap()).norm();
                
                if(patches(v, u) != 0 && 
                  patches(v, u) != patches(row, col) &&
                  dist < param.ra_dist)
                  surounding[patches(v, u) -1]++;
              }
            }
          }
          
          int max_neighbors = 0;
          int most_id = 0;
          for(unsigned nr=0; nr<planes.size(); nr++) {
            if(max_neighbors < surounding[nr]) {
              max_neighbors = surounding[nr];
              most_id = nr;
            }
          }

          if(max_neighbors > 0) {
            have_assigned = true;
            assigned = true;
            planes[most_id].push_back(idx);

            std::vector<int> surfaces_indices_copy;
            for(int su=0; su < (int)planes[i].size(); su++)
              if(planes[i][su] != idx)
                surfaces_indices_copy.push_back(planes[i][su]);
            
            planes[i] = surfaces_indices_copy;
              
            patches(row, col) = most_id+1;
          }
          else {
            not_assigned_indexes.push_back(idx);
          }
        }
      }
      reassign_idxs[i] = not_assigned_indexes;
    }
    if(!assigned)
      ready = true;
  }
  return have_assigned;
}


void FindPlanes::DeleteEmptyPlanes()
{ 
  std::vector<std::vector<int> > planes_copy;
  for(int su=0; su<(int)planes.size(); su++) {
    if((int)planes[su].size() > 0) {
      if((int)planes[su].size() < param.minPoints) {
        // view->surfaces[su]->type = -1;
        // planes_copy.push_back(view->surfaces[su]);
      } else {
        planes_copy.push_back(planes[su]);
      }
    }
  }
  planes = planes_copy;
}

// /* Reasign single pixels (line-ends) */
void FindPlanes::SinglePixelCheck()
{
  int max_nb = 2;
  int max_nnb = 1;
  int nnb_inc = -1;
  bool assigned = true;
  std::vector< std::vector<int> > reassign_idxs;
  while(assigned) {
    CountNeighbours(reassign_idxs, max_nb, max_nnb, nnb_inc);
    assigned = ReasignPoints(reassign_idxs);
  }
}


void FindPlanes::PixelCheck()
{ 


  std::vector< std::vector<int> > reassign_idxs;
  mask.clear(); // move pixel only once
  mask.resize(cloud->width*cloud->height, 0);

  // Reassign patches which have less neighbouring points than minPoints
  
  CreatePatchImage();

  CountNeighbours(reassign_idxs, max_neighbours, max_nneighbours, 1);

  for(unsigned i=0; i<planes.size(); i++) {
    if((planes[i].size() - reassign_idxs[i].size()) < param.minPoints)
      reassign_idxs[i] = planes[i];
    else
      reassign_idxs[i].clear();
  }
  ReasignPoints(reassign_idxs);

  // Reassign single-neighboured points of patches
  mask.clear();
  mask.resize(cloud->width*cloud->height,0);
  SinglePixelCheck();
  DeleteEmptyPlanes();
  

}


/**
 * Cluster rest of the points
 */
// void FindPlanes::ClusterRest(unsigned idx, 
//                                          pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
//                                          pcl::PointCloud<pcl::Normal> &normals, 
//                                          std::vector<int> &pts,
//                                          pcl::Normal &normal)
// {
//   short x,y;
//   normal = normals.points[idx];
//   pts.clear();

//   if (isnan(cloud.points[idx].x))
//     printf("[FindPlanes::ClusterRest] Error: NAN found.\n");

//   mask[idx] = 1;
//   pts.push_back(idx);
  
//   int queue_idx = 0;
//   std::vector<int> queue;
//   queue.reserve(cloud.width*cloud.height);
//   queue.push_back(idx);       
  
//   while ((int)queue.size() > queue_idx) {
//     idx = queue[queue_idx];
//     queue_idx++;
//     x = X(idx);
//     y = Y(idx);

//     for (int v=y-1; v<=y+1; v++) {
//       for (int u=x-1; u<=x+1; u++) {
//         if (v>=0 && u>=0 && v<height && u<width) {
//           idx = GetIdx(u,v);
//           if (mask[idx] == 0) {
//               mask[idx] = 1;

//               Mul3(&normal.normal[0], pts.size(), &normal.normal[0]);
//               Add3(&normal.normal[0], &normals.points[idx].normal[0], &normal.normal[0]);

//               pts.push_back(idx);
//               queue.push_back(idx);
              
//               Mul3(&normal.normal[0], 1./(float)pts.size(), &normal.normal[0]);
//               normal.getNormalVector3fMap().normalize();
//           }
//         }
//       }
//     }
//   }
// }

void FindPlanes::CalcAdaptive()
{
  p_adaptive_cosThrAngleNC.resize(cloud->width*cloud->height);
  p_adaptive_inlDist.resize(cloud->width*cloud->height);

  #pragma omp parallel for
  for(unsigned i=0; i<cloud->points.size(); i++) {
    Eigen::Vector3f curPt = orig_cloud->points[i].getVector3fMap();
    if(curPt[2] <= param.d_c) {
      p_adaptive_cosThrAngleNC[i] = cos(param.epsilon_c);
    } else {
      p_adaptive_cosThrAngleNC[i] = cos(param.epsilon_c + param.epsilon_g*(curPt[2]-param.d_c));
    }
    p_adaptive_inlDist[i] = param.omega_c + param.omega_g*curPt[2];
  }
}

/**
 * ClusterNormals
 */
void FindPlanes::ClusterNormals(unsigned idx, 
     pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud, std::vector<int> &planes,
    pcl::Normal &plane_normal) {

  Eigen::Vector3f Pnormal = cloud.points[idx].getNormalVector3fMap();
  planes.clear();

  if (Pnormal[0]!=Pnormal[0]) {
    return;
  }

  mask[idx] = 1;
  planes.push_back(idx);
  
  int queue_idx = 0;
  std::vector<int> queue;
  queue.reserve(cloud.width*cloud.height);
  queue.push_back(idx);
  
  EIGEN_ALIGN16 Eigen::Vector3f pt = cloud.points[idx].getVector3fMap();

  while ((int)queue.size() > queue_idx) {
    int iddx = queue[queue_idx];
    queue_idx++;

    std::vector<int> n4ind;
    n4ind.push_back(iddx-1);
    n4ind.push_back(iddx+1);
    n4ind.push_back(iddx+width);
    n4ind.push_back(iddx-width);
    for(unsigned i=0; i<n4ind.size(); i++) {
      int u = n4ind[i] % width;
      int v = n4ind[i] / width;

      if (v>=0 && u>=0 && v<height && u<width) {
        int pid = GetIdx(u,v);
        if (mask[pid]==0) {
          Eigen::Vector3f n = cloud.points[pid].getNormalVector3fMap();
          Eigen::Vector3f p = cloud.points[pid].getVector3fMap();
          if (n[0]!= n[0]) {
            continue;
          }

          float newCosThrAngleNC = cosThrAngleNC;
          float newInlDist = param.inlDist;
          if(param.adaptive) {
            // std::cout << "adapt" << std::endl;
            newCosThrAngleNC = p_adaptive_cosThrAngleNC[pid];
            newInlDist = p_adaptive_inlDist[pid];
          }
          
          if ( Pnormal.dot(n) > newCosThrAngleNC && 
            fabs((pt - p).dot(Pnormal)) < newInlDist ) {
            mask[pid] = 1;
            Pnormal = Pnormal * planes.size();
            Pnormal += n;
            planes.push_back(pid);
            queue.push_back(pid);
            Pnormal.normalize();
          }
        }
      }
    }
  }

  // // // check if all points are on the plane 
  // unsigned ptsSize = planes.size();
  // if( (int) planes.size() < param.minPoints) {
  //   return;
  // }
  
  // for(unsigned i=0; i<ptsSize; i++) {
  //   Eigen::Vector3f n = cloud.points[planes[i]].getNormalVector3fMap();
  //   Eigen::Vector3f p = cloud.points[planes[i]].getVector3fMap();

  //   float newCosThrAngleNC = cosThrAngleNC;
  //   float newInlDist = param.inlDist;
  //   if(param.adaptive) {
  //     newCosThrAngleNC = p_adaptive_cosThrAngleNC[idx];
  //     newInlDist = p_adaptive_inlDist[idx];
  //   }          
          
  //   if (Pnormal.dot(n) < newCosThrAngleNC ||
  //       fabs((pt - p).dot(Pnormal)) > newInlDist)
  //   {
  //     mask[planes[i]]=0;
  //     planes.erase(planes.begin()+i);
  //     ptsSize--;
  //     i--;
  //   }
  // }

  // // recalculate plane normal
  // if(planes.size() > 0) {
  //   EIGEN_ALIGN16 Eigen::Vector3f new_n = cloud.points[planes[0]].getNormalVector3fMap();
  //   for(unsigned i=1; i<planes.size(); i++) {
  //     new_n += cloud.points[planes[i]].getNormalVector3fMap();
  //   }
  //   new_n.normalize();
  //   plane_normal.normal[0] = new_n[0];
  //   plane_normal.normal[1] = new_n[1];
  //   plane_normal.normal[2] = new_n[2];
  // }
}



/**
 * ClusterNormals
 */
void FindPlanes::ClusterNormals(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud, 
         std::vector<std::vector<int> > &planes)
{
  
  // init mask (without nans)
  planes.clear();
  planecoeffs.clear();
  mask.clear();
  mask.resize(cloud.width*cloud.height,0);
  for (unsigned i=0; i<cloud.width*cloud.height; i++) {
    if(isnan(cloud.points[i].x)) {
      mask[i] = 1;
    }
  }

  for(unsigned i=0; i< srt_curvature.size(); i++) {
    srt_curvature[i].clear();
  }

  for(int idx=0; idx< (int) cloud.points.size(); idx++) {
    if(cloud.points[idx].curvature == 0.0) {
      if (mask[idx]==0) {
        srt_curvature[0].push_back(idx);
      }
    } else {
      double curvature = cloud.points[idx].curvature*1000;
      unsigned curv = (unsigned) curvature;
      if(curv > 19) {
        curv = 19;
      }
      if (mask[idx]==0) {
        srt_curvature[curv].push_back(idx);
      }
    }
  }
  
  // cluster points
  bool morePlanes = true;
  while(morePlanes) {
    morePlanes = false;
    for(unsigned i=0; i<srt_curvature.size(); i++) {
      for(unsigned j=0; j<srt_curvature[i].size(); j++) {
        unsigned idx = srt_curvature[i][j]; 
        if (mask[idx]!=0) { continue;}
        std::vector<int> plane;
        pcl::Normal plane_normal;
 
        ClusterNormals(idx, cloud, plane, plane_normal);

        if (plane.size() >= param.minPoints) {
          morePlanes = true;
          planes.push_back(plane);
          planecoeffs.push_back(plane_normal);

        } else {
          for (unsigned i=0; i<plane.size(); i++) {
            mask[plane[i]]=0;
          }
        }
      }
      
    }
  }
  

  
}

// /**
//  * ComputeLSPlanes to check plane models
//  */
template<typename T1,typename T2>
T1 FindPlanes::Dot3(const T1 v1[3], const T2 v2[3])
{
  return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

void FindPlanes::ComputeLSPlanes(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud, 
                                             std::vector<std::vector<int> > &planes)
{
  pcl::SampleConsensusModelPlane<pcl::PointXYZRGBNormal> lsPlane(cloud);
  Eigen::VectorXf coeffs(4);

  for (unsigned i=0; i<planes.size(); i++) {
    if(planes[i].size() > 4) {

      lsPlane.optimizeModelCoefficients(planes[i], coeffs, coeffs);

      if (Dot3(&coeffs[0], &cloud->points[planes[i][0]].x) > 0)
        coeffs*=-1.;
      
      // if (Dot3(&coeffs[0], &planecoeffs[i].normal[0]) > cosThrAngleNC) {
        planecoeffs[i].normal[0] = coeffs[0];
        planecoeffs[i].normal[1] = coeffs[1];
        planecoeffs[i].normal[2] = coeffs[2];
        // plane.coeffs[3] = coeffs[3];
      // }

    }
  }
}



/************************** PUBLIC *************************/

/**
 * set the input cloud for detecting planes
 */
void FindPlanes::setInputCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &_cloud)
{
  if (_cloud->height<=1 || _cloud->width<=1 || !_cloud->isOrganized())
    throw std::runtime_error("[FindPlanes::setInputCloud] Invalid point cloud (height must be > 1)");
  cloud = _cloud;
  width = cloud->width;
  height = cloud->height;
}


void FindPlanes::setOrigCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &_cloud)
{
  if (_cloud->height<=1 || _cloud->width<=1 || !_cloud->isOrganized())
    throw std::runtime_error("[FindPlanes::setInputCloud] Invalid point cloud (height must be > 1)");

  orig_cloud = _cloud;
 
}

/**
 * setParameter
 */
void FindPlanes::setParameter(Parameter p)
{
  param = p;
  cosThrAngleNC = cos(param.thrAngle);
}

/**
 * @brief Check if there are patch models with "line"-style (not more than n neighbors)
 * @param check True to check
 * @param neighbors Threshold for line_check neighbors
 */
void FindPlanes::setPixelCheck(bool check, int neighbors)
{
  pixel_check = check;
  max_neighbours = neighbors;
  max_nneighbours = neighbors*2;
}


/**
 * Compute
 */
std::vector<std::vector<int> >
 FindPlanes::compute()
{
  std::cout << "********************** start detecting planes **********************" << std::endl;
  
  if (cloud.get()==0 || orig_cloud.get() == 0)
    throw std::runtime_error("[FindPlanes::compute] Point cloud or view not set!"); 

  if(param.adaptive)
    CalcAdaptive();
  ClusterNormals(*cloud, this->planes);
  // this->planes = indices;  
  if(pixel_check)
    PixelCheck();

  ComputeLSPlanes(cloud, this->planes);

  return this->planes;
}




