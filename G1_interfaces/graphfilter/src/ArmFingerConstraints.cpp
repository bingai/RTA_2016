#include <graph_filter/ArmFingerConstraints.h>
#include <graph_filter/GraphUtilities.h>

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

ArmFingerConstraints::ArmFingerConstraints(std::vector<std::set<int> > & nbgh_list_,
  std::map <int, std::set<int> > & scene2sub_,
  std::map<int, std::set<int> > & subDest_,
  std::map<int, int> & req_nums_,
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr i_,
  std::vector<std::vector<int> > & pPI, Centroid& c_,
  Pca & pcas_):
  EdgeConstraints(nbgh_list_, scene2sub_, true, subDest_, req_nums_),
  input_(i_), planePointIndices(pPI), 
  centroid(c_), pcas(pcas_)
{
  // pca.setInputCloud(i_);
}

ArmFingerConstraints::~ArmFingerConstraints() {

}

std::string ArmFingerConstraints::getClassName() {
  std::cout << "[class] ArmFingerConstraints " << std::endl;
  return "ArmFingerConstraints";
}

std::vector<int> ArmFingerConstraints::getNeighbors(int id) {
  std::vector<int> neighbors;
  
  for (int i = 0; i < planePointIndices.size(); i++) {
    neighbors.push_back(i);
  }
  return neighbors;
}

// arm id: a,  finger id: b
int ArmFingerConstraints::check(int a, int b) {
  // Eigen::Vector3f arm_c = centroid.getCentroid(a);
  Eigen::Vector3f finger_c = centroid.getCentroid(b);

  Eigen::Vector3f arm_c = centroid.getCentroid(a);

  Eigen::Matrix3f evs = pcas.getPca(a);

  Eigen::Matrix3f fevs = pcas.getPca(b);

  float d1max = std::numeric_limits<float>::lowest();
  float d1min = std::numeric_limits<float>::max();

  Eigen::Vector3f pointMax(0,0, 0); 
  Eigen::Vector3f pointMin(0, 0, 0);
  int indMax = -1;
  int indMin = -1;
  
  pcl::IndicesPtr ind(new std::vector<int>(planePointIndices[a]) ) ;
  for (int i = 0; i < ind->size(); i++) {
    float d1 = pcl::pointToPlaneDistanceSigned(input_->points[(*ind)[i] ],
      evs(0,0), evs(1,0), evs(2,0), 0);

    if (d1 > d1max) {
      d1max = d1;
      indMax = (*ind)[i];
      pointMax = input_->points[(*ind)[i]].getVector3fMap();
    } else if (d1 < d1min) {
      pointMin = input_->points[(*ind)[i]].getVector3fMap();
      indMin = (*ind)[i];
      d1min = d1;
    }
  }


  Eigen::Vector3f v1 = evs.col(0);
  Eigen::Vector3f vf = fevs.col(0);

  float f2max = (finger_c - pointMax).dot(v1);
  float d2max = (finger_c - pointMax).norm();
  float f2min = -1 * (finger_c - pointMin).dot(v1);
  float d2min = (finger_c - pointMin).norm();
  float inner = fabs(v1.dot(vf));
  // std::cout << "inner " << inner << std::endl;
  if (inner < 0.6) {
    return 0;
  }
  if ( (f2max < 0.20 && f2max > 0 && d2max < f2max * 1.2) 
      || (f2min > 0 && f2min < 0.20 && d2min < f2min * 1.2) ) {
    std::cout << "||arm: " << a << "   finger: " << b << std::endl;
    std::cout << "||dist max =" << f2max<< std::endl;
    std::cout << "|| dist min =" << f2min << std::endl;
    std::cout << "arm pos " << arm_c.transpose() << "\t" 
      <<"finger pos " << finger_c.transpose() << std::endl;

    if (f2max < 0.20 && f2max > 0 && d2max < f2max * 1.2) {
      // v1
      if (v1[2] > 0) {
        return 0;
      }

      // if (!checkFingerDir(a, b) ) {
      //   return 0;
      // }

      armFingerIndex[getPairHash(a, b)] = indMax;
      pca1dir[a] = true;
    } else {
      // -1 * v1
      if (v1[2] < 0) {
        return 0;
      }
      // if (!checkFingerDir(a, b) ) {
      //   return 0;
      // }

      armFingerIndex[getPairHash(a, b)] = indMin;
      pca1dir[a] = false;
    }

    return 1;
  }
  return 0;


}

int ArmFingerConstraints::getEndPoint(int a, int f) {
  if (armFingerIndex.find(getPairHash(a, f)) == armFingerIndex.end()) {
    std::cout << "Error, Pair does not exist" << std::endl;
    return -1;
  }
  return armFingerIndex[getPairHash(a, f)];
}

int ArmFingerConstraints::getPairHash(int a, int f) {
  return 30000*a + f;
}

void ArmFingerConstraints::getDistAndTip(int a, int f, Eigen::Vector3f &ftip,
  float & dist) {


  Eigen::Matrix3f evsa = pcas.getPca(a);
  Eigen::Matrix3f evsf = pcas.getPca(f);

  float d1max = std::numeric_limits<float>::lowest();
 
  Eigen::Vector3f pointMax(0,0, 0);
  Eigen::Vector3f dira = evsa.col(0);
  Eigen::Vector3f dirf = evsf.col(0);

  if (pca1dir.find(a) == pca1dir.end() ) {
    std::cout << "Error happend in pca1dir, check getDistAndTip" << std::endl;
    return ;
  }
  Eigen::Vector3f dir = dirf;
  if (pca1dir[a]) {
    if (dirf.dot(dira) < 0) {  
      dir = -1 * dir;
    }
  } else {
    if (dirf.dot(dira) * -1 < 0) {  
      dir = -1 * dir;
    }
  }

  pcl::IndicesPtr ind(new std::vector<int>(planePointIndices[f]) ) ;
  for (int i = 0; i < ind->size(); i++) {
    float d1 = pcl::pointToPlaneDistanceSigned(input_->points[(*ind)[i] ],
      dir(0), dir(1), dir(2), 0);

    if (d1 > d1max) {
      d1max = d1;
      pointMax = input_->points[(*ind)[i]].getVector3fMap();
   }

  }

  dist = d1max;
  ftip = pointMax;
  
}

bool ArmFingerConstraints::passLabel(int sid, int l) {
  std::vector<int> neighbors = getNeighbors(sid);
  int count = 0;

  for (std::vector<int>::iterator it = neighbors.begin(); it != neighbors.end(); ++it) {
    if (!isValid(*it, l)) {

      continue;
    }
    if (l == 0) {
      count += getEdgeCheckResult(sid, *it);
    } else {
      count += getEdgeCheckResult(*it, sid);
    }
  } 

  // std::cout << "after for count" << std::endl;

  return count >= req_nums[l];
}


void ArmFingerConstraints::getDistAndTip(int a, int f, int &finger_ind,
  float & dist) {
  Eigen::Matrix3f evsa = pcas.getPca(a);
  // Eigen::Matrix3f evsf = pcas.getPca(f);
  Eigen::Vector3f dir = evsa.col(0);
  if (!pca1dir[a]) {
    dir = -1 * dir;
  }

  float d1max = std::numeric_limits<float>::lowest();
  finger_ind = -1;

  // Eigen::Matrix3d Rd = T_Eigen.matrix().block<3, 3>(0, 0);
  // Eigen::Matrix3f Rf = Rd.cast<float>();
  // Eigen::Vector3f dir2d = Graph::Util::rotateDir(dir, Rf.inverse());
  cv::Mat_<float> visit = cv::Mat_<float>::zeros(input_->width, input_->height);
  

  // BFS to find the finger Tip
  for (int i = 0; i < planePointIndices[f].size(); i++) {
    int x = planePointIndices[f][i] % input_->width;
    int y = planePointIndices[f][i] / input_->width;
    if (visit(x, y) != 0) {
      continue;
    }
    
    std::vector<std::pair<int, int> > que;
    que.push_back(std::pair<int, int>(x, y) );
    visit(x, y) = 1;
    int top = 0;

    while (top < que.size() ) {
      std::pair<int, int> ind = que[top];
      visit(ind.first, ind.second) = 2;

      float d1 = pcl::pointToPlaneDistanceSigned(input_->at(ind.first, ind.second),
        dir(0), dir(1), dir(2), 0);

      if (d1 > d1max) {
        d1max = d1;
        finger_ind = ind.second * input_->width + ind.first;
      }


      top++;
      Eigen::Vector3f p = input_->at(ind.first, ind.second
        ).getVector3fMap();
   
      std::vector<std::pair<int, int> > ngbr = 
        Graph::Util::imageNgbr(ind, input_->width, input_->height);
      for (int ni = 0; ni < ngbr.size(); ni++) {
        int nx = ngbr[ni].first;
        int ny = ngbr[ni].second;
        if (visit(nx, ny) > 0 || !pcl::isFinite(input_->at(nx, ny))) {
          continue;
        }

        Eigen::Vector3f np = input_->at(nx, ny).getVector3fMap();
        float dirdist = (np - p).dot(dir);
        float distl2 = (np - p).norm();
        if (dirdist >= 0 && distl2 < 0.02 && distl2 < 0.03) {
          que.push_back(ngbr[ni]);
          visit(nx, ny) = 1;
        }
      }

    }


  }

}

void ArmFingerConstraints::setTransform(const Eigen::Affine3d & T_Eigen) {
  this->T_Eigen = T_Eigen;
}


Eigen::Vector3f ArmFingerConstraints::getPrincipalDir(int a) {
  Eigen::Matrix3f evsa = pcas.getPca(a);
  // Eigen::Matrix3f evsf = pcas.getPca(f);
  Eigen::Vector3f dir = evsa.col(0);
  if (!pca1dir[a]) {
    dir = -1 * dir;
  } 

  return dir;
}


Eigen::Vector3f ArmFingerConstraints::getPrincipalDirOrig(int a) {
  Eigen::Vector3f dir = getPrincipalDir(a);

  Eigen::Matrix3d Rd = T_Eigen.matrix().block<3, 3>(0, 0);
  Eigen::Matrix3f Rf = Rd.cast<float>();
  Eigen::Vector3f dir2d = Graph::Util::rotateDir(dir, Rf.inverse());

  return dir2d;
}


bool ArmFingerConstraints::checkFingerDir(int a, int f) {
  Eigen::Vector3f dir = getPrincipalDir(a);

  float d1max = std::numeric_limits<float>::lowest();

  Eigen::Vector3f finger_c = centroid.getCentroid(f);
  float finger_d = -1 * finger_c.dot(dir);
  cv::Mat_<float> visit = cv::Mat_<float>::zeros(input_->width, input_->height);
  
  float dist_thresh = 0.2;


  // BFS to find the furthest point
  for (int i = 0; i < planePointIndices[f].size(); i++) {
    int x = planePointIndices[f][i] % input_->width;
    int y = planePointIndices[f][i] / input_->width;
    if (visit(x, y) != 0) {
      continue;
    }
    
    std::vector<std::pair<int, int> > que;
    que.push_back(std::pair<int, int>(x, y) );
    visit(x, y) = 1;
    int top = 0;

    while (top < que.size() ) {
      std::pair<int, int> ind = que[top];
      visit(ind.first, ind.second) = 2;

      float d1 = pcl::pointToPlaneDistanceSigned(input_->at(ind.first, ind.second),
        dir(0), dir(1), dir(2), finger_d);

      if (d1 > d1max) {
        d1max = d1;
        // finger_ind = ind.second * input_->width + ind.first;
        if (d1max > dist_thresh) {
          return false;
        }
      }

      top++;
      Eigen::Vector3f p = input_->at(ind.first, ind.second
        ).getVector3fMap();
   
      std::vector<std::pair<int, int> > ngbr = 
        Graph::Util::imageNgbr(ind, input_->width, input_->height);
      for (int ni = 0; ni < ngbr.size(); ni++) {
        int nx = ngbr[ni].first;
        int ny = ngbr[ni].second;
        if (visit(nx, ny) > 0 || !pcl::isFinite(input_->at(nx, ny))) {
          continue;
        }

        Eigen::Vector3f np = input_->at(nx, ny).getVector3fMap();
        float dirdist = (np - p).dot(dir);
        float distl2 = (np - p).norm();
        if (dirdist >= -0.01 && distl2 < 0.02 && distl2 < 0.03) {
          que.push_back(ngbr[ni]);
          visit(nx, ny) = 1;
        }
      }

    }


  }

  return true;

}