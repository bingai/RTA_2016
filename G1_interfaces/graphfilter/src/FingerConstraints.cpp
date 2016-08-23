#include <graph_filter/FingerConstraints.h>


FingerConstraints::FingerConstraints(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr i_,
  std::vector<std::vector<int> > & pPI,
  Pca & pcas_,
  std::map<int, int> & req_nums_
  ):NodeConstraints(req_nums_), 
  input_(i_), planePointIndices(pPI),
  pcas(pcas_){

  
  // pca.setInputCloud(i_);

}


FingerConstraints::~FingerConstraints(){

}

float FingerConstraints::getWidth(Eigen::Vector3f v1, Eigen::Vector3f v2,
  Eigen::Vector3f bottom, float step,
  pcl::IndicesPtr& ind) {
    
  float v1d = v1.dot(bottom) * -1;
  float d2max = std::numeric_limits<float>::lowest();
  float d2min = std::numeric_limits<float>::max();

  for (int i = 0; i < ind->size(); i++) {
    float d1 = pcl::pointToPlaneDistanceSigned(input_->points[(*ind)[i] ],
      v1(0), v1(1), v1(2), v1d);
    if (d1 < 0 || d1 > step) {
      continue;
    }

    float d2 = pcl::pointToPlaneDistanceSigned(input_->points[(*ind)[i] ],
      v2(0), v2(1), v2(2), 0);

    if (d2 > d2max) {
      d2max = d2;
    } else if (d2 < d2min) {
      d2min = d2;
    }  

  }

  if(d2max <= d2min) {
    return 0;
  } else {
    return d2max - d2min;
  }
}

bool FingerConstraints::findConsecutiveLength(
  std::vector<float>& arr, int length, float w) {
  bool result = true;
  int b = 0;
  for (; b < length; b++) {
    if (arr[b] > 0.00004) {
      break;
    }
  }
  if (b + length >= arr.size() ) {
    return false;
  }

  for (int i = b; i < length; i++) {
    if (arr[i] >  w) {
      result = false;
      break;
    }
  }
  if (result) {
    return true;
  }
  b = arr.size() - 1;
  for (; b > -1; b--) {
    if (arr[b] > 0.0004) {
      break;
    }
  }
  if (b - length < -1) {
    return false;
  }

  for (int i = b; i > b - length; i--) {
    if (arr[i] > w) {
      return false;
    }
  }

  return true;
}

int FingerConstraints::findFinger(Eigen::Matrix3f & evs, pcl::IndicesPtr& ind) {
  // for (int i = 0; i < 3; i++) {
  // 	std::cout << evs.col(i) << std::endl << std::endl;
  // }
  

  float d1max = std::numeric_limits<float>::lowest();
  float d1min = std::numeric_limits<float>::max();

  float d2max = std::numeric_limits<float>::lowest();
  float d2min = std::numeric_limits<float>::max();

  Eigen::Vector3f pointMax(0,0, 0); 
  Eigen::Vector3f pointMin(0, 0, 0);
  for (int i = 0; i < ind->size(); i++) {
  	float d1 = pcl::pointToPlaneDistanceSigned(input_->points[(*ind)[i] ],
      evs(0,0), evs(1,0), evs(2,0), 0);
  	float d2 = pcl::pointToPlaneDistanceSigned(input_->points[(*ind)[i] ],
      evs(0,1), evs(1,1), evs(2,1), 0);

  	if (d1 > d1max) {
  		d1max = d1;
      pointMax = input_->points[(*ind)[i]].getVector3fMap();
  	} else if (d1 < d1min) {
      pointMin = input_->points[(*ind)[i]].getVector3fMap();
  		d1min = d1;
  	}

  	if (d2 > d2max) {
  		d2max = d2;
  	} else if (d2 < d2min) {
  		d2min = d2;
  	}


  }

  float d1 = d1max - d1min, d2 = d2max - d2min;
  if (d1 >= 0.2 || d2 > 0.2) {
    return 0;
  }
  Eigen::Vector3f v1 = evs.col(0);
  Eigen::Vector3f v2 = evs.col(1);
  // sample the line from base to top, see the width of palm at each sampled point
  int numSamples = 19;
  std::vector<float> widthArr;
  float step = d1 / (float) numSamples;
  step = 0.01;

  for (int i = 0; i < numSamples - 1; i++) {
    Eigen::Vector3f bottom = pointMin + v1* step * i;
    // Eigen::Vector3f top = pointMin + v1* step * (i + 1);
    float width = getWidth(v1, v2, bottom, step, ind);
    // std::cout << width << " ";
    widthArr.push_back(width);
  }
  // std::cout << std::endl;

  if (findConsecutiveLength(widthArr, 3, 0.032) ) {
    // for (int i = 0; i < widthArr.size(); i++) {
    //   std::cout << widthArr[i] << " ";
    // }
    // std::cout << std::endl;
    std::cout << d1 << " " << d2 << " " << ind->size() <<  std::endl;
    return 1;
  } else {
    return 0;
  }

  // return 1;
}

int FingerConstraints::check(int a) {
  // if (planePointIndices[a].size() > 7000) {
  //   return 0;
  // }

  pcl::IndicesPtr ind(new std::vector<int>(planePointIndices[a]) ) ;
  // pca.setIndices(ind);
  // Eigen::Matrix3f eigen_vectors;
  // try
  // {
  //   eigen_vectors = pca.getEigenVectors ();
  // }
  // catch (pcl::InitFailedException &/*e*/)
  // {
  //   std::cerr << "PCA wrong" << std::endl;
  // }
  Eigen::Matrix3f eigen_vectors = pcas.getPca(a);


  return findFinger(eigen_vectors, ind);
}


std::string FingerConstraints::getClassName() {
  std::cout << "[class] FingerConstraints " << std::endl;
  return "FingerConstraints";
}