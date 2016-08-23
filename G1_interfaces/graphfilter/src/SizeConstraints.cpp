#include <graph_filter/SizeConstraints.h>


SizeConstraints::SizeConstraints(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr i_,
  std::vector<std::vector<int> > & pPI,
  std::vector<float> & dim_constraints_,
  Pca & pcas_,
  std::map<int, int> & req_nums_):NodeConstraints(req_nums_), 
  input_(i_), planePointIndices(pPI), 
  dim_constraints(dim_constraints_),
  pcas(pcas_){
  
  // pca.setInputCloud(i_);

}


SizeConstraints::~SizeConstraints(){

}

bool SizeConstraints::get3Dims(Eigen::Matrix3f & evs, 
	std::vector<float>& dims, pcl::IndicesPtr& ind) {
  // for (int i = 0; i < 3; i++) {
  // 	std::cout << evs.col(i) << std::endl << std::endl;
  // }
  
  float d1max = std::numeric_limits<float>::lowest();
  float d1min = std::numeric_limits<float>::max();

  float d2max = std::numeric_limits<float>::lowest();
  float d2min = std::numeric_limits<float>::max();

  for (int i = 0; i < ind->size(); i++) {
  	float d1 = pcl::pointToPlaneDistanceSigned(input_->points[(*ind)[i] ],
      evs(0,0), evs(1,0), evs(2,0), 0);
  	float d2 = pcl::pointToPlaneDistanceSigned(input_->points[(*ind)[i] ],
      evs(0,1), evs(1,1), evs(2,1), 0);

  	if (d1 > d1max) {
  		d1max = d1;
  	} else if (d1 < d1min) {
  		d1min = d1;
  	}

  	if (d2 > d2max) {
  		d2max = d2;
  	} else if (d2 < d2min) {
  		d2min = d2;
  	}


  }

  dims.resize(3);
  dims[0] = d1max - d1min;
  dims[1] = d2max - d2min;
  dims[2] = -1;


  return true;
}

int SizeConstraints::check(int a) {
  if (planePointIndices[a].size() < 2000) {
    return 0;
  }

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
  std::vector<float> dims;
  get3Dims(eigen_vectors, dims, ind);

  if (dims[0] > dim_constraints[0] && dims[0] < dim_constraints[1] 
    && dims[1] < dim_constraints[3]) {
  	std::cout << a << " : " << dims[0] << " " << dims[1] << "  " << planePointIndices[a].size() << std::endl;
  	return 1;
  } else {
   return 0;
  }
}


std::string SizeConstraints::getClassName() {
  std::cout << "[class] SizeConstraints " << std::endl;
  return "SizeConstraints";
}