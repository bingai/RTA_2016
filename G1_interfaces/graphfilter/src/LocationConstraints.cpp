#include <graph_filter/LocationConstraints.h>


LocationConstraints::LocationConstraints(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr i_,
  std::vector<std::vector<int> > & pPI,
  Centroid& c_,
  std::vector<float> & loc_constraints_,
  std::map<int, int> & req_nums_
  ):NodeConstraints(req_nums_), 
  input_(i_), planePointIndices(pPI), 
  loc_constraints(loc_constraints_),
  centroid(c_){

}


LocationConstraints::~LocationConstraints(){

}


int LocationConstraints::check(int a) {
  // Eigen::Vector3f c(0,0,0);
  // for (int i = 0; i < planePointIndices[a].size(); i++){
  //   c = c + input_->points[planePointIndices[a][i] ].getVector3fMap();
  // }
  // c = c / planePointIndices[a].size();
  Eigen::Vector3f c = centroid.getCentroid(a);
  if (c[0] > loc_constraints[0] && c[0] < loc_constraints[1] 
    && c[2] > loc_constraints[4] && c[2] < loc_constraints[5]) {
    return 1;
  } else {
    return 0;
  }

}


std::string LocationConstraints::getClassName() {
  std::cout << "[class] LocationConstraints " << std::endl;
  return "LocationConstraints";
}
