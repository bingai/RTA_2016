#include <graph_filter/Centroid.h>


Centroid::Centroid(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr i_,
  std::vector<std::vector<int> > & pPI):
  input_(i_), planePointIndices(pPI) {

  centroid.clear();
}

Centroid::~Centroid() {

}

Eigen::Vector3f Centroid::getCentroid(int id) {
  if (centroid.find(id) != centroid.end()) {
    return centroid[id];
  }
  Eigen::Vector3f c(0,0,0);
  for (int i = 0; i < planePointIndices[id].size(); i++){
    c = c + input_->points[planePointIndices[id][i] ].getVector3fMap();
  }
  c = c / planePointIndices[id].size();
  centroid[id] = c;
  return centroid[id];
}