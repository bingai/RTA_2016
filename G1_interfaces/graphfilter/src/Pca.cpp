#include <graph_filter/Pca.h>


Pca::Pca(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr i_,
  std::vector<std::vector<int> > & pPI):
  input_(i_), planePointIndices(pPI) {

  pca.setInputCloud(i_);
  pcas.clear();
}

Pca::~Pca() {

}

Eigen::Matrix3f Pca::getPca(int id) {
  if (pcas.find(id) != pcas.end()) {
    return pcas[id];
  }

  pcl::IndicesPtr ind(new std::vector<int>(planePointIndices[id]) ) ;
  pca.setIndices(ind);
  Eigen::Matrix3f eigen_vectors;
  try
  {
    eigen_vectors = pca.getEigenVectors ();
  }
  catch (pcl::InitFailedException &/*e*/)
  {
    std::cerr << "PCA wrong" << std::endl;
  }
  
  pcas[id] = eigen_vectors;
  return pcas[id];


}