#include <graph_filter/Trainer.h>

Trainer::Trainer() {
	initialize();
}

Trainer::~Trainer() {

}

void Trainer::initialize() {
  samples.clear();
}

void Trainer::addTrainingSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_,
  const Eigen::Affine3d & T_Eigen) {
  
  GraphFilter gf;
  gf.initialize();
  gf.setInputCloudAndTransform(cloud_, T_Eigen);
  samples.push_back(gf);

}


