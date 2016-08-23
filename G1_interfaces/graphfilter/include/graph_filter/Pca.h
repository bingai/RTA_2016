#ifndef PCA_H
#define PCA_H

#include <stdio.h>
#include <map>
#include <iostream>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/common/pca.h>

class Pca {
  public:
   Pca(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr i_,
      std::vector<std::vector<int> > & pPI);
   ~Pca();
   Eigen::Matrix3f getPca(int id);
  private:
  	 const std::vector<std::vector<int> > & planePointIndices;
     pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_;
     std::map<int, Eigen::Matrix3f> pcas;
     pcl::PCA<pcl::PointXYZRGBNormal> pca;

};

#endif