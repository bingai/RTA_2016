#ifndef CENTROID_H
#define CENTROID_H

#include <stdio.h>
#include <map>
#include <iostream>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

class Centroid {
  public:
   Centroid(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr i_,
      std::vector<std::vector<int> > & pPI);
   ~Centroid();
   Eigen::Vector3f getCentroid(int id);
  private:
  	 const std::vector<std::vector<int> > & planePointIndices;
     pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_;
     std::map<int, Eigen::Vector3f> centroid;
};

#endif