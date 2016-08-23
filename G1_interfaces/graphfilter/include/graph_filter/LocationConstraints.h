#ifndef LOCATIONCONSTRAINTS_H
#define LOCATIONCONSTRAINTS_H

#include <stdio.h>
#include <map>
#include <iostream>
#include <vector>
#include <pcl/common/pca.h>
#include <pcl/point_cloud.h>
#include <graph_filter/NodeConstraints.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <graph_filter/Centroid.h>


class LocationConstraints: public NodeConstraints {
  public:
     LocationConstraints(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_,
     	std::vector<std::vector<int> > & planePointIndices,
      Centroid& c_,
      std::vector<float> & loc_constraints_,
     	std::map<int, int> & req_nums_);
     ~LocationConstraints();

     virtual int check(int a);
     virtual std::string getClassName();

  private:
  	 const std::vector<std::vector<int> > & planePointIndices;
     pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_;
     Centroid& centroid;
     const std::vector<float> loc_constraints;
 
};

#endif

