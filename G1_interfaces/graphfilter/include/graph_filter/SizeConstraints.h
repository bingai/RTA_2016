#ifndef SIZECONSTRAINTS_H
#define SIZECONSTRAINTS_H

#include <stdio.h>
#include <map>
#include <iostream>
#include <vector>
#include <pcl/common/pca.h>
#include <pcl/point_cloud.h>
#include <graph_filter/NodeConstraints.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>


#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/pcl_search.h>
#include <pcl/features/normal_3d_omp.h>
#include <Eigen/Dense>
#include <pcl/common/intersections.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/extract_clusters.h>

#include <graph_filter/Pca.h>


class SizeConstraints: public NodeConstraints {
  public:
     SizeConstraints(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_,
     	std::vector<std::vector<int> > & planePointIndices,
      std::vector<float>& dim_constraints_,
      Pca & pcas_,
     	std::map<int, int> & req_nums_);
      ~SizeConstraints();

      virtual int check(int a);
      virtual std::string getClassName();


  private:
  	 const std::vector<std::vector<int> > & planePointIndices;
     pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_;
     // pcl::PCA<pcl::PointXYZRGBNormal> pca;
     Pca & pcas;
     const std::vector<float> dim_constraints;

     bool get3Dims(Eigen::Matrix3f & evs, std::vector<float>& dims, 
       pcl::IndicesPtr& ind);
};

#endif
