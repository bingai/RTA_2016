#ifndef ARMFINGERCONSTRAINTS_H
#define ARMFINGERCONSTRAINTS_H

#include <stdio.h>
#include <map>
#include <iostream>
#include <vector>
#include <set>
#include <graph_filter/EdgeConstraints.h>
#include <pcl/common/pca.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <graph_filter/Centroid.h>

#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/search/pcl_search.h>
#include <pcl/features/normal_3d_omp.h>
#include <Eigen/Dense>
#include <pcl/common/intersections.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/extract_clusters.h>

#include <graph_filter/Pca.h>
#include <graph_filter/Centroid.h>

// Edge constraints from arm to finger
class ArmFingerConstraints: public EdgeConstraints {

  public:
  	ArmFingerConstraints(std::vector<std::set<int> > & nbgh_list,
      std::map <int, std::set<int> > & scene2sub,
      std::map<int, std::set<int> >& subDest_,
      std::map<int, int> & req_nums_,
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr i_,
      std::vector<std::vector<int> > & pPI,
      Centroid& c_,
      Pca & pcas_
    );
    ~ArmFingerConstraints();

    void getDistAndTip(int a, int f, Eigen::Vector3f &ftip,
      float & dist);
    int getEndPoint(int a, int f);

    void getDistAndTip(int a, int f, int &finger_ind,
      float & dist);
    void setTransform(const Eigen::Affine3d & T_Eigen);   
    Eigen::Vector3f getPrincipalDir(int a);
    Eigen::Vector3f getPrincipalDirOrig(int a);
  

    virtual std::string getClassName();
    virtual std::vector<int> getNeighbors(int id);
    virtual int check(int a, int b);
    virtual bool passLabel(int sid, int l);

  private:
  	 const std::vector<std::vector<int> > & planePointIndices;
     pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_;
     // pcl::PCA<pcl::PointXYZRGBNormal> pca; 
     Centroid& centroid;
     Pca & pcas;
     std::map<int, bool> pca1dir;
     std::map<int, int> armFingerIndex;
     int getPairHash(int a, int f);
     Eigen::Affine3d T_Eigen;
     bool checkFingerDir(int a, int f);
};

#endif