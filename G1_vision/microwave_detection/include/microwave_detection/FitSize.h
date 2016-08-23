

#ifndef FITSIZE
#define FIZSIZE

#include <vector>
#include <stdio.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <map>

class FitSize
{
public:
  
protected:

private:

  bool have_cloud;
  bool have_indices;
  
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pcl_cloud;
  
  std::vector<std::vector<int> > planePointIndices;

  void sortLength(float x, float y, float z, float& l1, float& l2, float& l3);

public:
  FitSize();
  ~FitSize();
  
  void setInputCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pcl_cloud);
  void setSurfaceIndices(std::vector<std::vector<int> > indices);

  /** Compute if size fit **/
  void compute(std::map<int, int>& sizeFit);
  
  int fitCubeSize(std::vector<int> ind);
  std::vector<float> get3DBox(std::vector<int> ind);
  bool isHandleShape(const std::vector<int> & indices);
};


#endif

