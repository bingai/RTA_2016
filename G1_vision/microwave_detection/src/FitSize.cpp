
#include <microwave_detection/FitSize.h>

FitSize::FitSize()
{
  have_cloud = false;
  have_indices = false;
}

FitSize::~FitSize()
{
}



void FitSize::setInputCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pcl_cloud)
{
  this->pcl_cloud = pcl_cloud;
  have_cloud = true;
}

void FitSize::setSurfaceIndices(std::vector<std::vector<int> > indices) {
  planePointIndices = indices;
  have_indices = true;
}


void FitSize::sortLength(float x, float y, float z, float& l1, float& l2, float& l3) {
  l1 = std::max( std::max (x, y), z);
  l3 = std::min( std::min (x, y), z);
  l2 = x + y + z - l1 - l3; 

}

void FitSize::compute(std::map<int, int>& sizeFit) {
//  std::map<unsigned, int> sizeFit;
  for (int i=0; i < planePointIndices.size(); i++) {
    if (planePointIndices[i].size() < 200) {
      sizeFit.insert(std::pair<int, int>(i, 0));
      continue;
    }
    float xmin = 100, xmax = -100, ymin = 100, ymax = -100, zmin = 100, zmax = -100;
    float l1, l2, l3;
//    std::cout << view.surfaces[i]->indices.size() << "\t";
    for (int j = 0; j < planePointIndices[i].size(); j++) {
      pcl::PointXYZRGBNormal& p = pcl_cloud->points[planePointIndices[i][j]];
      xmin = std::min(xmin, p.x);
      xmax = std::max(xmax, p.x);
      ymin = std::min(ymin, p.y);
      ymax = std::max(ymax, p.y);
      zmin = std::min(zmin, p.z);
      zmax = std::max(zmax, p.z);
    }
 //   std::cout << xmax - xmin << "\t" << ymax - ymin << "\t"  << zmax - zmin << "\n";
    sortLength(xmax - xmin, ymax - ymin, zmax - zmin, l1, l2, l3);

    if(l1 > 0.8 || l1 < 0.15) {
      sizeFit.insert(std::pair<unsigned, int>(i, 0));
    } else if (l1 / l2 > 3) {
      sizeFit.insert(std::pair<unsigned, int>(i, 0));
    } else {
      sizeFit.insert(std::pair<unsigned, int>(i, 1));
    }
    
  }  
}

int FitSize::fitCubeSize(std::vector<int> ind) {
  float xmin = 100, xmax = -100, ymin = 100, ymax = -100, zmin = 100, zmax = -100;
  float l1, l2, l3;	
  for (unsigned i = 0; i < ind.size(); i++) {
    for (unsigned j = 0; j < planePointIndices[ind[i]].size(); j++) {
      pcl::PointXYZRGBNormal& p = pcl_cloud->points[planePointIndices[ind[i]][j]];
      xmin = std::min(xmin, p.x);
      xmax = std::max(xmax, p.x);
      ymin = std::min(ymin, p.y);
      ymax = std::max(ymax, p.y);
      zmin = std::min(zmin, p.z);
      zmax = std::max(zmax, p.z);
    }

  }
  sortLength(xmax - xmin, ymax - ymin, zmax - zmin, l1, l2, l3);
  // std::cout << l1 << "  " << l2 << " " << " " << l3 << " " << l1*l2*l3 << std::endl;
  if (l1 < 0.4 || l2 < 0.3 || l3 < 0.2 || ymax- ymin > 0.6 || xmax - xmin > 0.8 || zmax - zmin > 0.6) {
  	return 0;
  } else {
  	return 1;
  }
}

std::vector<float> FitSize::get3DBox(std::vector<int> ind) {
  float xmin = 100, xmax = -100, ymin = 100, ymax = -100, zmin = 100, zmax = -100;
  float l1, l2, l3; 
  for (unsigned i = 0; i < ind.size(); i++) {
    for (unsigned j = 0; j < planePointIndices[ind[i]].size(); j++) {
      pcl::PointXYZRGBNormal& p = pcl_cloud->points[planePointIndices[ind[i]][j]];
      xmin = std::min(xmin, p.x);
      xmax = std::max(xmax, p.x);
      ymin = std::min(ymin, p.y);
      ymax = std::max(ymax, p.y);
      zmin = std::min(zmin, p.z);
      zmax = std::max(zmax, p.z);
    }

  }

  std::vector<float> result;
  result.push_back(xmin);
  result.push_back(xmax);
  result.push_back(ymin);
  result.push_back(ymax);
  result.push_back(zmin);
  result.push_back(zmax);
  return result;
}

bool FitSize::isHandleShape(const std::vector<int> & indices) {
  float xmin = 100, xmax = -100, ymin = 100, ymax = -100, zmin = 100, zmax = -100;
  float l1, l2, l3; 
  for (int i = 0; i < indices.size(); i++) {
    pcl::PointXYZRGBNormal& p = pcl_cloud->points[indices[i]];
    xmin = std::min(xmin, p.x);
    xmax = std::max(xmax, p.x);
    ymin = std::min(ymin, p.y);
    ymax = std::max(ymax, p.y);
    zmin = std::min(zmin, p.z);
    zmax = std::max(zmax, p.z);
  }

  double height = zmax-zmin;
  double width = std::sqrt((xmax-xmin)*(xmax-xmin) + (ymax-ymin)*(ymax-ymin));
  std::cout << height << " " << width << "  " << xmax-xmin << " " << ymax-ymin << std::endl;

  if (height / width > 2.5) {
    return true;
  } else {
    return false;
  }
 }



