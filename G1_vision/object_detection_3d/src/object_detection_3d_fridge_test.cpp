#include <object_detection_3d/object_detection_3d.h>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>

// Utilities
#include <namaris/utilities/filesystem.hpp>
#include <namaris/utilities/pcl_typedefs.hpp>


int main( int argc, char** argv )
{
  //----------------------------------------------------------------------------
  // Read data
  //----------------------------------------------------------------------------
  
  if (argc < 2)
  {
    std::cout << "You must provide a pointcloud filename" << std::endl;
    return -1;
  }
    
  std::cout << "Loading data..." << std::endl;  
  
  // Read cloud
  std::string cloudFilename = argv[1];
  pcl::PointCloud<PointNC>::Ptr sceneCloud (new pcl::PointCloud<PointNC>);
  if (pcl::io::loadPCDFile(cloudFilename, *sceneCloud))
  {
    std::cout << "Could not load pointcloud from file:" << std::endl;
    std::cout << cloudFilename << std::endl;
    return -1;
  }
  
  //----------------------------------------------------------------------------
  // Process
  //----------------------------------------------------------------------------

  // Start measuring time
  std::cout << "--------------------------------------------" << std::endl;
  double totalStart = pcl::getTime();
  
  // Segment fridge interior
  pcl::PointCloud<PointNC>::Ptr fridgeCloud (new pcl::PointCloud<PointNC>);
  g1::vision::Box fridgeBox;
  
  g1::vision::segmentFridgeInterior(sceneCloud, fridgeCloud, fridgeBox, true);

  
  // Report total time
  std::cout << "Total processing time: " << (pcl::getTime() - totalStart) << " seconds" << std::endl;
  
  return 0;
}
