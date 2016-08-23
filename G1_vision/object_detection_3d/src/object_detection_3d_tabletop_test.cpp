#include <object_detection_3d/object_detection_3d.h>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>
#include <pcl/features/normal_3d_omp.h>

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
  
  //----------------------------------------------------------------------------
  // Segment tabletop
  pcl::PointCloud<PointNC>::Ptr tabletopCloud (new pcl::PointCloud<PointNC>);
  pcl::PointCloud<PointNC>::Ptr tablePlaneHull (new pcl::PointCloud<PointNC>);
  Eigen::Vector4f tablePlaneCoefficients;
  
  g1::vision::segmentTable (sceneCloud, tabletopCloud, tablePlaneCoefficients, tablePlaneHull, false);
  std::cout << std::endl;
  
  //----------------------------------------------------------------------------
  // Segment objects
  utl::map::Map segments_euclidean, segments_smooth;
  g1::vision::segmentObjects (tabletopCloud, segments_euclidean, segments_smooth, false);
  std::cout << std::endl;

  //----------------------------------------------------------------------------
  // Recalculate normals

  pcl::NormalEstimationOMP<PointNC, PointNC> normalEstimator;
  pcl::search::KdTree<PointNC>::Ptr searchTree (new pcl::search::KdTree<PointNC>);
  normalEstimator.setSearchMethod (searchTree);
  normalEstimator.setKSearch(15);
  normalEstimator.setInputCloud(tabletopCloud);
  normalEstimator.compute(*tabletopCloud);
  
  //----------------------------------------------------------------------------
  // Detect rotational objects
  std::vector<g1::vision::Cylinder> rotationalObjectBoundingCylinders;
  std::vector<std::vector<int> > segmentsUsedForRotational;
  std::vector<sym::RotationalSymmetry> rotationalObjectSymmetries;
  std::vector<pcl::PointCloud<PointNC>::Ptr> rotationalObjectClouds;
  
  g1::vision::rotationalObjectDetection ( tabletopCloud,
                                          segments_euclidean,
                                          tablePlaneCoefficients,
                                          rotationalObjectSymmetries,
                                          rotationalObjectBoundingCylinders,
                                          segmentsUsedForRotational,
                                          false );
  
  // Get indices of points used for each rotationally symmetric object
  utl::map::Map rotationalObjectSegments (segmentsUsedForRotational.size());
  for (size_t objId = 0; objId < segmentsUsedForRotational.size(); objId++)
  {
    std::vector<int> indices;
    for (size_t segIdIt = 0; segIdIt < segmentsUsedForRotational[objId].size(); segIdIt++)
    {
      int segId = segmentsUsedForRotational[objId][segIdIt];
      indices.insert(indices.end(), segments_euclidean[segId].begin(), segments_euclidean[segId].end());
    }
    rotationalObjectSegments[objId] = indices;
  }
  
  // Detect handles
  std::vector<g1::vision::Box> rotationalObjectHandleBoxes;
  g1::vision::handleDetection ( tabletopCloud,
                                tablePlaneCoefficients,
                                rotationalObjectSegments,
                                rotationalObjectSymmetries,
                                rotationalObjectBoundingCylinders,
                                rotationalObjectHandleBoxes,
                                false  );
  
  //----------------------------------------------------------------------------
  // Fit boxes to non rotational segments

  // Get segments not used for rotational objects
  std::vector<int> nonRotationalSegmentIds;
  for (size_t segId = 0; segId < segments_euclidean.size(); segId++)
  {
    bool used = false;
    
    for (size_t rotObjId = 0; rotObjId < segmentsUsedForRotational.size(); rotObjId++)
      for (size_t rotObjSegIdIt = 0; rotObjSegIdIt < segmentsUsedForRotational[rotObjId].size(); rotObjSegIdIt++)
        if (std::find(segmentsUsedForRotational[rotObjId].begin(), segmentsUsedForRotational[rotObjId].end(), segId) != segmentsUsedForRotational[rotObjId].end())
          used = true;

    if (!used)
      nonRotationalSegmentIds.push_back(segId);
  }
    
  // Fit boxes
  std::vector<g1::vision::Box> objectBoundignBoxes;
  g1::vision::fitBoundingBoxes(tabletopCloud, segments_euclidean, tablePlaneCoefficients, nonRotationalSegmentIds, objectBoundignBoxes, false);
  
  //----------------------------------------------------------------------------
  // Visualize final output

  pcl::visualization::PCLVisualizer visualizer;
  visualizer.setCameraPosition (  2.5, 0.0, 2.0,   // camera position
                                  0.0, 0.0, 0.0,   // viewpoint
                                  0.0, 0.0, 1.0,   // normal
                                  0.0);            // viewport
  visualizer.setBackgroundColor (utl::pclvis::bgColor.r, utl::pclvis::bgColor.g, utl::pclvis::bgColor.b);
  
  
  // Add pointcloud
  utl::pclvis::showPointCloudColor<PointNC>(visualizer, tabletopCloud, "cloud");
  
  // Add bounding cylinders and handles
  for (size_t cylinderId = 0; cylinderId < rotationalObjectBoundingCylinders.size(); ++cylinderId)
  {
    utl::pclvis::Color color = utl::pclvis::getGlasbeyColor(cylinderId);
    g1::vision::showCylinder(visualizer, rotationalObjectBoundingCylinders[cylinderId], rotationalObjectBoundingCylinders[cylinderId].id_, color, 0.5);
    
    if (rotationalObjectHandleBoxes[cylinderId].id_ != "")
      g1::vision::showBox(visualizer, rotationalObjectHandleBoxes[cylinderId], rotationalObjectHandleBoxes[cylinderId].id_, color, 0.5);
  }
  
  // Add bounding boxes
  for (int boxId = 0; boxId < objectBoundignBoxes.size(); ++boxId)
  {
    utl::pclvis::Color color = utl::pclvis::getGlasbeyColor(objectBoundignBoxes.size() + boxId);
    g1::vision::showBox(visualizer, objectBoundignBoxes[boxId], objectBoundignBoxes[boxId].id_, color, 0.5);
  } 
  
  // Add robot frame
  visualizer.addCoordinateSystem(0.5);
  
  visualizer.spin();
  
  // Report total time
  std::cout << "Total processing time: " << (pcl::getTime() - totalStart) << " seconds" << std::endl;
  
  return 0;
}
