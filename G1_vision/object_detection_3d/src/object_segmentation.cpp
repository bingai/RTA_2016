// Current package includes
#include "object_detection_3d/object_detection_3d.h"

// PCL
#include <pcl/common/time.h>

// Utilities
#include <namaris/utilities/map.hpp>
#include <namaris/utilities/pcl_visualization.hpp>

// Algorithms
#include <namaris/algorithms/region_growing_normal_variation/region_growing_normal_variation.hpp>
#include <namaris/algorithms/region_growing_smoothness/region_growing_smoothness.hpp>

// Symmetry includes
#include <symmetry/rotational_symmetry_detection.hpp>

////////////////////////////////////////////////////////////////////////////////
// Visualization

// State
struct VisState
{
  VisState ()
    : cloudDisplayState_ (CLOUD)
    , showNormals_(false)
    , updateDisplay_(true)
    , pointSize_(4.0)
  {};
  
  enum CloudDisplay { CLOUD, SEGMENTATION_EUCLIDEAN, SEGMENTATION_SMOOTH };
  
  CloudDisplay cloudDisplayState_;
  bool showNormals_;
  bool updateDisplay_;
  float pointSize_;
};

// Callback
void keyboard_callback_os (const pcl::visualization::KeyboardEvent &event, void *cookie)
{
  VisState* visState = reinterpret_cast<VisState*> (cookie);
  
  if (event.keyUp ())
  {    
    std::string key = event.getKeySym ();
//     std::cout << key << " key pressed!\n";
    
    visState->updateDisplay_ = true;
    
    if (key == "KP_1")
      visState->cloudDisplayState_ = VisState::CLOUD;
    else if (key == "KP_2")
      visState->cloudDisplayState_ = VisState::SEGMENTATION_EUCLIDEAN;
    else if (key == "KP_3")
      visState->cloudDisplayState_ = VisState::SEGMENTATION_SMOOTH;
    
    // Point size
    else if (key == "KP_Add")
      visState->pointSize_ += 1.0;
    else if (key == "KP_Subtract")
      visState->pointSize_ = std::max(0.0, visState->pointSize_ - 1.0);
    
    else if ((key == "n") || (key == "N"))
      visState->showNormals_ = !visState->showNormals_;

    else
      visState->updateDisplay_ = false;
  }
}

////////////////////////////////////////////////////////////////////////////////
bool euclideanClusteringBinaryConditionFunction (const PointNC& p1, const PointNC& p2, const float dist_squared)
{
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Run
bool g1::vision::segmentObjects ( const pcl::PointCloud<PointNC>::ConstPtr &tabletop_cloud,
                                  utl::map::Map &segments_euclidean,
                                  utl::map::Map &segments_smooth,
                                  const bool visualize
                              )
{
  segments_euclidean.resize(0);
  segments_smooth.resize(0);
  
  //----------------------------------------------------------------------------
  // Segmentation parameters
  //----------------------------------------------------------------------------

  float searchRadius = 0.005 * std::sqrt (3.0f);      // Scene will be downsampled to this resolution
  float minSegmentSize = 80;                            // Minimum size of segment used as symmetry support (in metres)
  float normalVariation    = 12.0;       // Normal variation thresholds for support segments
  float validBinFraction   = 0.7;

  //----------------------------------------------------------------------------
  // Find euclidean segments in the cloud
  //----------------------------------------------------------------------------
  
  std::cout << "Calculating euclidean segments..." << std::endl;
  
  double start = pcl::getTime();
  double totalStart = start;

  // Create handle clustering object
  boost::function<bool (const PointNC&, const PointNC&, const float)> euclideanClusteringBinaryFunction =
    boost::bind (&euclideanClusteringBinaryConditionFunction, _1, _2, _3);
  
  // Prepare segmentation object
  alg::RegionGrowing<PointNC> rg_euclidean;
  rg_euclidean.setBinaryConditionFunction(euclideanClusteringBinaryFunction);
  rg_euclidean.setInputCloud (tabletop_cloud);
  rg_euclidean.setSearchRadius (searchRadius);
  rg_euclidean.setMinSegmentSize(minSegmentSize);
  rg_euclidean.setMinValidBinaryNeighborsFraction(0.0f);
  rg_euclidean.segment(segments_euclidean);
  
  std::cout << "  " << segments_euclidean.size() << " segments." << std::endl;
  std::cout << "  " << (pcl::getTime() - start) << " seconds." << std::endl;
  
  //----------------------------------------------------------------------------
  // Find smooth segments in the cloud
  //----------------------------------------------------------------------------
  
  std::cout << "Calculating smooth segments..." << std::endl;
  
  start = pcl::getTime();

  // Prepare segmentation object
  alg::RegionGrowingSmoothness<PointNC> rg_smooth;
  rg_smooth.setInputCloud (tabletop_cloud);
  rg_smooth.setConsistentNormals(true);
  rg_smooth.setSearchRadius (searchRadius);
  rg_smooth.setMinSegmentSize(minSegmentSize);
  rg_smooth.setNormalAngleThreshold (pcl::deg2rad (normalVariation));
  rg_smooth.setMinValidBinaryNeighborsFraction(validBinFraction);
  rg_smooth.segment(segments_smooth);      
  
  std::cout << "  " << segments_smooth.size() << " segments." << std::endl;
  std::cout << "  " << (pcl::getTime() - start) << " seconds." << std::endl;
  
//   pcl::io::savePCDFileBinary("../cloud.pcd", *segments[12]);

  //----------------------------------------------------------------------------
  // Visualize
  //----------------------------------------------------------------------------

  if (!visualize)
    return 0;
    
  VisState visState;
  pcl::visualization::PCLVisualizer visualizer;
  visualizer.setCameraPosition (  2.5, 0.0, 2.0,   // camera position
                                  0.0, 0.0, 0.0,   // viewpoint
                                  0.0, 0.0, 1.0,   // normal
                                  0.0);            // viewport
  visualizer.setBackgroundColor (utl::pclvis::bgColor.r, utl::pclvis::bgColor.g, utl::pclvis::bgColor.b);
  visualizer.registerKeyboardCallback(keyboard_callback_os, (void*)(&visState));  
  visState.updateDisplay_ = true;
    
  while (!visualizer.wasStopped())
  {
    // Update display if needed
    if (visState.updateDisplay_)
    {
      // First remove everything
      visualizer.removeAllPointClouds();
      visualizer.removeAllShapes();
      visualizer.removeAllCoordinateSystems();
      visState.updateDisplay_ = false;
      
      // Then add things as required
      if (visState.cloudDisplayState_ == VisState::CLOUD)
      {
        utl::pclvis::showPointCloudColor<PointNC>(visualizer, tabletop_cloud, "cloud", visState.pointSize_);

        if (visState.showNormals_)
          utl::pclvis::showNormalCloud<PointNC>(visualizer, tabletop_cloud, 10, 0.02, "normals", visState.pointSize_, utl::pclvis::green);
        
        visualizer.addText("Original cloud", 0, 100, 24, 1.0, 1.0, 1.0);
      }
      
      else if (visState.cloudDisplayState_ == VisState::SEGMENTATION_EUCLIDEAN)
      {              
        utl::pclvis::showPointCloudColor<PointNC>(visualizer, tabletop_cloud, "cloud", visState.pointSize_, 0.3);
        utl::pclvis::showSegmentation<PointNC>(visualizer, tabletop_cloud, segments_euclidean, "segment", visState.pointSize_);
        
        if (visState.showNormals_)
          utl::pclvis::showNormalCloud<PointNC>(visualizer, tabletop_cloud, 10, 0.02, "normals", visState.pointSize_, utl::pclvis::green);
                
        visualizer.addText("Segmentation euclidean", 0, 150, 24, 1.0, 1.0, 1.0);
      }

      else if (visState.cloudDisplayState_ == VisState::SEGMENTATION_SMOOTH)
      {              
        utl::pclvis::showPointCloudColor<PointNC>(visualizer, tabletop_cloud, "cloud", visState.pointSize_, 0.3);
        utl::pclvis::showSegmentation<PointNC>(visualizer, tabletop_cloud, segments_smooth, "segment", visState.pointSize_);
        
        if (visState.showNormals_)
          utl::pclvis::showNormalCloud<PointNC>(visualizer, tabletop_cloud, 10, 0.02, "normals", visState.pointSize_, utl::pclvis::green);
                
        visualizer.addText("Segmentation smooth", 0, 150, 24, 1.0, 1.0, 1.0);
      }
    }
    
    // Spin once
    visualizer.spinOnce();
    boost::this_thread::sleep (boost::posix_time::milliseconds (50));
  }
  
  return true;
}