// Current package includes
#include "object_detection_3d/object_detection_3d.h"

// PCL
#include <pcl/common/time.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/io/pcd_io.h>

// Utilities
#include <namaris/utilities/filesystem.hpp>
#include <namaris/utilities/map.hpp>
#include <namaris/utilities/pointcloud.hpp>
#include <namaris/utilities/eigen.hpp>
#include <namaris/utilities/pcl_visualization.hpp>

// Algorithms
#include <namaris/algorithms/region_growing_normal_variation/region_growing_normal_variation.hpp>

////////////////////////////////////////////////////////////////////////////////
// Visualization

// State
struct VisState
{
  VisState ()
    : cloudDisplayState_ (CLOUD)
    , showNormals_(false)
    , showGravityVector_(false)
    , showTablePlane_ (false)
    , pointSize_(4.0)
    , updateDisplay_(true)
  {};
  
  enum CloudDisplay { CLOUD, PLANE_POINTS, PLANE_HULLS, TABLETOP_OBJECTS, TABLETOP_OBJECTS_REFINED };
  
  CloudDisplay cloudDisplayState_;
  bool showNormals_;
  bool showGravityVector_;
  bool showTablePlane_;
  float pointSize_;
  bool updateDisplay_;
};

// Callback
void keyboard_callback_ts (const pcl::visualization::KeyboardEvent &event, void *cookie)
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
      visState->cloudDisplayState_ = VisState::PLANE_POINTS;
    else if (key == "KP_3")
      visState->cloudDisplayState_ = VisState::PLANE_HULLS;
    else if (key == "KP_4")
      visState->cloudDisplayState_ = VisState::TABLETOP_OBJECTS;
    else if (key == "KP_5")
      visState->cloudDisplayState_ = VisState::TABLETOP_OBJECTS_REFINED;
    
    // Point size
    else if (key == "KP_Add")
      visState->pointSize_ += 1.0;
    else if (key == "KP_Subtract")
      visState->pointSize_ = std::max(0.0, visState->pointSize_ - 1.0);
    
    else if ((key == "n") || (key == "N"))
      visState->showNormals_ = !visState->showNormals_;
    else if ((key == "M") || (key == "m"))
      visState->showGravityVector_ = !visState->showGravityVector_;
    else if ((key == "comma") || (key == "less"))
      visState->showTablePlane_ = !visState->showTablePlane_;
    else
      visState->updateDisplay_ = false;
  }
}

////////////////////////////////////////////////////////////////////////////////
bool g1::vision::tablePlaneUnaryConditionFunction (const PointNC& p, const Eigen::Vector3f& gravity_vector, const float gravity_angle_thresh)
{
  Eigen::Vector3f normal = p.getNormalVector3fMap();
  return normal.dot(-gravity_vector) > std::cos(gravity_angle_thresh) && p.z > -0.4f;
}


////////////////////////////////////////////////////////////////////////////////
// Run
bool g1::vision::segmentTable ( const pcl::PointCloud<PointNC>::ConstPtr &scene_cloud,
                                pcl::PointCloud<PointNC>::Ptr &tabletop_cloud,
                                Eigen::Vector4f &table_plane_coefficients,
                                pcl::PointCloud<PointNC>::Ptr &table_plane_hull,
                                const bool visualize
                              )
{
  //----------------------------------------------------------------------------
  // Measure total processing time
  //----------------------------------------------------------------------------

  double totalStart = pcl::getTime();
  
  //----------------------------------------------------------------------------
  // Table plane point detection
  //----------------------------------------------------------------------------
  
  std::cout << "Segmenting table planes..." << std::endl;
  double start = pcl::getTime();
  
  float voxelSize = 0.005;
  utl::map::Map planePointIndices;
  int numPlanesDected = 0;
  
  // Parameters
  float gravityVectorAngleThreshold = pcl::deg2rad(10.0);         // Maximum allowed difference between a table plane point normal and gravity vector
  float normalVariationThreshold    = pcl::deg2rad(15.0) * 100;   // Maximum allowed normal difference between adjacent points (in rad/m)
  int minTablePlanePoints = 1000;
    
  // Create unary condition function
  Eigen::Vector3f gravityVector (0.0f, 0.0f, -1.0f);  
  boost::function<bool (const PointNC&)> tablePlaneUnaryFunction =
    boost::bind (&g1::vision::tablePlaneUnaryConditionFunction, _1, gravityVector, gravityVectorAngleThreshold);

  // Create region growing segmentation object
  alg::RegionGrowingNormalVariation<PointNC> rg;
  rg.setInputCloud (scene_cloud);
  rg.setConsistentNormals (true);
  rg.setNumberOfNeighbors (5);
  rg.setSearchRadius (voxelSize * std::sqrt (2));
  rg.setNormalVariationThreshold (normalVariationThreshold);
  rg.setMinValidBinaryNeighborsFraction (0.8);
  rg.setUnaryConditionFunction (tablePlaneUnaryFunction);
  rg.setMinSegmentSize (minTablePlanePoints);
  rg.segment (planePointIndices);
  
  numPlanesDected = planePointIndices.size ();
  if (numPlanesDected == 0)
  {
    std::cout << "  Planar points could not be found. Something is wrong!" << std::endl;
    return false;
  }
  
  std::cout << "  " << numPlanesDected << " plane segments" << std::endl;
  std::cout << "  " << (pcl::getTime() - start) << " seconds" << std::endl;
  
  //----------------------------------------------------------------------------
  // Filter table points based on minimum height
  //----------------------------------------------------------------------------
  
  //----------------------------------------------------------------------------
  // Fit convex hulls to remaining table planes
  //----------------------------------------------------------------------------
  
  std::cout << "Fitting convex hulls to plane points..." << std::endl;
  start = pcl::getTime();
  
  std::vector<pcl::PointCloud<PointNC>::Ptr> planeHulls (planePointIndices.size());
  std::vector<float> planeAreas                         (planePointIndices.size());
  std::vector<Eigen::Vector4f> planeCoefficients        (planePointIndices.size());
  
  utl::cloud::ConvexHull2D<PointNC> chull2d;
  chull2d.setInputCloud (scene_cloud);
  
  for (size_t planeId = 0; planeId < planePointIndices.size (); planeId++)
  {
    planeHulls[planeId].reset (new pcl::PointCloud<PointNC>);
    chull2d.setIndices (boost::make_shared<std::vector<int> > (planePointIndices[planeId]) );
    chull2d.reconstruct (*planeHulls[planeId]);
    planeAreas[planeId] = chull2d.getTotalArea ();
    planeCoefficients[planeId] = chull2d.getPlaneCoefficients ();
  }
  
  std::cout << "  " << (pcl::getTime() - start) << " seconds" << std::endl;
  
  //----------------------------------------------------------------------------
  // Select table plane with the largest convex hull area
  //----------------------------------------------------------------------------
  
  std::cout << "Selecting the hull with the largest area..." << std::endl;
  start = pcl::getTime();
  
  float maxArea;
  std::vector<int> maxAreaIds;
  utl::stdvec::vectorMaxLoc (planeAreas, maxArea, maxAreaIds);

  int tablePlaneId = maxAreaIds[0];
  std::vector<int> tablePlanePointIndices = planePointIndices[tablePlaneId];
  table_plane_hull = planeHulls[tablePlaneId];
  table_plane_coefficients = planeCoefficients[tablePlaneId];
  
  Eigen::Vector3f planePoint, planeNormal;
  utl::geom::planeCoefficientsToPointNormal(table_plane_coefficients, planePoint, planeNormal);
  if (planeNormal[2] < 0.0f)
    table_plane_coefficients *= -1;
  
  std::cout << "  " << (pcl::getTime() - start) << " seconds" << std::endl;
  
  //----------------------------------------------------------------------------
  // Extract points above the table plane
  //----------------------------------------------------------------------------
  
  std::cout << "Extracting points above the table" << std::endl;
  start = pcl::getTime();
  pcl::PointCloud<PointNC>::Ptr objectCloud (new pcl::PointCloud<PointNC>);
  
  float minHeight = 0.02;
  float maxHeight = 0.3;
  
  // Scale table hull to get rid of points at the very boundary of the table.
  utl::cloud::scalePointCloud(*table_plane_hull, 0.9, *table_plane_hull);      
  pcl::PointIndices objectPointIndices;
  pcl::ExtractPolygonalPrismData<PointNC> pprism;
  pprism.setInputCloud (scene_cloud);  
  pprism.setInputPlanarHull (table_plane_hull);
  pprism.setViewPoint (0.0f, 0.0f, 0.0f);
  pprism.setHeightLimits (minHeight, maxHeight);
  pprism.segment (objectPointIndices);  

  // Remove points that belong to the plane
  objectPointIndices.indices = utl::stdvec::vectorDifference (objectPointIndices.indices, tablePlanePointIndices);
  pcl::copyPointCloud<PointNC> (*scene_cloud, objectPointIndices, *objectCloud);
    
  std::cout << "  " << (pcl::getTime() - start) << " seconds" << std::endl;

  //----------------------------------------------------------------------------
  // Remove small clusters
  //----------------------------------------------------------------------------
   
  std::cout << "Removing small clusters" << std::endl;
  start = pcl::getTime();
  
  pcl::PointCloud<PointNC>::Ptr objectCloudRefined (new pcl::PointCloud<PointNC>);
  
  utl::graph::Graph connectivityGraph;
  utl::cloud::getCloudConnectivityRadius<PointNC> (objectCloud, connectivityGraph, voxelSize * sqrt(2));
  
  int minCCSize = 50;
  utl::map::Map connectedComponents = utl::graph::getConnectedComponents (connectivityGraph, minCCSize);
  
  std::vector<int> objectPointIndicesFiltered;
  for (size_t ccId = 0; ccId < connectedComponents.size(); ccId++)
    objectPointIndicesFiltered.insert (objectPointIndicesFiltered.end (), connectedComponents[ccId].begin (), connectedComponents[ccId].end ());
  
  pcl::copyPointCloud<PointNC> (*objectCloud, objectPointIndicesFiltered, *objectCloudRefined);
  
  std::cout << "  " << (pcl::getTime() - start) << " seconds" << std::endl;
  
  //----------------------------------------------------------------------------
  // Set final result
  //----------------------------------------------------------------------------

  tabletop_cloud  = objectCloudRefined;
  
//   std::cout << "Done!" << std::endl;
//   std::cout << "  Total processing time: " << (pcl::getTime() - totalStart) << " seconds" << std::endl;
  
  //----------------------------------------------------------------------------
  // Visualize
  //----------------------------------------------------------------------------

  // If we are not visualizing - we are done
  if (!visualize)
    return true;
  
  // Otherwise visualize
  VisState visState;
  pcl::visualization::PCLVisualizer visualizer;
  visualizer.setCameraPosition (  2.5, 0.0, 2.0,   // camera position
                                  0.0, 0.0, 0.0,   // viewpoint
                                  0.0, 0.0, 1.0,   // normal
                                  0.0);            // viewport
  visualizer.setBackgroundColor (utl::pclvis::bgColor.r, utl::pclvis::bgColor.g, utl::pclvis::bgColor.b);
  visualizer.registerKeyboardCallback(keyboard_callback_ts, (void*)(&visState));  
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
        utl::pclvis::showPointCloudColor<PointNC>(visualizer, scene_cloud, "cloud", visState.pointSize_);

        if (visState.showNormals_)
          utl::pclvis::showNormalCloud<PointNC>(visualizer, scene_cloud, 100, 0.02, "normals", visState.pointSize_, utl::pclvis::green);
        
        visualizer.addText("Cloud", 15, 100, 24, 1.0, 1.0, 1.0);
      }
      
      else if (visState.cloudDisplayState_ == VisState::PLANE_POINTS)
      {
        utl::pclvis::showSegmentation<PointNC>(visualizer, scene_cloud, planePointIndices, "planes", visState.pointSize_);
        
        if (visState.showNormals_)
        {
          for (size_t planeId = 0; planeId < planePointIndices.size(); planeId++)
            utl::pclvis::showNormalCloud<PointNC>(visualizer, scene_cloud, planePointIndices[planeId], 0.05, "normals_" + std::to_string(planeId), 1.0, utl::pclvis::green);
        }
        
        visualizer.addText(std::to_string(planePointIndices.size()) + " planes", 15, 100, 24, 1.0, 1.0, 1.0);
      }
      
      else if (visState.cloudDisplayState_ == VisState::PLANE_HULLS)
      {
        for (size_t planeId = 0; planeId < planePointIndices.size(); planeId++)
        {
          std::string id = "table_polygon_" + std::to_string(planeId);
          utl::pclvis::Color curColor = utl::pclvis::getGlasbeyColor(planeId);
          visualizer.addPolygon<PointNC>(planeHulls[planeId], curColor.r, curColor.g, curColor.b, id);
          utl::pclvis::setLineRenderProps(visualizer, id, 4.0);
        }
        
        visualizer.addText(std::to_string(planeHulls.size()) + " planes", 15, 100, 24, 1.0, 1.0, 1.0);
      }
      
      else if (visState.cloudDisplayState_ == VisState::TABLETOP_OBJECTS || visState.cloudDisplayState_ == VisState::TABLETOP_OBJECTS_REFINED)
      {
        pcl::PointCloud<PointNC>::Ptr displayCloud (new pcl::PointCloud<PointNC>);
        if (visState.cloudDisplayState_ == VisState::TABLETOP_OBJECTS)
          displayCloud = objectCloud;
        else if (visState.cloudDisplayState_ == VisState::TABLETOP_OBJECTS_REFINED)
          displayCloud = objectCloudRefined;
        
        visualizer.addPolygon<PointNC> (table_plane_hull, "table_plane");
        utl::pclvis::showPointCloudColor<PointNC>(visualizer, displayCloud, "object_points", visState.pointSize_);
        
        if (visState.showNormals_)
          utl::pclvis::showNormalCloud<PointNC>(visualizer, displayCloud, 5, 0.02, "object_point_normals", 2, utl::pclvis::green);
        
        visualizer.addText("Final table plane and objects", 15, 100, 24, 1.0, 1.0, 1.0);
      }
        
      if (visState.showGravityVector_)
        visualizer.addCoordinateSystem(0.3);
      
      if (visState.showTablePlane_)
      {
        std::string id = "table_plane_filled";
        visualizer.addPolygon<PointNC> (table_plane_hull, id);
        visualizer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, id);
        utl::pclvis::setShapeRenderProps (visualizer, id, utl::pclvis::green);
        
        Eigen::Vector3f point, normal;
        utl::geom::planeCoefficientsToPointNormal(table_plane_coefficients, point, normal);
        Point p1, p2;
        p1.getVector3fMap() = point;
        p2.getVector3fMap() = point+normal;
        visualizer.addLine(p1, p2, "line");
      }
    }
    
    // Spin once
    visualizer.spinOnce();
    boost::this_thread::sleep (boost::posix_time::milliseconds (50));
  }
  
  return true;
}