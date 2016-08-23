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
// Visualize

// State
struct VisState
{
  VisState ()
    : cloudDisplayState_ (CLOUD)
    , showNormals_(false)
    , showTablePlane_ (false)
    , showReconstructedCloud_(false)
    , showProfileCurve_ (false)
    , showBoundingCylinder_ (true)
    , updateDisplay_(true)
    , segIterator_(0)
    , handleSegIterator_(0)
    , showAll_(false)
    , pointSize_(4.0)
  {};
  
  enum CloudDisplay { CLOUD, SYMMETRIES, HANDLE_POINTS, HANDLE_SEGMENTS, HANDLE_SEGMENTS_FILTERED, HANDLE_SEGMENTS_FINAL};
  
  CloudDisplay cloudDisplayState_;
  bool showNormals_;
  bool showTablePlane_;
  bool showReconstructedCloud_;
  bool showProfileCurve_;
  bool showBoundingCylinder_;
  bool updateDisplay_;
  int segIterator_;
  int handleSegIterator_;
  bool showAll_;
  float pointSize_;
};

// Callback
void keyboard_callback_hd (const pcl::visualization::KeyboardEvent &event, void *cookie)
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
      visState->cloudDisplayState_ = VisState::SYMMETRIES;
    else if (key == "KP_3")
      visState->cloudDisplayState_ = VisState::HANDLE_POINTS;    
    else if (key == "KP_4")
      visState->cloudDisplayState_ = VisState::HANDLE_SEGMENTS;
    else if (key == "KP_5")
      visState->cloudDisplayState_ = VisState::HANDLE_SEGMENTS_FILTERED;
    else if (key == "KP_6")
      visState->cloudDisplayState_ = VisState::HANDLE_SEGMENTS_FINAL;
    
    else if (key == "Left")
      visState->segIterator_--;
    else if (key == "Right")
      visState->segIterator_++;
    
    else if (key == "Up")
      visState->handleSegIterator_++;
    else if (key == "Down")
      visState->handleSegIterator_--;
    
    // Point size
    else if (key == "KP_Add")
      visState->pointSize_ += 1.0;
    else if (key == "KP_Subtract")
      visState->pointSize_ = std::max(0.0, visState->pointSize_ - 1.0);
    
    else if ((key == "n") || (key == "N"))
      visState->showNormals_ = !visState->showNormals_;
    else if ((key == "m") || (key == "M"))
      visState->showProfileCurve_ = !visState->showProfileCurve_;
    else if ((key == "comma") || (key == "less"))
      visState->showBoundingCylinder_ = !visState->showBoundingCylinder_;
    else if ((key == "Shift_L") || (key == "comma"))
      visState->showReconstructedCloud_ = !visState->showReconstructedCloud_;
    else if ((key == "KP_0") || (key == "insert"))
      visState->showTablePlane_ = !visState->showTablePlane_;
    else if ((key == "KP_Decimal") || (key == "KP_Delete"))
      visState->showAll_ = !visState->showAll_;
    

    else
      visState->updateDisplay_ = false;
  }
}

////////////////////////////////////////////////////////////////////////////////
bool handleDetectionBinaryConditionFunction (const PointNC& p1, const PointNC& p2, const float dist_squared, const float max_dist_squared)
{
  return (dist_squared <= max_dist_squared);
}


////////////////////////////////////////////////////////////////////////////////
// Run
bool g1::vision::handleDetection  ( const pcl::PointCloud<PointNC>::ConstPtr &tabletop_cloud,
                                    const Eigen::Vector4f &table_plane_coefficients,
                                    const utl::map::Map &object_segments,
                                    const std::vector<sym::RotationalSymmetry> &object_symmetries,
                                    const std::vector<Cylinder> &object_bounding_cylinders,
                                    std::vector<Box> &object_handle_bounding_boxes,
                                    const bool visualize
                                  )
{
  // Check that segments are non-empty

  //----------------------------------------------------------------------------
  // Parameters
  //----------------------------------------------------------------------------
  
  float maxCoverageAngle = pcl::deg2rad(45.0f);
  float minHandleSizeRatio = 0.5;
  float minHandlePoints = 10;
  
  //----------------------------------------------------------------------------
  // Prepare necessary variables
  //----------------------------------------------------------------------------
  
  Eigen::Vector3f planePoint, planeNormal;
  utl::geom::planeCoefficientsToPointNormal<float> (table_plane_coefficients, planePoint, planeNormal);
//   planeNormal = Eigen::Vector3f::UnitZ();
  
  PointNC centroid;
  pcl::computeCentroid (*tabletop_cloud, centroid);
  planePoint = utl::geom::projectPointToPlane<float> (centroid.getVector3fMap (), planePoint, planeNormal);
    
  //----------------------------------------------------------------------------
  // Detect mug handles
  //----------------------------------------------------------------------------
  
  std::cout << "Detecting mug handles..." << std::endl;
  double start = pcl::getTime();
    
  // Variables
  std::vector<pcl::PointCloud<PointNC>::Ptr> handleClouds (object_symmetries.size());
  std::vector<std::vector<pcl::PointCloud<PointNC>::Ptr> > handleSegmentClouds (object_symmetries.size());
  std::vector<std::pair<int, std::vector<int> > > handleSegmentIds;
  std::vector<std::pair<int, std::vector<int> > > handleSegmentFilteredIds;
  std::vector<std::pair<int, std::vector<int> > > handleSegmentFinalIds;
  std::vector<std::vector<float> > coverageAngles (object_symmetries.size());
  std::vector<std::vector<float> > handleSizeRatios (object_symmetries.size());
  std::vector<std::vector<g1::vision::Box> > handleBoxes (object_symmetries.size());
    
  // Create handle clustering object
  boost::function<bool (const PointNC&, const PointNC&, const float)> handleDetectionBinaryFunction =
    boost::bind (&handleDetectionBinaryConditionFunction, _1, _2, _3, std::pow(0.01f, 2));
    
  alg::RegionGrowing<PointNC> rgHandle;
  rgHandle.setBinaryConditionFunction(handleDetectionBinaryFunction);
  rgHandle.setMinValidBinaryNeighborsFraction(0.5);
  rgHandle.setSearchRadius(0.01);
  rgHandle.setInputCloud(tabletop_cloud);
  rgHandle.setMinSegmentSize(minHandlePoints);
  
  for (size_t objId = 0; objId < object_symmetries.size(); objId++)
  {
    //----------------------------------------------------------------------------
    // Get all points outside that belonf to the initial object segment but are
    // the bounding cylinder
    
    std::vector<int> handlePointIndices;
    float maxRadius = object_bounding_cylinders[objId].radius_;
    
    for (size_t pointIdIt = 0; pointIdIt < object_segments[objId].size(); pointIdIt++)
    {
      int pointId = object_segments[objId][pointIdIt];
      Eigen::Vector3f point = tabletop_cloud->points[pointId].getVector3fMap();
      float radialDistance = object_symmetries[objId].pointDistance(point);
      if (radialDistance >= maxRadius)
        handlePointIndices.push_back(pointId);
    }
    
    handleClouds[objId].reset (new pcl::PointCloud<PointNC>);
    pcl::copyPointCloud<PointNC>(*tabletop_cloud, handlePointIndices, *handleClouds[objId]);
    
    //--------------------------------------------------------------------------
    // Cluster handle segments
    rgHandle.setIndices(boost::make_shared<std::vector<int> >(handlePointIndices));
    utl::map::Map handleSegments;
    rgHandle.segment(handleSegments);
    
    std::vector<int> curHandleSegmentIds;
    handleSegmentClouds[objId].resize(handleSegments.size());
    for (size_t handleSegId = 0; handleSegId < handleSegments.size(); handleSegId++)
    {
      handleSegmentClouds[objId][handleSegId].reset (new pcl::PointCloud<PointNC>);
      pcl::copyPointCloud<PointNC>(*tabletop_cloud, handleSegments[handleSegId], *handleSegmentClouds[objId][handleSegId]);
      curHandleSegmentIds.push_back(handleSegId);
    }
    
    if (curHandleSegmentIds.size() > 0)
      handleSegmentIds.push_back(std::pair<int, std::vector<int> >(objId, curHandleSegmentIds));
    
    //--------------------------------------------------------------------------
    // Fit boxes to handle segments
    handleBoxes[objId].resize(handleSegmentClouds[objId].size());
        
    Eigen::Vector3f cylinderDirection  = object_bounding_cylinders[objId].pose_.linear().col(3);
    Eigen::Vector3f cylinderOrigin = object_bounding_cylinders[objId].pose_.translation();
    Eigen::Vector3f cylinderOriginProjected = utl::geom::projectPointToPlane<float>(cylinderOrigin, table_plane_coefficients);
    float cylinderHeight = object_bounding_cylinders[objId].height_;
    
    for (size_t handleSegId = 0; handleSegId < handleSegmentClouds[objId].size(); handleSegId++)
    {
      g1::vision::Box box;
      
      // Get centroid
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*handleSegmentClouds[objId][handleSegId], centroid);

      //  Unit vector in the direction of centroid
      Eigen::Vector3f referenceVector = (centroid.head(3) - object_symmetries[objId].projectPoint(centroid.head(3))).normalized();
      
      // Construct a coordinate frame
      Eigen::Vector3f cX, cY, cZ;
      cZ = planeNormal;
      cX = referenceVector;
      cY = cZ.cross(cX);      

      // Get maximum and minimum distance along the x axis and miaximum distance along the y axis
      float maxDistX = 0;
      float minDistX = std::numeric_limits<int>::max();      
      float maxDistY = 0;
      
      for (size_t pointId = 0; pointId < handleSegmentClouds[objId][handleSegId]->size(); pointId++)
      {
        Eigen::Vector3f pointProjected = utl::geom::projectPointToPlane<float>(handleSegmentClouds[objId][handleSegId]->points[pointId].getVector3fMap(), table_plane_coefficients);
        maxDistX = std::max(maxDistX, utl::geom::pointToLineDistance<float>(pointProjected, cylinderOriginProjected, cylinderOriginProjected + cY));
        minDistX = std::min(minDistX, utl::geom::pointToLineDistance<float>(pointProjected, cylinderOriginProjected, cylinderOriginProjected + cY));
        maxDistY = std::max(maxDistY, utl::geom::pointToLineDistance<float>(pointProjected, cylinderOriginProjected, cylinderOriginProjected + cX));
      }
      
      // Construct box
//       box.pose_.translation() = cylinderOrigin + referenceVector * (maxDistX + minDistX) / 2 + cylinderHeight / 2 * planeNormal;
      box.pose_.translation() = cylinderOrigin + referenceVector * (maxDistX + minDistX) / 2;
      box.pose_.linear().col(0) = cX;
      box.pose_.linear().col(1) = cY;
      box.pose_.linear().col(2) = cZ;
      box.size_ = Eigen::Vector3f(maxDistX - minDistX, maxDistY*2, cylinderHeight);      
      box.id_ = "handle_" + object_bounding_cylinders[objId].id_;
      
      handleBoxes[objId][handleSegId] = box;
    }    
    
    //--------------------------------------------------------------------------
    // Compute fitness properties of handle segments
    coverageAngles[objId].resize(handleSegmentClouds[objId].size());
    handleSizeRatios[objId].resize(handleSegmentClouds[objId].size());
    
    for (size_t handleSegId = 0; handleSegId < handleSegmentClouds[objId].size(); handleSegId++)
    {
      coverageAngles[objId][handleSegId] = sym::cloudRotationalSymmetryCoverageAngle<PointNC>(object_symmetries[objId], *handleSegmentClouds[objId][handleSegId]);
      handleSizeRatios[objId][handleSegId] = handleBoxes[objId][handleSegId].size_(0) / handleBoxes[objId][handleSegId].size_(1);
    }
    
    //--------------------------------------------------------------------------
    // Filter handles based on fitness properties
    std::vector<int> curHandleSegmentFilteredIds;
    for (size_t handleSegId = 0; handleSegId < handleSegmentClouds[objId].size(); handleSegId++)
    {
      if (coverageAngles[objId][handleSegId] < maxCoverageAngle && handleSizeRatios[objId][handleSegId] > minHandleSizeRatio)
        curHandleSegmentFilteredIds.push_back(handleSegId);      
    }
    
    if (curHandleSegmentFilteredIds.size() != 0)
      handleSegmentFilteredIds.push_back(std::pair<int, std::vector<int> >(objId, curHandleSegmentFilteredIds));

    //--------------------------------------------------------------------------
    // Select final handles. If there are more than one handle for an object,
    // select the one that has the largest support
    std::vector<int> curHandleSegmentFinalIds;
    
    if (curHandleSegmentFilteredIds.size() == 1)
      curHandleSegmentFinalIds = curHandleSegmentFilteredIds;
    else if (curHandleSegmentFilteredIds.size() > 1)
    {
      int maxSupport = 0;
      int maxSupprtId = -1;
      
      for (size_t handleSegIdIt = 0; handleSegIdIt < curHandleSegmentFilteredIds.size(); handleSegIdIt++)
      {
        int handleSegId = curHandleSegmentFilteredIds[handleSegIdIt];
        int curSupport = handleSegmentClouds[objId][handleSegId]->size();
        if (curSupport > maxSupport)
        {
          maxSupport = curSupport;
          maxSupprtId = handleSegId;
        }
      }
      
      curHandleSegmentFinalIds.push_back(maxSupprtId);
      std::cout << "[handleDetection] detected more than one handle for an object. Seleted the handle with largest support as the correct handle." << std::cout;
    }

    if (curHandleSegmentFinalIds.size() != 0)
      handleSegmentFinalIds.push_back(std::pair<int, std::vector<int> >(objId, curHandleSegmentFinalIds));
  }
  
//   std::cout << "  " << segmentIdsMerged.size() << " merged segments." << std::endl;
  std::cout << "  " << (pcl::getTime() - start) << " seconds." << std::endl;
    
  //----------------------------------------------------------------------------
  // Construct output
  //----------------------------------------------------------------------------

  object_handle_bounding_boxes.resize(object_segments.size());
  
  for (size_t objIdIt = 0; objIdIt < handleSegmentFilteredIds.size(); objIdIt++)
  {
    int objId = handleSegmentFilteredIds[objIdIt].first;
    int handleSegId = handleSegmentFinalIds[objIdIt].second[0];
    object_handle_bounding_boxes[objId] = handleBoxes[objId][handleSegId];    
  }
      
  //----------------------------------------------------------------------------
  // Visualize
  //----------------------------------------------------------------------------

  if (!visualize)
    return 0;
  
  // Get pointclouds for each object
  std::vector<pcl::PointCloud<PointNC>::Ptr> objectClouds (object_segments.size());
  for (size_t objId = 0; objId < object_segments.size(); objId++)
  {
    objectClouds[objId].reset (new pcl::PointCloud<PointNC>);
    pcl::copyPointCloud(*tabletop_cloud, object_segments[objId], *objectClouds[objId]);
  }
    
  VisState visState;
  pcl::visualization::PCLVisualizer visualizer;
  visualizer.setCameraPosition (  2.5, 0.0, 2.0,   // camera position
                                  0.0, 0.0, 0.0,   // viewpoint
                                  0.0, 0.0, 1.0,   // normal
                                  0.0);            // viewport
  visualizer.setBackgroundColor (utl::pclvis::bgColor.r, utl::pclvis::bgColor.g, utl::pclvis::bgColor.b);
  visualizer.registerKeyboardCallback(keyboard_callback_hd, (void*)(&visState));  
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
            
      if (visState.cloudDisplayState_ == VisState::CLOUD)
      {
        utl::pclvis::showPointCloudColor<PointNC>(visualizer, tabletop_cloud, "cloud", visState.pointSize_);

        if (visState.showNormals_)
          utl::pclvis::showNormalCloud<PointNC>(visualizer, tabletop_cloud, 10, 0.02, "normals", visState.pointSize_, utl::pclvis::green);
        
        visualizer.addText("Cloud", 15, 100, 24, 1.0, 1.0, 1.0);        
      }
      
      else if ( visState.cloudDisplayState_ == VisState::SYMMETRIES || visState.cloudDisplayState_ == VisState::HANDLE_POINTS)
      {
        std::vector<pcl::PointCloud<PointNC>::Ptr> cloudsDisplay;
        std::string text;
        
        if (visState.cloudDisplayState_ == VisState::SYMMETRIES)
        {
          cloudsDisplay = objectClouds;
          text = "Symmetries ";
        }
        else if (visState.cloudDisplayState_ == VisState::HANDLE_POINTS)
        {
          cloudsDisplay = handleClouds;
          text = "Handle points ";
        }
        
        // Display individual symmetries
        if (!visState.showAll_)
        {
          // Get iterator
          visState.segIterator_ = utl::math::clampValue<int>(visState.segIterator_, 0, object_segments.size()-1);
          int objId = visState.segIterator_;

          // Variables
          pcl::PointCloud<PointNC>::Ptr cloudDisplay (new pcl::PointCloud<PointNC>);
          
          // Show symmetry axis
          sym::showRotationalSymmetry(visualizer, object_symmetries[objId], "symmetry_hyp", 0.2, 4.0);
          
          // Add reconstructed cloud
          if (visState.showReconstructedCloud_)
          {
            pcl::PointCloud<PointNC>::Ptr reconstructedModel (new pcl::PointCloud<PointNC>);
            object_symmetries[objId].reconstructCloud<PointNC>(*cloudsDisplay[objId], *reconstructedModel);
            utl::pclvis::showPointCloud<PointNC>(visualizer, reconstructedModel, "reconstructed model", visState.pointSize_, utl::pclvis::blue);
          }
          
          // Add current segment cloud
          
          if (visState.showProfileCurve_)
          {
            object_symmetries[objId].getProfileCurveWithNormals(*cloudsDisplay[objId], *cloudDisplay);
          }
          else
          {
            cloudDisplay = cloudsDisplay[objId];
          }
         
          // Show segment
          utl::pclvis::showPointCloud<PointNC>(visualizer, cloudDisplay, "segment", visState.pointSize_, utl::pclvis::red);
          if (visState.showNormals_)
            utl::pclvis::showNormalCloud<PointNC>(visualizer, cloudDisplay, 10, 0.02, "normals", visState.pointSize_, utl::pclvis::green);
          
          // Show bounding cylinder
          if (visState.showBoundingCylinder_)
          {
            g1::vision::showCylinder(visualizer, object_bounding_cylinders[objId], "bonding_cylinder", utl::pclvis::Color(), 0.5);
          }
                    
          // Add text
          visualizer.addText(text + std::to_string(objId+1) + " / " + std::to_string(object_segments.size()), 0, 150, 24, 1.0, 1.0, 1.0);
        }
        
        // Show all detected objects
        else
        {
          // Show cloud
          utl::pclvis::showPointCloudColor<PointNC>(visualizer, tabletop_cloud, "cloud", visState.pointSize_/2, 0.7);
          
          for (size_t objId = 0; objId < object_segments.size(); objId++)
          {
            std::string id_prefix = std::to_string (objId);
            utl::pclvis::Color segmentColor = utl::pclvis::getGlasbeyColor(objId);
                        
            // Show symmetry
            sym::RotationalSymmetry symmetryDisplay = object_symmetries[objId];
            sym::showRotationalSymmetry(visualizer, symmetryDisplay, "symmetry_" + id_prefix, 0.2, 4.0);
            
            // Show cloud
            utl::pclvis::showPointCloud<PointNC>(visualizer, cloudsDisplay[objId], "segment_" + id_prefix, visState.pointSize_, segmentColor);
            
            // Show cylinder model
            if (visState.showBoundingCylinder_)
              g1::vision::showCylinder(visualizer, object_bounding_cylinders[objId], "bonding_cylinder_" + id_prefix, utl::pclvis::Color(), 0.5);
          }
          
          visualizer.addCoordinateSystem(0.5);
        }
      }

      // Show handle segments
      else if ( visState.cloudDisplayState_ == VisState::HANDLE_SEGMENTS ||
                visState.cloudDisplayState_ == VisState::HANDLE_SEGMENTS_FILTERED ||
                visState.cloudDisplayState_ == VisState::HANDLE_SEGMENTS_FINAL
              )
      {
        std::vector<std::pair<int, std::vector<int> > > handleSegmentIdsDisplay;
        std::string text;
        
        if (visState.cloudDisplayState_ == VisState::HANDLE_SEGMENTS)
        {
          handleSegmentIdsDisplay = handleSegmentIds;
          text = "Handle segments ";
        }
        else if (visState.cloudDisplayState_ == VisState::HANDLE_SEGMENTS_FILTERED)
        {
          handleSegmentIdsDisplay = handleSegmentFilteredIds;
          text = "Handle segments filtered ";
        }
        else if (visState.cloudDisplayState_ == VisState::HANDLE_SEGMENTS_FINAL)
        {
          handleSegmentIdsDisplay = handleSegmentFinalIds;
          text = "Handle segments final ";
        }
                
        if (!visState.showAll_)
        {  
          if (handleSegmentIdsDisplay.size() == 0)
          {
            visualizer.addText(text, 0, 150, 24, 1.0, 1.0, 1.0);
            visualizer.addText("No handles found", 0, 115, 24, 1.0, 1.0, 1.0);
          }
          else
          {
            // Get iterator
            visState.segIterator_ = utl::math::clampValue<int>(visState.segIterator_, 0, handleSegmentIdsDisplay.size()-1);
            int objIdIt = visState.segIterator_;
            int objId = handleSegmentIdsDisplay[objIdIt].first;

            visualizer.addText(text + std::to_string(objIdIt+1) + " / " + std::to_string(handleSegmentIdsDisplay.size()), 0, 150, 24, 1.0, 1.0, 1.0);
            
            if (handleSegmentIdsDisplay[objIdIt].second.size() == 0)
            {
              visualizer.addText("No valid handle segments", 0, 115, 24, 1.0, 1.0, 1.0);
            }
            else
            {
              visState.handleSegIterator_ = utl::math::clampValue<int>(visState.handleSegIterator_, 0, handleSegmentIdsDisplay[objIdIt].second.size()-1);
              int handleSegIdIt = visState.handleSegIterator_;
              int handleSegId = handleSegmentIdsDisplay[objIdIt].second[visState.handleSegIterator_];
              
              utl::pclvis::showPointCloud<PointNC>(visualizer, handleSegmentClouds[objId][handleSegId], "segment", visState.pointSize_, utl::pclvis::red);          
              utl::pclvis::showPointCloud<PointNC>(visualizer, objectClouds[objId], "cloud", visState.pointSize_, utl::pclvis::Color(), 0.1);
              
              showCylinder(visualizer, object_bounding_cylinders[objId], "cylinder", utl::pclvis::white, 0.5);
              showBox(visualizer, handleBoxes[objId][handleSegId], "handle", utl::pclvis::white, 0.5);
                                  
              // Add text
              visualizer.addText("Handle segment " + std::to_string(handleSegId+1) + " / " + std::to_string(handleSegmentClouds[objId].size()), 0, 125, 24, 1.0, 1.0, 1.0);
              visualizer.addText("Coverage angle: " + std::to_string (pcl::rad2deg(coverageAngles[objId][handleSegId])), 0, 100, 24, 1.0, 1.0, 1.0);
              visualizer.addText("Size ratio: " + std::to_string (handleSizeRatios[objId][handleSegId]), 0, 75, 24, 1.0, 1.0, 1.0);
            }
          }
        }
        else
        {
          // Show cloud
          utl::pclvis::showPointCloudColor<PointNC>(visualizer, tabletop_cloud, "cloud", visState.pointSize_/2, 0.3);
          
          for (size_t objIdIt = 0; objIdIt < handleSegmentIdsDisplay.size(); objIdIt++)
          {
            int objId = handleSegmentIdsDisplay[objIdIt].first;
            std::string id_prefix = std::to_string (objId);
            utl::pclvis::Color segmentColor = utl::pclvis::getGlasbeyColor(objId);
                                    
            // Show cloud
            utl::pclvis::showPointCloud<PointNC>(visualizer, objectClouds[objId], "segment_" + id_prefix, visState.pointSize_, segmentColor);
            
            // Show cylinder model
            if (visState.showBoundingCylinder_)
              g1::vision::showCylinder(visualizer, object_bounding_cylinders[objId], "bonding_cylinder_" + id_prefix, utl::pclvis::Color(), 0.5);
            
            // Show handles
            for (size_t handleSegIdIt = 0; handleSegIdIt < handleSegmentIdsDisplay[objIdIt].second.size(); handleSegIdIt++)
            {
              int handleSegId = handleSegmentIdsDisplay[objIdIt].second[handleSegIdIt];
              showBox(visualizer, handleBoxes[objId][handleSegId], "handle_" + std::to_string(objId) + "_" + std::to_string(handleSegId), segmentColor, 0.3);
            }
          }
          visualizer.addCoordinateSystem(0.5);          
        }
      }

      // Show table plane
      if (visState.showTablePlane_)
        utl::pclvis::showPlane (visualizer, planePoint, planeNormal, "table_plane", 1.0);
    }
    
    // Spin once
    visualizer.spinOnce();
    boost::this_thread::sleep (boost::posix_time::milliseconds (50));
  }
  
  return true;
}