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
    , showBoundingCylinder_ (false)
    , updateDisplay_(true)
    , segIterator_(0)
    , handleSegIterator_(0)
    , showAll_(false)
    , pointSize_(4.0)
  {};
  
  enum CloudDisplay { CLOUD, SEGMENTATION, SYMMETRIES_INITIAL, SYMMETRIES_FILTERED, SYMMETRIES_REFINED, SYMMETRIES_MERGED, HANDLE_DETECTION, HANDLE_DETECTION_FILTERED, SEGMENTS_USED};
  
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
void keyboard_callback_rod (const pcl::visualization::KeyboardEvent &event, void *cookie)
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
      visState->cloudDisplayState_ = VisState::SEGMENTATION;
    else if (key == "KP_3")
      visState->cloudDisplayState_ = VisState::SYMMETRIES_INITIAL;
    else if (key == "KP_4")
      visState->cloudDisplayState_ = VisState::SYMMETRIES_FILTERED;
    else if (key == "KP_5")
      visState->cloudDisplayState_ = VisState::SYMMETRIES_REFINED;
    else if (key == "KP_6")
      visState->cloudDisplayState_ = VisState::SYMMETRIES_MERGED;
    else if (key == "KP_7")
      visState->cloudDisplayState_ = VisState::SEGMENTS_USED;
    
    else if (key == "Left")
      visState->segIterator_--;
    else if (key == "Right")
      visState->segIterator_++;
        
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
// Get bounding cylinder for an object
void getObjectBoundingCylinder  ( const pcl::PointCloud<PointNC> &object_segment,
                                  const sym::RotationalSymmetry &symmetry,
                                  const Eigen::Vector4f &table_plane_coefficients,
                                  pcl::ModelCoefficients &cylinder_coefs
                                )
{
  // Get indices of points that are above the table
  std::vector<int> validPointIndices (0);
  for (size_t pointId = 0; pointId < object_segment.size(); pointId++)
  {
    Eigen::Vector3f point = object_segment.points[pointId].getVector3fMap();
    if (utl::geom::pointToPlaneDistance(point, table_plane_coefficients) >= 0.02f)
      validPointIndices.push_back(pointId);
  }
  
  // Get cylinder coefficients
  symmetry.getBoundingCylinder<PointNC> (object_segment, validPointIndices, cylinder_coefs);

  // Nake sure that cylinder origin is at the table plane
  Eigen::Vector3f origin    (cylinder_coefs.values[0], cylinder_coefs.values[1], cylinder_coefs.values[2]);
  Eigen::Vector3f direction (cylinder_coefs.values[3], cylinder_coefs.values[4], cylinder_coefs.values[5]);
  float radius = cylinder_coefs.values[6];
  
  // Invert the direction axis so that it points with Z axis
  if (direction.dot(Eigen::Vector3f::UnitZ()) < 0.0f)
  {
    origin = origin + direction;
    direction = -direction;
  }
  
  Eigen::Vector3f topPoint = origin+direction;
  origin = utl::geom::linePlaneIntersection(origin, topPoint, table_plane_coefficients);
  direction = topPoint - origin;
  
  cylinder_coefs.values[0] = origin[0];
  cylinder_coefs.values[1] = origin[1];
  cylinder_coefs.values[2] = origin[2];
  cylinder_coefs.values[3] = direction[0];
  cylinder_coefs.values[4] = direction[1];
  cylinder_coefs.values[5] = direction[2];
}

////////////////////////////////////////////////////////////////////////////////
// Convert a pcl cylinder model coefficients to g1::vision cylinder
// NOTE: assuming that z axis is pointing in the general Z direction
g1::vision::Cylinder pclCylinderToG1Cylinder  (const pcl::ModelCoefficients &pcl_cylinder, const std::string &id)
{
  // Get cylinder parameters
  Eigen::Vector3f origin    (pcl_cylinder.values[0], pcl_cylinder.values[1], pcl_cylinder.values[2]);
  Eigen::Vector3f direction (pcl_cylinder.values[3], pcl_cylinder.values[4], pcl_cylinder.values[5]);
  float radius = pcl_cylinder.values[6];

  // Construct a coordinate frame
  Eigen::Vector3f cX, cY, cZ;
  cZ = direction.normalized();
  cY = cZ.cross(origin).normalized();
  cX = cY.cross(cZ);
  
  Eigen::Affine3f pose;
  pose.translation() = origin;
  pose.linear().col(0) = cX;
  pose.linear().col(1) = cY;
  pose.linear().col(2) = cZ;
  pose.translate (pose.rotation().col(2) * direction.norm () / 2);
  
  // Construct cylinder obect
  g1::vision::Cylinder g1_cylinder;
  g1_cylinder.pose_ = pose;
  g1_cylinder.height_ = direction.norm ();
  g1_cylinder.radius_ = radius;
  g1_cylinder.id_ = id;
  
  return g1_cylinder;
}

////////////////////////////////////////////////////////////////////////////////
// Visualize a cylinder
void g1::vision::showCylinder ( pcl::visualization::PCLVisualizer &visualizer,
                                const g1::vision::Cylinder &cylinder,
                                const std::string &id,
                                utl::pclvis::Color color,
                                const float opacity
                              )
{
  //----------------------------------------------------------------------------
  // Convert cylinder to PCL
  
  pcl::ModelCoefficients cylinder_coefs;
  cylinder_coefs.values.resize(7);

  // Direction
  Eigen::Vector3f direction = cylinder.pose_.linear().col(2) * cylinder.height_;
  cylinder_coefs.values[3] = direction[0];
  cylinder_coefs.values[4] = direction[1];
  cylinder_coefs.values[5] = direction[2];
  
  // Origin
  Eigen::Vector3f origin    = cylinder.pose_.translation() - direction / 2.0f;
  cylinder_coefs.values[0] = origin[0];
  cylinder_coefs.values[1] = origin[1];
  cylinder_coefs.values[2] = origin[2];  
    
  // Radius
  cylinder_coefs.values[6] = cylinder.radius_;
  
  //----------------------------------------------------------------------------
  // Visualize
  
  visualizer.addCylinder(cylinder_coefs, id);
  utl::pclvis::setShapeRenderProps(visualizer, id, color, opacity);
  visualizer.addCoordinateSystem(cylinder.height_, cylinder.pose_, id + "_pose");
}

////////////////////////////////////////////////////////////////////////////////
// Run
bool g1::vision::rotationalObjectDetection  ( const pcl::PointCloud<PointNC>::ConstPtr &tabletop_cloud,
                                              const utl::map::Map &segments,
                                              const Eigen::Vector4f &table_plane_coefficients,
                                              std::vector<sym::RotationalSymmetry> &object_symmetries,
                                              std::vector<Cylinder> &object_bounding_cylinders,
                                              std::vector<std::vector<int> > &object_segments,
                                              const bool visualize
                                            )
{
  // Check that segments are non-empty
  
  //----------------------------------------------------------------------------
  // Table plane parameters
  //----------------------------------------------------------------------------
  
  Eigen::Vector3f planePoint, planeNormal;
  utl::geom::planeCoefficientsToPointNormal<float> (table_plane_coefficients, planePoint, planeNormal);
//   planeNormal = Eigen::Vector3f::UnitZ();
  
  PointNC centroid;
  pcl::computeCentroid (*tabletop_cloud, centroid);
  planePoint = utl::geom::projectPointToPlane<float> (centroid.getVector3fMap (), planePoint, planeNormal);
  
  //----------------------------------------------------------------------------
  // Symmetry detection parameters
  //----------------------------------------------------------------------------  

  float maxPointFitnessScore = std::sin (M_PI / 4);
  float perpendicularThreshold = pcl::deg2rad (75.0f);
  
  float maxFitnessScore = 0.12;
  float minPerpendicularScore = 0.3;
  float minCoverageAngle = pcl::deg2rad(60.0);
  
  // ---------------------------------------------------------------------------
  // Rotational symmetry detection
  // ---------------------------------------------------------------------------
    
  std::cout << "Rotational symmetry detection..." << std::endl;
  
  double start = pcl::getTime();
  
  std::vector<pcl::PointCloud<PointNC>::Ptr> segmentClouds (segments.size());
  sym::RotationalSymmetries     symmetries          (segments.size());
  std::vector<float>            errors              (segments.size());
  std::vector<int>              statuses            (segments.size());
  std::vector<int>              numIterations       (segments.size());
  std::vector<float>            perpendicularScores (segments.size());
  std::vector<float>            fitnessScores       (segments.size());
  std::vector<float>            coverageAngles      (segments.size());
  
  std::vector<int> segmentIdsInitial                (segments.size());
  for (size_t segId = 0; segId < segments.size(); segId++)
    segmentIdsInitial[segId] = segId;
  
  # pragma omp parallel for
  for (size_t segId = 0; segId < segments.size(); segId++)
  {
    segmentClouds[segId].reset(new pcl::PointCloud<PointNC>);
    pcl::copyPointCloud<PointNC>(*tabletop_cloud, segments[segId], *segmentClouds[segId]);
    
    sym::detectRotationalSymmetry<PointNC>  ( segmentClouds[segId],
                                              symmetries[segId],
                                              planeNormal,
                                              fitnessScores[segId],
                                              perpendicularScores[segId],
                                              maxPointFitnessScore,
                                              perpendicularThreshold
                                            );    
    
    coverageAngles[segId] = sym::cloudRotationalSymmetryCoverageAngle<PointNC> (symmetries[segId], *segmentClouds[segId]);
  }  
  
  std::cout << "  " << segments.size() << " initial segments." << std::endl;
  std::cout << "  " << (pcl::getTime() - start) << " seconds." << std::endl;

  // ---------------------------------------------------------------------------
  // Filter out segments that do not fit symmetry well
  // ---------------------------------------------------------------------------  
  // Filter out segments that have too low perpendicular score and those with
  // too high fitness score.
  
  std::cout << "Symmetry filtering..." << std::endl;
  start = pcl::getTime();

  std::vector<int> segmentIdsFiltered;
  
  for (size_t segId = 0; segId < segments.size(); segId++)
  {
    if (perpendicularScores[segId] > minPerpendicularScore && fitnessScores[segId] < maxFitnessScore && coverageAngles[segId] > minCoverageAngle)
      segmentIdsFiltered.push_back(segId);
  }  

  std::cout << "  " << segmentIdsFiltered.size() << " filtered segments." << std::endl;
  std::cout << "  " << (pcl::getTime() - start) << " seconds." << std::endl;

  // ---------------------------------------------------------------------------
  // Get bounding cylinders
  // ---------------------------------------------------------------------------  

  std::vector<pcl::ModelCoefficients> boundingCylinders (segments.size());;
  
  for (size_t symIdIt = 0; symIdIt < segmentIdsFiltered.size(); symIdIt++)
  {
    int segId = segmentIdsFiltered[symIdIt];
    getObjectBoundingCylinder(*segmentClouds[segId], symmetries[segId], table_plane_coefficients, boundingCylinders[segId]);
  }

  //----------------------------------------------------------------------------
  // Refine symmetries
  //----------------------------------------------------------------------------
  
  std::cout << "Symmetry refinement..." << std::endl;
  start = pcl::getTime();
  
  // Variables
  utl::map::Map segmentsRefined                                     (segments.size());
  std::vector<pcl::PointCloud<PointNC>::Ptr> segmentsRefinedClouds  (segments.size());
  std::vector<float>            perpendicularScoresRefined          (segments.size());
  std::vector<float>            fitnessScoresRefined                (segments.size());
  std::vector<pcl::ModelCoefficients> boundingCylindersRefined      (segments.size());
  sym::RotationalSymmetries symmetriesRefined                       (segments.size());
  std::vector<std::vector<int> > adjacentSegments                   (segments.size());
  
//   # pragma omp parallel for  
  for (size_t segIdIt = 0; segIdIt < segmentIdsFiltered.size(); segIdIt++)
  {
    int segId = segmentIdsFiltered[segIdIt];
    
    // Find all points within a radius from the object
    std::vector<int> circularNeighborPoints;
    float maxRadius = boundingCylinders[segId].values[6];
    float maxHeight = Eigen::Vector3f(boundingCylinders[segId].values[3], boundingCylinders[segId].values[4], boundingCylinders[segId].values[5]).norm() * 1.05;
    for (size_t pointId = 0; pointId < tabletop_cloud->size(); pointId++)
    {
      Eigen::Vector3f point = tabletop_cloud->points[pointId].getVector3fMap();
      float radialDistance = symmetries[segId].pointDistance(point);
      float verticalDistance = utl::geom::pointToPlaneDistance(point, table_plane_coefficients);
      if (radialDistance <= maxRadius && verticalDistance <= maxHeight)
        circularNeighborPoints.push_back(pointId);
    }
    
    // Find segments that overlap with neighbouring points
    segmentsRefined[segId] = segments[segId];
    for (size_t segId2 = 0; segId2 < segments.size(); segId2++)
    {
      // If this is the same as current segments - we are using it!
      if (segId2 == segId)
        adjacentSegments[segId2].push_back(segId2);
      
      // Otherwise find the overlap between two segments
      int overlap = utl::stdvec::vectorIntersection(circularNeighborPoints, segments[segId2]).size();
      if (static_cast<float>(overlap) / static_cast<float>(segments[segId2].size()) > 0.7)
      {
        std::cout << segIdIt << ": " << static_cast<float>(overlap) / static_cast<float>(segments[segId2].size()) << std::endl;
        adjacentSegments[segId2].push_back(segId2);
        segmentsRefined[segId].insert(segmentsRefined[segId].end(), segments[segId2].begin(), segments[segId2].end());
      }
    }
    
    // Construct pointcloud from used segment points
    segmentsRefinedClouds[segId].reset (new pcl::PointCloud<PointNC>);
    pcl::copyPointCloud<PointNC>(*tabletop_cloud, segmentsRefined[segId], *segmentsRefinedClouds[segId]);
    
    // Find symmetry with refined pointcloud
    sym::detectRotationalSymmetry<PointNC>  ( segmentsRefinedClouds[segId],
                                              symmetriesRefined[segId],
                                              planeNormal,
                                              fitnessScoresRefined[segId],
                                              perpendicularScoresRefined[segId],
                                              maxPointFitnessScore,
                                              perpendicularThreshold
                                            );
    
    // Get new bounding cylinder
    getObjectBoundingCylinder(*segmentsRefinedClouds[segId], symmetriesRefined[segId], table_plane_coefficients, boundingCylindersRefined[segId]);
    
    // Find all segments that are used by the refined symmetry
    for (size_t segId2 = 0; segId2 < segments.size(); segId2++)
    {
      // If this is the same as current segments - we are using it!
      if (segId2 == segId)
        adjacentSegments[segId].push_back(segId2);
      
      // Otherwise find the overlap between two segments
      int overlap = utl::stdvec::vectorIntersection(circularNeighborPoints, segments[segId2]).size();
      if (static_cast<float>(overlap) / static_cast<float>(segments[segId2].size()) > 0.7)
        adjacentSegments[segId].push_back(segId2);
    }
  }
  
  std::cout << "  " << (pcl::getTime() - start) << " seconds." << std::endl;
  
  //----------------------------------------------------------------------------
  // Merge dupplicate symmetries
  //----------------------------------------------------------------------------

  std::cout << "Merging duplicate detections and look for duplicate detections..." << std::endl;
  start = pcl::getTime();
    
  // Parameters
  float duplicateOverlapThresh = 0.9;   // Two segments are considered duplicate if the overlap area is greater than this fraction of the total areas of both circles
  float conflictOverlapThresh = 0.1;    // Two segments are considered conflicting if the overlap area is greater than this fraction of the total areas of any two of the circles
  
  // Variables
  std::vector<int> segmentIdsMerged;                 // Ids of merged segments
  
  // Construct a graph where vertices represent object segments and edges
  // indicate segments that duplicate
  utl::graph::Graph objectSegmentAdjacency (segmentIdsFiltered.size());
  
  // List of conflicting segments
  std::vector<std::pair<int, int> > conflictSegmentIds;
  
  for (size_t srcIt = 0; srcIt < segmentIdsFiltered.size(); srcIt++)
  {
    int srcId = segmentIdsFiltered[srcIt];
    
    // Get source circle parameters
    Eigen::Vector3f srcCircleC;
    srcCircleC[0] = boundingCylindersRefined[srcId].values[0];
    srcCircleC[1] = boundingCylindersRefined[srcId].values[1];
    srcCircleC[2] = boundingCylindersRefined[srcId].values[2];
    float srcCircleR = boundingCylindersRefined[srcId].values[6];
    
    for (size_t tgtIt = srcIt+1; tgtIt < segmentIdsFiltered.size(); tgtIt++)
    {
      int tgtId = segmentIdsFiltered[tgtIt];      
      
      // Get target circle parameters
      Eigen::Vector3f tgtCircleC;
      tgtCircleC[0] = boundingCylindersRefined[tgtId].values[0];
      tgtCircleC[1] = boundingCylindersRefined[tgtId].values[1];
      tgtCircleC[2] = boundingCylindersRefined[tgtId].values[2];
      float tgtCircleR = boundingCylindersRefined[tgtId].values[6];
      
      // Calculate overlap area
      Eigen::Vector2f srcCircleC2d  (0.0f, 0.0f);
      Eigen::Vector2f tgtCircleC2d  (utl::geom::pointToPointDistance<float>(srcCircleC, tgtCircleC), 0.0f);
      
      float intersectionArea = utl::geom::circleCircleIntersectionArea<float>(srcCircleC2d, srcCircleR, tgtCircleC2d, tgtCircleR);
            
      float srcItersectFraction = intersectionArea / (M_PI * srcCircleR * srcCircleR);
      float tgtItersectFraction = intersectionArea / (M_PI * tgtCircleR * tgtCircleR);
      
      if (srcItersectFraction > duplicateOverlapThresh || tgtItersectFraction > duplicateOverlapThresh)
        utl::graph::addEdge(srcIt, tgtIt, objectSegmentAdjacency);
      
      else if (srcItersectFraction > conflictOverlapThresh || tgtItersectFraction > conflictOverlapThresh)
      {
        conflictSegmentIds.push_back(std::pair<int,int>(srcId, tgtId));
        std::cout << srcItersectFraction << ", " << tgtItersectFraction << std::endl;
      }
    }
  }

  // Find all connected components in the graph
  std::vector<std::vector<int> > objectHypothesisClusters;
  objectHypothesisClusters = utl::graph::getConnectedComponents(objectSegmentAdjacency);
  
  // Select best hypothesis for each cluster
  // - have the largest support
  // - out of all clusters having the same support have the smallest radius
  segmentIdsMerged.resize(objectHypothesisClusters.size());
  
  for (size_t clusterId = 0; clusterId < objectHypothesisClusters.size(); clusterId++)
  {
    // Find the object hypothesis with the largest support
    std::vector<int> bestHypothesisCandidates;
    int maxSupport = 0;
    
    for (size_t segIdIt = 0; segIdIt < objectHypothesisClusters[clusterId].size(); segIdIt++)
    {
      int segId = segmentIdsFiltered[objectHypothesisClusters[clusterId][segIdIt]];
      int support = segmentsRefined[segId].size();
      
      if (support == maxSupport)
      {
        bestHypothesisCandidates.push_back(segId);
      }
      else if (support > maxSupport)
      {
        bestHypothesisCandidates.resize(1);
        bestHypothesisCandidates[0] = segId;
        maxSupport = support;
      }
    }
    
    // Out of all hypotheses with same support select the one with the smallest radius
    float minRadius = std::numeric_limits<float>::max();
    int bestHypothesisId = -1;
    for (size_t bestHypothesisIt = 0; bestHypothesisIt < bestHypothesisCandidates.size(); bestHypothesisIt++)
    {
      int segId = bestHypothesisCandidates[bestHypothesisIt];
      if (boundingCylindersRefined[segId].values[6] < minRadius)
      {
        minRadius = boundingCylindersRefined[segId].values[6];
        bestHypothesisId = segId;
      }
    }
    
    segmentIdsMerged[clusterId] = bestHypothesisId;
  }
  
  // Report if there are conflicting detections
  if (conflictSegmentIds.size() > 0)
  {
    std::cout << "  WARNING: there are some conflicting detections i.e. cylinders that overlap too much. Please investigate!" << std::endl;
  }
  
  std::cout << "  " << segmentIdsMerged.size() << " merged segments." << std::endl;
  std::cout << "  " << (pcl::getTime() - start) << " seconds." << std::endl;

  //----------------------------------------------------------------------------
  // Refine symmetry object support
  //----------------------------------------------------------------------------
  
  
  
  
  //----------------------------------------------------------------------------
  // Construct output
  //----------------------------------------------------------------------------
  
  object_symmetries.resize(segmentIdsMerged.size());
  object_bounding_cylinders.resize(segmentIdsMerged.size());
  object_segments.resize(segmentIdsMerged.size());
  
  for (size_t segIdIt = 0; segIdIt < segmentIdsMerged.size(); segIdIt++)
  {
    int segId = segmentIdsMerged[segIdIt];
    
    // Symmetry
    object_symmetries[segIdIt] = symmetriesRefined[segId];
    
    // Bounding boxes
    object_bounding_cylinders[segIdIt] = pclCylinderToG1Cylinder(boundingCylindersRefined[segId], "dish_" + std::to_string(segIdIt));
    
    // Segments used
    object_segments[segIdIt] = adjacentSegments[segId];
  }
      
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
  visualizer.registerKeyboardCallback(keyboard_callback_rod, (void*)(&visState));  
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
      else if (visState.cloudDisplayState_ == VisState::SEGMENTATION)
      {   
        utl::pclvis::showPointClouds<PointNC>(visualizer, segmentClouds, "segments", visState.pointSize_);
        
        if (visState.showNormals_)
          utl::pclvis::showNormalClouds<PointNC>(visualizer, segmentClouds, 10, 0.02, "normals", visState.pointSize_, utl::pclvis::green);
                
        visualizer.addText("Segments", 0, 150, 24, 1.0, 1.0, 1.0);
      }
      
      else if ( visState.cloudDisplayState_ == VisState::SYMMETRIES_INITIAL   ||
                visState.cloudDisplayState_ == VisState::SYMMETRIES_FILTERED  ||
                visState.cloudDisplayState_ == VisState::SYMMETRIES_REFINED   ||
                visState.cloudDisplayState_ == VisState::SYMMETRIES_MERGED
         )
      {
        // Show cloud
        utl::pclvis::showPointCloudColor<PointNC>(visualizer, tabletop_cloud, "cloud", visState.pointSize_/2, 0.7);
                
        // Variables
        std::vector<pcl::PointCloud<PointNC>::Ptr> segmentsDisplay;
        std::vector<int> segmentIdsDisplay;        
        sym::RotationalSymmetries symmetriesDisplay;
        std::vector<pcl::ModelCoefficients> boundingCylindersDisplay;
        std::string text;
        std::vector<float> fitnessScoresDisplay, perpendicularScoresDisplay;
        
        if (visState.cloudDisplayState_ == VisState::SYMMETRIES_INITIAL)
        {
          segmentsDisplay   = segmentClouds;
          segmentIdsDisplay = segmentIdsInitial;
          symmetriesDisplay = symmetries;
          boundingCylindersDisplay = boundingCylinders;
          fitnessScoresDisplay = fitnessScores;
          perpendicularScoresDisplay = perpendicularScores;
          text = "Initial symmetries";
        }
        else if (visState.cloudDisplayState_ == VisState::SYMMETRIES_FILTERED)
        {
          segmentsDisplay   = segmentClouds;
          segmentIdsDisplay = segmentIdsFiltered;
          symmetriesDisplay = symmetries;
          boundingCylindersDisplay = boundingCylinders;
          fitnessScoresDisplay = fitnessScores;
          perpendicularScoresDisplay = perpendicularScores;
          text = "Filtered symmetries";
        }
        else if (visState.cloudDisplayState_ == VisState::SYMMETRIES_REFINED)
        {
          segmentsDisplay   = segmentsRefinedClouds;
          segmentIdsDisplay = segmentIdsFiltered;
          symmetriesDisplay = symmetriesRefined;
          boundingCylindersDisplay = boundingCylindersRefined;
          fitnessScoresDisplay = fitnessScoresRefined;
          perpendicularScoresDisplay = perpendicularScoresRefined;
          text = "Refined symmetries";
        }

        else if (visState.cloudDisplayState_ == VisState::SYMMETRIES_MERGED)
        {
          segmentsDisplay   = segmentsRefinedClouds;
          segmentIdsDisplay = segmentIdsMerged;
          symmetriesDisplay = symmetriesRefined;
          boundingCylindersDisplay = boundingCylindersRefined;
          fitnessScoresDisplay = fitnessScoresRefined;
          perpendicularScoresDisplay = perpendicularScoresRefined;
          text = "Merged symmetries";
        }
                
        // Check if there are any segments to display
        int numSegs = segmentIdsDisplay.size();
        if (numSegs == 0)
        {
          visualizer.addText("No valid symmetries", 0, 100, 24, 1.0, 1.0, 1.0);          
        }
        
        // Display individual symmetries
        else if (!visState.showAll_)
        {
          // Get iterator
          visState.segIterator_ = utl::math::clampValue<int>(visState.segIterator_, 0, numSegs-1);
          int segIdIt = visState.segIterator_;
          int segId = segmentIdsDisplay[segIdIt];
          
          // Get more display variables
          pcl::PointCloud<PointNC>::Ptr cloudDisplay (new pcl::PointCloud<PointNC>);
          sym::RotationalSymmetry symmetryDisplay = symmetriesDisplay[segId];
          pcl::PointCloud<PointNC>::Ptr segmentDisplay (new pcl::PointCloud<PointNC>);
          segmentDisplay = segmentsDisplay[segId];
          pcl::ModelCoefficients boundingCylinderDisplay = boundingCylindersDisplay[segId];
          float fitnessScoreDisplay = fitnessScoresDisplay[segId];
          float perpendicularScoreDisplay = perpendicularScoresDisplay[segId];
          float coverageAngleDisplay = coverageAngles[segId];
          
          // Show symmetry axis
          sym::showRotationalSymmetry(visualizer, symmetryDisplay, "symmetry_hyp", 0.2, 4.0);
          
          // Add reconstructed cloud
          if (visState.showReconstructedCloud_)
          {
            pcl::PointCloud<PointNC>::Ptr reconstructedModel (new pcl::PointCloud<PointNC>);
            symmetryDisplay.reconstructCloud<PointNC>(*segmentsDisplay[segId], *reconstructedModel);
            utl::pclvis::showPointCloud<PointNC>(visualizer, reconstructedModel, "reconstructed model", visState.pointSize_, utl::pclvis::blue);
          }
          
          // Add current segment cloud
          if (visState.showProfileCurve_)
          {
            symmetryDisplay.getProfileCurveWithNormals(*segmentsDisplay[segId], *cloudDisplay);
          }
          else
          {
            cloudDisplay = segmentsDisplay[segId];
          }
         
          // Show segment
          utl::pclvis::showPointCloud<PointNC>(visualizer, cloudDisplay, "segment", visState.pointSize_, utl::pclvis::red);
          if (visState.showNormals_)
            utl::pclvis::showNormalCloud<PointNC>(visualizer, cloudDisplay, 10, 0.02, "normals", visState.pointSize_, utl::pclvis::green);
          
          // Show bounding cylinder
          if (visState.cloudDisplayState_ != VisState::SYMMETRIES_INITIAL &&  visState.showBoundingCylinder_)
          {
            visualizer.addCylinder(boundingCylinderDisplay, "bonding_cylinder");
            utl::pclvis::setShapeRenderProps(visualizer, "bonding_cylinder", utl::pclvis::Color(), 0.5);
          }
                    
          // Add text
          visualizer.addText(text, 0, 150, 24, 1.0, 1.0, 1.0);
          visualizer.addText("Segment " + std::to_string (segIdIt+1) + " / " + std::to_string (numSegs), 0, 125, 24, 1.0, 1.0, 1.0);
          visualizer.addText("Perpendicular score: " + std::to_string (perpendicularScoreDisplay), 0, 100, 24, 1.0, 1.0, 1.0);
          visualizer.addText("Fitness score: " + std::to_string (fitnessScoreDisplay), 0, 75, 24, 1.0, 1.0, 1.0);
          visualizer.addText("Coverage angle: " + std::to_string (pcl::rad2deg (coverageAngleDisplay)), 0, 50, 24, 1.0, 1.0, 1.0);
        }
        
        // Show all detected objects
        else
        {          
          for (size_t segIdIt = 0; segIdIt < segmentIdsDisplay.size(); segIdIt++)
          {
            int segId = segmentIdsDisplay[segIdIt];
            std::string id_prefix = std::to_string (segId);
                        
            // Show symmetry
            sym::RotationalSymmetry symmetryDisplay = symmetriesDisplay[segId];
            sym::showRotationalSymmetry(visualizer, symmetryDisplay, "symmetry_" + id_prefix, 0.2, 4.0);
            
            // Show cloud
            if (visState.showReconstructedCloud_)
            {
              pcl::PointCloud<PointNC>::Ptr reconstructedModel (new pcl::PointCloud<PointNC>);
              symmetryDisplay.reconstructCloud<PointNC>(*segmentClouds[segId], *reconstructedModel);
              utl::pclvis::showPointCloud<PointNC>(visualizer, reconstructedModel, "reconstructed_model_" + id_prefix, visState.pointSize_, utl::pclvis::blue);
            }
            else
            {
              utl::pclvis::showPointCloud<PointNC>(visualizer, segmentsDisplay[segId], "segment_" + id_prefix, visState.pointSize_, utl::pclvis::red);
            }
            
            // Show cylinder model
            if (visState.cloudDisplayState_ != VisState::SYMMETRIES_INITIAL)
            {
              if (visState.cloudDisplayState_ == VisState::SYMMETRIES_MERGED)
              {
                showCylinder(visualizer, object_bounding_cylinders[segIdIt], "cylinder_" + id_prefix, utl::pclvis::white, 0.5);
              }
              else
              {
                visualizer.addCylinder(boundingCylindersDisplay[segId], "bonding_cylinder_" + id_prefix);
                utl::pclvis::setShapeRenderProps(visualizer, "bonding_cylinder_" + id_prefix, utl::pclvis::white, 0.5);
              }
            }
          }
          
          visualizer.addCoordinateSystem(0.5);          
        }
      }
      
      // Show used segments
      else if ( visState.cloudDisplayState_ == VisState::SEGMENTS_USED)
      {
        for (size_t segId = 0; segId < segments.size(); segId++)
        {
          utl::pclvis::Color color = utl::pclvis::getGlasbeyColor(segId);
          float opacity = 1.0f;
          
          bool used = false;
          
          for (size_t rotObjId = 0; rotObjId < object_segments.size(); rotObjId++)
            for (size_t rotObjSegIdIt = 0; rotObjSegIdIt < object_segments[rotObjId].size(); rotObjSegIdIt++)
              if (std::find(object_segments[rotObjId].begin(), object_segments[rotObjId].end(), segId) != object_segments[rotObjId].end())
                used = true;
          
          if (!used)
          {
            opacity = 0.1;
            color = utl::pclvis::white;
          }
          
          utl::pclvis::showPointCloud<PointNC>(visualizer, segmentClouds[segId], "seg_" + std::to_string(segId), visState.pointSize_, color, opacity);
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