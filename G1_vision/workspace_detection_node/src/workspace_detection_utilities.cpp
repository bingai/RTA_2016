#include <workspace_detection_node/workspace_detection_utilities.hpp>

#include <namaris/utilities/pointcloud.hpp>
#include <namaris/utilities/map.hpp>
#include <namaris/algorithms/region_growing_normal_variation/region_growing_normal_variation.hpp>

#include <sisyphus/box_detection.hpp>
#include <sisyphus/box_fitting.hpp>

bool tablePlaneUnaryConditionFunction(const PointNC& p, const Eigen::Vector3f &gravity_vector, const float gravity_angle_thresh) {
	Eigen::Vector3f normal = p.getNormalVector3fMap();
	return normal.dot(-gravity_vector) > std::cos(gravity_angle_thresh) && p.z > -0.4f;
}

bool g1::vision::findTableBox	(	const pcl::PointCloud<PointNC>::ConstPtr &cloud,
									float table_height,
									const std::string &table_id,
									g1::vision::Box &table_box
								)
{

	// Table plane point detection
	float voxelSize = 0.005;
	utl::map::Map planePointIndices;
	int numPlanesDected = 0;

	// Parameters
	float gravityVectorAngleThreshold = pcl::deg2rad(10.0);
	float normalVariationThreshold    = pcl::deg2rad(15.0) * 100;
	int minTablePlanePoints = 1000;

	// Create unary condition function
	Eigen::Vector3f gravityVector (0.0f, 0.0f, -1.0f);  
	boost::function<bool (const PointNC&)> tablePlaneUnaryFunction =
	boost::bind (&tablePlaneUnaryConditionFunction, _1, gravityVector, gravityVectorAngleThreshold);

	// Create region growing segmentation object
	alg::RegionGrowingNormalVariation<PointNC> rg;
	rg.setInputCloud (cloud);
	rg.setConsistentNormals (true);
	rg.setNumberOfNeighbors (5);
	rg.setSearchRadius (voxelSize * std::sqrt (2));
	rg.setNormalVariationThreshold (normalVariationThreshold);
	rg.setMinValidBinaryNeighborsFraction (0.8);
	rg.setUnaryConditionFunction (tablePlaneUnaryFunction);
	rg.setMinSegmentSize (minTablePlanePoints);
	rg.segment (planePointIndices);

	numPlanesDected = planePointIndices.size ();
	if (numPlanesDected == 0) {
		return false;
	}

	// Fit convex hulls to remaining table planes
	std::vector<pcl::PointCloud<PointNC>::Ptr> planeHulls (planePointIndices.size());
	std::vector<float> planeAreas                         (planePointIndices.size());
	std::vector<Eigen::Vector4f> planeCoefficients        (planePointIndices.size());

	utl::cloud::ConvexHull2D<PointNC> chull2d;
	chull2d.setInputCloud (cloud);

	for (size_t planeId = 0; planeId < planePointIndices.size (); planeId++) {
		planeHulls[planeId].reset (new pcl::PointCloud<PointNC>);
		chull2d.setIndices (boost::make_shared<std::vector<int> > (planePointIndices[planeId]) );
		chull2d.reconstruct (*planeHulls[planeId]);
		planeAreas[planeId] = chull2d.getTotalArea ();
		planeCoefficients[planeId] = chull2d.getPlaneCoefficients ();
	}

	// Select table plane with the largest convex hull area
	float maxArea;
	std::vector<int> maxAreaIds;
	utl::stdvec::vectorMaxLoc (planeAreas, maxArea, maxAreaIds);

	int tablePlaneId = maxAreaIds[0];
	std::vector<int> tablePlanePointIndices = planePointIndices[tablePlaneId];
	pcl::PointCloud<PointNC>::Ptr table_plane_hull = planeHulls[tablePlaneId];
	Eigen::Vector4f table_plane_coefficients = planeCoefficients[tablePlaneId];

	Eigen::Vector3f planePoint, planeNormal;
	utl::geom::planeCoefficientsToPointNormal(table_plane_coefficients, planePoint, planeNormal);
	if (planeNormal[2] < 0.0f)
		table_plane_coefficients *= -1;

	// Get the table box
	table_plane_coefficients(3) += table_height;
	table_box.id_ = table_id;
	return fitMinimumVolumeBoundingBoxConstrained<PointNC>(table_plane_hull, table_plane_coefficients, table_box.pose_, table_box.size_);
}

bool g1::vision::findFridgeBox	(	const pcl::PointCloud<PointNC>::ConstPtr &cloud,
									bool interior,
									const std::string &fridge_box_id,
									g1::vision::Box &fridge_box
								)
{
	// Segmenting parameters
	int min_cluster_size = 1000;
	int ransac_iter = 1000;
	float max_plane_dist = 0.015;
	float max_seg_dist = 0.2;
	float rg_radius = 0.02;
	float rg_normal_var_thresh = 100*30.0*M_PI/180.0;
	float rg_angle_thresh = 30.0*M_PI/180.0;
	float angle_tol = 5.0*M_PI/180.0;
	float max_angle_diff = 15.0*M_PI/180.0;
	float angle_diff_cluster = 10.0*M_PI/180.0;

	// Grouping parameters
	float max_side_dist = 0.1;
	float ratio_thresh = 0.05;
	float range_thresh = 0.40;
	bool outside = !interior;
	bool require_cliques = false;
	bool brute_force = false;

	// Detect boxes
	std::vector<::Box<PointNC> > boxes = detectBoxes<PointNC>(cloud, min_cluster_size, ransac_iter, max_plane_dist, max_seg_dist, rg_radius, rg_normal_var_thresh, rg_angle_thresh, angle_tol, max_angle_diff, angle_diff_cluster, max_side_dist, ratio_thresh, range_thresh, outside, require_cliques, brute_force);

	if (boxes.empty()) {
		return false;
	}

	// Get the largest box
	float max_v = 0.0;
	int max_ind;
	for (int i = 0; i < boxes.size(); ++i) {
		float v = boxes[i].size.prod();
		if (max_v < v) {
			max_v = v;
			max_ind = i;
		}
	}

	renameBoxAxes<PointNC>(boxes[max_ind], Eigen::Vector3f::Zero(), Eigen::Vector3f::UnitZ());

	fridge_box.pose_.matrix() = boxes[max_ind].pose;
	fridge_box.size_ = boxes[max_ind].size;
	fridge_box.id_ = fridge_box_id;

	return true;
}
