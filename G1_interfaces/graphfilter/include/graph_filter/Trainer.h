#ifndef TRAINER_H
#define TRAINER_H

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/search/pcl_search.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

#include <omp.h>
#include <Eigen/Dense>
#include <pcl/segmentation/extract_clusters.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/features/normal_3d.h>
#include <graph_filter/FindPlanes.h>
#include <pcl/common/intersections.h>
#include <Eigen/Geometry> 

// #include <graph_filter/FitSize.h>
// #include <graph_filter/Constraints.h>
// #include <graph_filter/NodeConstraints.h>
// #include <graph_filter/EdgeConstraints.h>
// #include <graph_filter/SizeConstraints.h>
// #include <graph_filter/LocationConstraints.h>
// #include <graph_filter/FingerConstraints.h>
// #include <graph_filter/ArmFingerConstraints.h>
// #include <graph_filter/Pca.h>
// #include <graph_filter/Centroid.h>
#include <graph_filter/GraphFilter.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <set>
#include <string>
#include <unordered_set>
#include <time.h> 

// #include <object_cls_msgs/BBox.h>
#include <hand_tracker_2d/HandBBox.h>

  class Trainer {
    public:
      Trainer();
      ~Trainer();
      void initialize();

      void addTrainingSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_,
        const Eigen::Affine3d & T_Eigen);


   
    private:
      std::vector<GraphFilter> samples;
  };

#endif
