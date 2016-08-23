#ifndef ATTRIBUTE_PROCESS_H
#define ATTRIBUTE_PROCESS_H

#include <pcl/segmentation/impl/region_growing.hpp>
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

#include <Eigen/Dense>
#include <pcl/segmentation/extract_clusters.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/features/normal_3d.h>
#include <graph_filter/FindPlanes.h>
#include <pcl/common/intersections.h>
#include <Eigen/Geometry> 

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <set>
#include <string>
#include <unordered_set>
#include <time.h> 

// #include <object_cls_msgs/BBox.h>
#include <hand_tracker_2d/HandBBox.h>
#include <graph_filter/Centroid.h>

  class AttributeProcess {
    public:
      AttributeProcess();
      ~AttributeProcess();
      // AttributeProcess(const pcl::visualization::PCLVisualizer & pv);     
      void initialize();
      void segmentPlane();
      void generateGraph();
      void setTimeStamp(ros::Time stamp_);

      void setInputCloudAndTransform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_,
        const Eigen::Affine3d & T_Eigen);

      void visualizeTable(ros::Publisher& tf_pub);
      void findTabletop();
      void run(ros::Publisher& tf_pub);

    private:
      ros::Time frame_time_stamp;
      Eigen::Affine3d T_Eigen;
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr orig_cloud_;
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_;

      std::vector<pcl::Normal> planecoeffs;
      std::vector<std::vector<int> > planePointIndices;
      std::vector<std::vector<bool> > nbgh_matrix;
      std::vector<int> tableIndices;

      void segmentPlaneOrganized();
      void transformSurfaceNormals();
      void computeNeighborsOrganized();
      void ConvertPCLCloud2ColorSeg(const std::vector<int>& indices,
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &out);
      void getTableTopObjects();
      // ros::Publisher tf_pub;
  };

#endif
