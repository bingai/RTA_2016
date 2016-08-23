#ifndef MICROWAVE_RECONSTRUCT_H
#define MICROWAVE_RECONSTRUCT_H

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
#include <omp.h>
#include <microwave_detection/FitSize.h>
#include <Eigen/Dense>
#include <pcl/segmentation/extract_clusters.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/features/normal_3d.h>
#include <microwave_detection/FindPlanes.h>
#include <pcl/common/intersections.h>
#include <Eigen/Geometry> 
#include <world_model_msgs/UpdateStatesObjects.h>

  class MicrowaveRect {
    public:
      MicrowaveRect();
      ~MicrowaveRect();
      void initialize();
      void segmentPlane();
      void setInputCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_);
      void setInputAndOrigCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_,
         pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr orig_cloud_);
      
      std::vector<std::vector<int> > planePointIndices;
      std::vector<std::vector<int> > component;
      
      void findMicrowave();
      bool generateMessage(world_model_msgs::UpdateStatesObjects & srv);
      shape_msgs::SolidPrimitive setSolidPrimitiveBox(Eigen::Vector3f dims);
      geometry_msgs::Pose setGeometryMsgs(Eigen::Vector3f c, 
        Eigen::Quaternionf q);

      void computeNeighbors();
      void printNeighbors();
      void computeRelation();
      void findConnectedComponent();
      void computePatchNorm(std::map<int, pcl::Normal>& PatchMeanN);
      void computeCentroid(std::map<int,  pcl::PointXYZ>& centroid);
      // boost::shared_ptr<std::vector<std::vector<bool> > > nbgh_matrix;
      std::vector<std::vector<bool> > nbgh_matrix;
      std::vector<std::vector<bool> > relations;
      Eigen::Affine3d T_Eigen;

      void setTransform(const Eigen::Affine3d & T_Eigen);
      void transformSurfaceNormals();
      
      void setUseBoxDetect(bool use_box_);

      void visualization();
   
      typedef union
      {
        struct
        {
          unsigned char b; // Blue channel
          unsigned char g; // Green channel
          unsigned char r; // Red channel
          unsigned char a; // Alpha channel
        };
        float float_value;
        long long_value;
       } RGBValue;

      float static GetRandomColor()
      {
        RGBValue x;
        x.b = std::rand()%255;
        x.g = std::rand()%255;
        x.r = std::rand()%255;
        x.a = 0.; 
        return x.float_value;
      };


      std::vector<pcl::Normal> planecoeffs;

      std::vector<int> handleIndices;
      Eigen::Vector3f frontNormal;
      Eigen::Vector3f topNormal;
      Eigen::Vector3f sideNormal;
      std::vector<int> frontDoor;
      std::vector<int> topFace;


      Eigen::Vector3f front_top_right;
      Eigen::Vector3f front_bottom_right;
      Eigen::Vector3f back_top_right;
      Eigen::Vector3f front_top_left;
      Eigen::Vector3f handle_top;
      Eigen::Vector3f handle_bottom;

      Eigen::Vector4f frontPlane;
      Eigen::Vector4f backPlane;
      Eigen::Vector4f topPlane;
      Eigen::Vector4f bottomPlane;
      Eigen::Vector4f rightPlane;
      Eigen::Vector4f leftPlane;
      Eigen::Vector4f handleSidePlane;

      Eigen::Quaternionf microwave_pose;
      Eigen::Vector3f top_centroid;
      Eigen::Vector3f top_dimension;

      Eigen::Vector3f right_side_centroid;
      Eigen::Vector3f right_side_dimension;

      Eigen::Vector3f left_side_centroid;
      Eigen::Vector3f left_side_dimension;

      Eigen::Vector3f bottom_centroid;
      Eigen::Vector3f bottom_dimension;

      Eigen::Vector3f door_centroid;
      Eigen::Vector3f door_dimension;

      Eigen::Vector3f handle_centroid;
      Eigen::Vector3f handle_dimension;

      Eigen::Vector3f door_open_centroid;
      Eigen::Quaternionf door_open_pose;

    private:
      pcl::search::Search<pcl::PointXYZRGBNormal>::Ptr searcher_;
      std::map<int, int> pointPlaneLabel;
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_;
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr orig_cloud_;

      bool use_box;
      int component_box_id;

      void fitCuboid();
      void microwaveParts();

      void locateFrontDoor(FitSize & fs, int box_id,
        std::map<int, pcl::Normal>& PatchMeanN);
      bool tablePlaneUnaryConditionFunction(const pcl::PointXYZRGBNormal& p, const Eigen::Vector3f &gravity_vector, const float gravity_angle_thresh);

      void setSearcher();
      void setPointPlaneLabel();
      void findHandle(FitSize& fs);
      void locateBox(std::vector<int> & surfaceState, FitSize & fs,
        std::map<int, pcl::Normal>& PatchMeanN);
      void FindHandleBox(FitSize & fs, std::map<int, pcl::Normal>& PatchMeanN);

      void ConvertPCLCloud2ColorSeg(const std::vector<int>& indices,
       pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &out);
      void ConvertPCLCloud2ColorSeg(const std::vector<std::vector<int> >& component,
       pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &out);

      bool convexBox(const pcl::PointXYZ& c0, const pcl::PointXYZ& c1,
        const pcl::Normal& n0, const pcl::Normal& n1);
      bool tablePlaneBinaryConditionFunction(const pcl::PointXYZRGBNormal& p0, 
        const pcl::PointXYZRGBNormal& p1, float dist);
      void computeNeighborsOrganized();
      void computeNeighborsUnOrganized();
      void segmentPlaneOrganized();
      void segmentPlaneUnOrganized();

      void findMicrowave_box();
  };

#endif
