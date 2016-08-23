#ifndef GRAPH_FILTER_H
#define GRAPH_FILTER_H

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

#include <omp.h>
#include <graph_filter/FitSize.h>
#include <Eigen/Dense>
#include <pcl/segmentation/extract_clusters.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/features/normal_3d.h>
#include <graph_filter/FindPlanes.h>
#include <pcl/common/intersections.h>
#include <Eigen/Geometry> 

#include <graph_filter/Constraints.h>
#include <graph_filter/NodeConstraints.h>
#include <graph_filter/EdgeConstraints.h>
#include <graph_filter/SizeConstraints.h>
#include <graph_filter/LocationConstraints.h>
#include <graph_filter/FingerConstraints.h>
#include <graph_filter/ArmFingerConstraints.h>
#include <graph_filter/Pca.h>
#include <graph_filter/Centroid.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <set>
#include <string>
#include <unordered_set>
#include <time.h> 

// #include <object_cls_msgs/BBox.h>
#include <hand_tracker_2d/HandBBox.h>

  class GraphFilter {
    public:
      GraphFilter();
      ~GraphFilter();
      // GraphFilter(const pcl::visualization::PCLVisualizer & pv);     
      void initialize();
      void segmentPlane();

      void setInputCloudAndTransform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_,
        const Eigen::Affine3d & T_Eigen);

      void setInputCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_);
      void setInputAndOrigCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_,
         pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr orig_cloud_);
      
      std::vector<std::vector<int> > planePointIndices;
      std::vector<std::vector<int> > component;      

      void computeNeighbors();
      void printNeighbors();
      void findConnectedComponent();
      void computePatchNorm(std::map<int, pcl::Normal>& PatchMeanN);
      void computeCentroid(std::map<int,  pcl::PointXYZ>& centroid);
      // boost::shared_ptr<std::vector<std::vector<bool> > > nbgh_matrix;
      //-----------------------///
      std::vector<std::vector<bool> > nbgh_matrix;
      std::vector<std::set<int> > nbgh_list;
      //map from subgraph vertex to scene vertice
      std::map<int, std::unordered_set<int> > sub2scene;
      // map from scene vertice to subgraph vertice 
      std::map <int, std::set<int> > scene2sub;
      void constraintsFiltering();
      void generateGraph();
      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
      boost::shared_ptr<pcl::visualization::CloudViewer> cloud_viewer;
      void cloudView(int remain);
      std::vector<hand_tracker_2d::HandBBox> getBBoxes();
      void setTimeStamp(ros::Time stamp_);
      std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > rays;
      std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f> > getRays();

      //-------------------------//

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
    private:
      pcl::search::Search<pcl::PointXYZRGBNormal>::Ptr searcher_;
      std::map<int, int> pointPlaneLabel;
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_;
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr orig_cloud_;

      bool use_box;
      int component_box_id;
      //------------------------------------------//

      boost::shared_ptr<Constraints> chooseConstraints(
        std::set<boost::shared_ptr<Constraints> >& c_set);
      void initializeMatching();

      boost::shared_ptr<Constraints> chooseConstraints(
        std::vector<boost::shared_ptr<Constraints> > &c_order);
      
      hand_tracker_2d::HandBBox generateBBox(int arm_end_ind, int fid);
      hand_tracker_2d::HandBBox generateBBoxFinger(int tip_ind, Eigen::Vector3f dir);
      void OptimizeConstraintOrder(std::vector<boost::shared_ptr<Constraints> >&
          c_order );
     void getTrainReward(boost::shared_ptr<Constraints> c,
         float & reward, float & cost);

     boost::shared_ptr<Constraints> getConstraints(int cind, 
       Centroid& centroid, Pca& pcav, const Eigen::Affine3d & T_Eigena,
       std::vector<std::set<int> > & nbgh_listc);

     std::vector<hand_tracker_2d::HandBBox> boxes;
       ros::Time frame_time_stamp;

      //------------------------------------------//

      bool tablePlaneUnaryConditionFunction(const pcl::PointXYZRGBNormal& p, const Eigen::Vector3f &gravity_vector, const float gravity_angle_thresh);

      void setSearcher();
      void setPointPlaneLabel();

      void ConvertPCLCloud2ColorSeg(const std::vector<int>& indices,
       pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &out);
      void ConvertPCLCloud2ColorSeg(const std::vector<std::vector<int> >& component,
       pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &out);

      bool tablePlaneBinaryConditionFunction(const pcl::PointXYZRGBNormal& p0, 
        const pcl::PointXYZRGBNormal& p1, float dist);
      void computeNeighborsOrganized();
      void computeNeighborsUnOrganized();
      void segmentPlaneOrganized();
      void segmentPlaneUnOrganized();

      // void findMicrowave_box();
  };

#endif
