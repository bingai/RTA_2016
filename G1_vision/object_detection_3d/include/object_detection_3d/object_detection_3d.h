#ifndef OBJECT_DETECTION_3D_H
#define OBJECT_DETECTION_3D_H

// OpenCV
#include <opencv2/core/core.hpp>

// PCL includes
#include <pcl/point_cloud.h>

// Utilities
#include <namaris/utilities/pcl_typedefs.hpp>

// Symmetry
#include <symmetry/rotational_symmetry.hpp>

namespace g1
{
  namespace vision
  {
    struct Cylinder
    {
      Eigen::Affine3f pose_;
      float radius_;
      float height_;
      std::string id_;
    };

    struct Box
    {
      Eigen::Affine3f pose_;
      Eigen::Vector3f size_;
      std::string id_;
    };
    
    /** \brief Add a box to the visualizer.
     *  \param[in]  visualizer  visualizer object
     *  \param[in]  box         box
     *  \param[in]  id          point cloud object id prefix (default: cloud)
     *  \param[in]  color       color of the cloud
     *  \param[in]  opacity     opacity of the cloud
     */
    void showBox  ( pcl::visualization::PCLVisualizer &visualizer,
                    const g1::vision::Box &box,
                    const std::string &id = "box",
                    utl::pclvis::Color color = utl::pclvis::Color(),
                    const float opacity = -1.0f
                  );
    
    /** \brief Add a cylinder to the visualizer.
     *  \param[in]  visualizer  visualizer object
     *  \param[in]  cylinder    cylinder
     *  \param[in]  id          point cloud object id prefix (default: cloud)
     *  \param[in]  color       color of the cloud
     *  \param[in]  opacity     opacity of the cloud
     */
    void showCylinder ( pcl::visualization::PCLVisualizer &visualizer,
                        const g1::vision::Cylinder &cylinder,
                        const std::string &id = "cylinder",
                        utl::pclvis::Color color = utl::pclvis::Color(),
                        const float opacity = -1.0f
                      );
        
    /** \brief Unary function for table points
     *  \param[in] p input point
     *  \param[in] gravity_vector gravity vector
     *  \param[in] gravity_angle_thresh maximum allowed angular difference between the gravity vector and the negative of the point normal
     *  \param[out] tabletop_cloud tabletop cloud
     */
    bool tablePlaneUnaryConditionFunction ( const PointNC& p, const Eigen::Vector3f &gravity_vector, const float gravity_angle_thresh);

    /** \brief Segment table
     *  \param[in] scene_cloud input cloud
     *  \param[out] tabletop_cloud tabletop cloud
     *  \param[out] table_plane_coefficients coefficients of the table plane
     *  \param[out] table_plane_hull convex hull of the table plane
     *  \param[in]  visualize if TRUE internal visualization of the results will be shown. Note that function will not exit until visualiztion is closed.
     *  \return TRUE if table plane was found, FALSE otherwise
     */
    bool segmentTable ( const pcl::PointCloud<PointNC>::ConstPtr &scene_cloud,
                        pcl::PointCloud<PointNC>::Ptr &tabletop_cloud,
                        Eigen::Vector4f &table_plane_coefficients,
                        pcl::PointCloud<PointNC>::Ptr &table_plane_hull,
                        const bool visualize = false
                      );

    /** \brief Inner fridge box segmentation
     *  \param[in] scene_cloud input cloud
     *  \param[out] fridge_cloud fridge interior cloud
     *  \param[out] fridge_box fridge inner box desriptor
     *  \param[in]  visualize if TRUE internal visualization of the results will be shown. Note that function will not exit until visualiztion is closed.
     *  \return TRUE if fridge inner box was found, FALSE otherwise
     */
    bool segmentFridgeInterior( const pcl::PointCloud<PointNC>::ConstPtr &scene_cloud,
                                pcl::PointCloud<PointNC>::Ptr &fridge_cloud,
                                g1::vision::Box &fridge_box,
                                const bool visualize = false
                              );

    /** \brief Segment a tebletop pointcloud into segments. Two segmentations
     * are provided:
     *  1)  First segmentation finds closely connected sets of points. Since we 
     *      are assuming that out objects are not touching these segments should
     *      include all points belonging to the objects.
     *  2)  Second segmentation finds smoothly varying surfaces in the
     *      pointcloud. These segments are used for detecting rotationaly
     *      symmetric objects.
     *  \param[in]  tabletop_cloud tabletop cloud
     *  \param[out] segments_euclidean  segments euclidean
     *  \param[out] segments_smooth     smooth segments
     *  \param[in]  visualize if TRUE internal visualization of the results will be shown. Note that function will not exit until visualiztion is closed.
     */
    bool segmentObjects ( const pcl::PointCloud<PointNC>::ConstPtr &tabletop_cloud,
                          utl::map::Map &segments_euclidean,
                          utl::map::Map &segments_smooth,
                          const bool visualize = false
                        );

    /** \brief Detect rotationally symmetric objects in the input pointcloud
     * segments.
     *  \param[in]  segments vector of object segment point indices
     *  \param[in]  table_plane_coefficients coeffcients of the table plane
     *  \param[out] valid_segments indices of segments that are rotationally symmetric
     *  \param[out] bounding_cylinders bounding 
     *  \param[in]  symmetries_supported  rotational symmetries with the segments used to generate them
     *  \param[in]  visualize if TRUE internal visualization of the results will be shown. Note that function will not exit until visualiztion is closed.
     */
    bool rotationalObjectDetection  ( const pcl::PointCloud<PointNC>::ConstPtr &tabletop_cloud,
                                      const utl::map::Map &segments,
                                      const Eigen::Vector4f &table_plane_coefficients,
                                      std::vector<sym::RotationalSymmetry> &object_symmetries,
                                      std::vector<Cylinder> &object_bounding_cylinders,
                                      std::vector<std::vector<int> > &object_segments,
                                      const bool visualize = false
                                    );

    /** \brief Detect mug handles for detected rotationally symmetric objects.
     *  \param[in]  tabletop_cloud cloud corresponding to objects on the table
     *  \param[in]  table_plane_coefficients coeffcients of the table plane
     *  \param[in]  object_segments indices of points belonging to individual rotationally symmetric objects
     *  \param[in]  object_symmetries symmetries of rotationally symmetric objects 
     *  \param[in]  object_bounding_cylinders bounding cylinders of rotationally symmetric objects
     *  \param[out] object_bounding_cylinders bounding cylinders of rotationally symmetric objects
     *  \param[out] object_handle_bounding_boxes a vector of bounding boxed for detected handles. Same size as number of ogjects.
     *  \param[in]  visualize if TRUE internal visualization of the results will be shown. Note that function will not exit until visualiztion is closed.
     */
    bool handleDetection  ( const pcl::PointCloud<PointNC>::ConstPtr &tabletop_cloud,
                            const Eigen::Vector4f &table_plane_coefficients,
                            const utl::map::Map &object_segments,
                            const std::vector<sym::RotationalSymmetry> &object_symmetries,
                            const std::vector<Cylinder> &object_bounding_cylinders,
                            std::vector<Box> &object_handle_bounding_boxes,
                            const bool visualize = false
                          );    
    
    /** \brief Fit tight bounding boxes to the input pointcloud segments.
     *  \param[in]  segments vector of object segment pointclouds
     *  \param[in]  table_plane_coefficients coeffcients of the table plane
     *  \param[in] box_segments_ind indices of segments to fit boxes to
     *  \param[out] bounding_boxes bounding box descriptors
     */
    bool fitBoundingBoxes ( const pcl::PointCloud<PointNC>::ConstPtr &tabletop_cloud,
                            const utl::map::Map &segments,
                            const Eigen::Vector4f &table_plane_coefficients,
                            const std::vector<int> &box_segments_ind,
                            std::vector<Box> &bounding_boxes,
                            bool visualize = false
                          );

    Eigen::MatrixXf projectPointCloudToImageView  ( const pcl::PointCloud<PointNC>::ConstPtr &cloud_in,
                                                    const cv::Mat &intrinsics,
                                                    const cv::Mat &distortion,
                                                    const Eigen::Matrix4f &tform = Eigen::Matrix4f::Identity()
                                                  );

    void boundingBox2DCorners(Eigen::MatrixXf pts, int &x1, int &y1, int &x2, int &y2);
  }
}





#endif    // OBJECT_DETECTION_3D_H