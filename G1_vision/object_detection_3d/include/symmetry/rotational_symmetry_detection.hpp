#ifndef ROTATIONAL_SYMMETRY_DETECTION_HPP
#define ROTATIONAL_SYMMETRY_DETECTION_HPP

// Eigen
#include "unsupported/Eigen/NonLinearOptimization"

// Utilities
#include <namaris/utilities/geometry.hpp>

// Symmetry
#include <symmetry/rotational_symmetry.hpp>

namespace sym
{  
  //----------------------------------------------------------------------------
  // Datatypes
  //----------------------------------------------------------------------------

  /** \brief Datatype storing a 3D point and it's normal.
   * ( p_x, p_y, p_z, n_x, n_y, n_z )
   */
  typedef Eigen::Matrix<float, 6, 1> PointNormal;
  typedef std::vector<PointNormal, Eigen::aligned_allocator<PointNormal> > PointNormalVector;

  //----------------------------------------------------------------------------
  // Datatype conversion
  //----------------------------------------------------------------------------

  /** \brief Convert a PCL point (with a normal) to a \ref{PointNormal}.
   *  \param[in]  point_pcl  PCL point
   *  \return     PointNormal
   */
  template <typename PointT>
  PointNormal pointPCL2PointNormal (const PointT &point_pcl)
  {
    PointNormal point_normal;
    point_normal.head (3) = point_pcl.getVector3fMap ();
    point_normal.tail (3) = point_pcl.getNormalVector3fMap ();
    return point_normal;
  }

  /** \brief Convert a \ref{PointNormal} to a PCL point (with a normal).
   *  \param[in]  point_normal  PointNormal
   *  \return     PCL point
   */
  template <typename PointT>
  PointT pointNormal2PointPCL (const PointNormal &point_normal)
  {
    PointT point_pcl;
    point_pcl.getVector3fMap ()       = point_normal.head (3);
    point_pcl.getNormalVector3fMap () = point_normal.tail (3);
    return point_pcl;
  }

  //----------------------------------------------------------------------------
  // Symmetry scoring functions
  //----------------------------------------------------------------------------
  
  /** \brief Calculate the error of fit between a symmetry axis and an oriented
    * point. Error of fit is calculated as the sine of the angle between the
    * point normal and the plane containing the symmetry axis and the point
    * itself. The final score is in the [0, 1] range. Lower values indicate
    * smaller error i.e. a better fit.
    * Optinonally fit errors are clamped to a specified threshold.
    *  \param[in]  symmetry                symmetry axis
    *  \param[in]  point                   point coordinate
    *  \param[in]  normal                  point normal
    *  \param[in]  max_fit_error           maximum fitness score threshold [0; 1] (default = 1.0)
    *  \return point symmetry error of fit
    */
  inline
  float pointRotationalSymmetryFitError ( const RotationalSymmetry &symmetry,
                                          const Eigen::Vector3f     &point,
                                          const Eigen::Vector3f     &normal,
                                          const float max_fit_error = 1.0f
                                        )
  {
    Eigen::Vector3f pointProjected = symmetry.projectPoint (point);
    Eigen::Vector3f planeNormal = (point - pointProjected).cross (symmetry.getDirection ());
   
    // Calculate angle sine
    float fit_error = std::abs (planeNormal.dot (normal) / planeNormal.norm ());
    
    // Limit angle sine and return
    return std::min (fit_error, max_fit_error);
  }  
  
  /** \brief Calculate the error of fit between a symmetry axis and a
   * pointcloud. Error of fit is calculated as the average sine of the angles
   * formed by the individual point normals and the planes containing the
   * symmetry axis and the points themselves. The final score is in the [0, 1]
   * range. Lower values indicate smaller error i.e. a better fit.
   * Optinonally fit errors are clamped to a specified threshold.
   *  \param[in]  cloud                   input cloud
   *  \param[in]  symmetry                input symmetry
   *  \param[in]  max_fit_error           maximum fitness score for a point [0; 1] (default = 1.0)
   *  \return pointcloud symmetry error of fit
   */
  template <typename PointT>
  inline
  float cloudRotationalSymmetryFitError ( const sym::RotationalSymmetry &symmetry,
                                          const pcl::PointCloud<PointT> &cloud,
                                          const float max_fit_error = 1.0f
                                        )
  {
    // Check that cloud is non-zero
    if (cloud.size() == 0)
    {
      std::cout << "[sym::cloudRotationalSymmetryFitError] input cloud is empty!" << std::endl;
      return -1.0f;
    }
    
    // Count the number of non-parallel lines and line-to-line distance
    float fit_error = 0.0;
    for (size_t pointId = 0; pointId < cloud.size(); pointId++)
    {
      Eigen::Vector3f point   = cloud.points[pointId].getVector3fMap();
      Eigen::Vector3f normal  = cloud.points[pointId].getNormalVector3fMap();
      
      // Calculate score
      fit_error += pointRotationalSymmetryFitError (symmetry, point, normal, max_fit_error);
    }
    
    // Normalize and return
    return (fit_error / static_cast<float>(cloud.size ()));
  }

//   /** \brief Calculate how perpendicular a point normal is to the symmetry axis.
//    * Perpendicularity is calculated as one minus the absolute value of the
//    * cosine between the point normal and the symmetry axis direction.
//    * Optionally the score can be clamped so that if the angle between the normal
//    * and symmetry axis is greater than a threshold it is assigned the score of 
//    * 1, while 
//    * pointcloud.
//    *  \param[in]  cloud                   input cloud
//    *  \param[in]  symmetry                input symmetry
//    *  \param[out] fitness_score           fitness score
//    *  \param[in]  perpendicular_angle_threshold    minimum angle between point normal and symmetry axis that are considered perpendicular (default = M_PI / 4)
//    */
//   template <typename PointT>
//   inline
//   float pointRotationalSymmetryPerpendicularity_old ( const sym::RotationalSymmetry &symmetry,
//                                                   const Eigen::Vector3f normal,
//                                                   const float perpendicular_angle_cos_threshold = 0.0f
//                                                 )
//   {
//     // If perpendicular threshold cos is 1.0f than any angle is 
//     if (perpendicular_angle_cos_threshold == 1.0f)
//       return 1.0f;
//     
//     // Get perpendicular score
//     float symAxisNormalDot  = std::abs (symmetry.getDirection ().dot (normal));
//     float symAxisNormalAngle = std::acos(symAxisNormalDot);
//     float perpendicularScore = 1 - symAxisNormalAngle / (M_PI/2);
//     
//     // Normalize the score
//     perpendicularScore /= (1.0f - perpendicular_angle_cos_threshold);
//     perpendicularScore = std::min(perpendicularScore, 1.0f);
//     
//     return perpendicularScore;
//   }
  
  /** \brief Calculate how perpendicular a point normal is to the symmetry axis.
   * Perpendicularity is calculated as the angle between the symmetry axis and
   * the point normal normalized by PI/2. The final score is in the [0, 1]
   * range. Higher values indicate higher perpendicularity.
   * Optinonally the score can be clamped such that angles greater than a
   * threshold get a score of 1 and the rest are normalized to [0, 1] range.
   *  \param[in]  symmetry        input symmetry
   *  \param[in]  normal          point normal
   *  \param[in]  angle_threshold angle threshold used for clamping
   *  \return point perpendicularity score
   */
  inline
  float pointRotationalSymmetryPerpendicularity ( const sym::RotationalSymmetry &symmetry,
                                                  const Eigen::Vector3f normal,
                                                  const float angle_threshold = M_PI / 2
                                                )
  {
    // If any angle greater than 0 get a score of 1, then all angles will get a score of 1.
    if (angle_threshold == 0.0f)
      return 1.0f;
    
    // Get perpendicular score
    float symAxisNormalAngle = utl::geom::lineLineAngle(symmetry.getDirection(), normal);
    
    // Normalize by angle threshold
    symAxisNormalAngle /= angle_threshold;
    symAxisNormalAngle = std::min (symAxisNormalAngle, 1.0f);
    
    return symAxisNormalAngle;
  }  
  
  /** \brief Calculate how perpendicular a pointcloud is to the symmetry axis.
   * Perpendicularity is calculated as the average angle between the symmetry
   * axis and the point normals normalized by PI/2. The final score is in the
   * [0, 1] range. Higher values indicate higher perpendicularity.
   * Optinonally the score can be clamped such that angles greater than a
   * threshold get a score of 1 and the rest are normalized to [0, 1] range.
   *  \param[in]  symmetry        input symmetry
   *  \param[in]  cloud           input cloud
   *  \param[in]  normal          point normal
   *  \param[in]  angle_threshold angle threshold used for clamping
   *  \return cloud perpendicularity score
   */
  template <typename PointT>
  inline
  float cloudRotationalSymmetryPerpendicularity ( const sym::RotationalSymmetry &symmetry,
                                                  const pcl::PointCloud<PointT> &cloud,
                                                  const float angle_threshold = M_PI / 2
                                                )
  {
    // Check that cloud is non-zero
    if (cloud.size() == 0)
    {
      std::cout << "[sym::cloudRotationalSymmetryPerpendicularity] input cloud is empty!" << std::endl;
      return -1.0f;
    }
    
    // Calculate perpendicularity
    float perpendicular_score = 0.0f;
    
    for (size_t pointId = 0; pointId < cloud.size (); pointId++)
      perpendicular_score += pointRotationalSymmetryPerpendicularity(symmetry, cloud.points[pointId].getNormalVector3fMap (), angle_threshold);
    
    // Normalize and return
    return (perpendicular_score / static_cast<float>(cloud.size()));
  }
  
  /** \brief Get the angle measuring how much the pointcloud "wraps" around 
   * the symmetry axis. It is calculated as 2*pi - the maximum angular step
   * between adjacent points of the pointcloud. The maximum angular step is
   * computed as:
   * 1. For each point find the vector that goes from point projected on the 
   *    symmetry axis to the point itself.
   * 2. Choose a random vector.
   * 3. Compute the clockwise angles between the random vector and all other
   *    vectors.
   * 4. Sort the angles in increasing order.
   * 5. Find the largest step between two adjacent angles.
   *  \param[in]  cloud                   input cloud
   *  \param[in]  symmetry                input symmetry
   *  \return largest angular step
   */
  template <typename PointT>
  float cloudRotationalSymmetryCoverageAngle  ( const sym::RotationalSymmetry &symmetry, const pcl::PointCloud<PointT> &cloud )
  {
    // If cloud is empty - make a warning and return -1.
    if (cloud.empty ())
    {
      std::cout << "[sym::cloudRotationalSymmetryCoverageAngle] pointcloud is empty! Returning -1." << std::endl;
      return -1.0f;
    }
    
    // If there is a single point - return 360 degrees.
    if (cloud.size () == 1)
    {
      0.0f;
    }      
    
    // Get reference vector
    Eigen::Vector3f referenceVector = cloud.points[0].getVector3fMap () - symmetry.projectPoint(cloud.points[0].getVector3fMap ());
    
    // Find angles between vectors formed by all other points and current vector.
    std::vector<float> angles (cloud.size ());
    angles[0] = 0.0f;
    for (size_t pointId = 1; pointId < cloud.size (); pointId++)
    {
      Eigen::Vector3f curVector = cloud.points[pointId].getVector3fMap () - symmetry.projectPoint(cloud.points[pointId].getVector3fMap ());
      angles[pointId] = utl::geom::vectorAngleCW<float>(referenceVector, curVector, symmetry.getDirection ());
    }
      
    // Sort the angles
    std::sort(angles.begin(), angles.end());
    
    // Get angle difference
    std::vector<float> angleDifference(angles.size());
    for (size_t i = 1; i < angles.size(); i++)
      angleDifference[i] = utl::geom::angleDifferenceCCW(angles[i-1], angles[i]);
    angleDifference[0] = utl::geom::angleDifferenceCCW(angles[angles.size()-1], angles[0]);
    
    return (2.0f * M_PI) - utl::stdvec::vectorMax(angleDifference);
  }
  
  //----------------------------------------------------------------------------
  // Optimization functors
  //----------------------------------------------------------------------------

  /** \brief Base functor for non-linear optimization with Eigen. All the models
   * that need non linear optimization must define their own one and implement
   * either of:
   *   operator() (const Eigen::VectorXd& x, Eigen::VectorXd& fvec)
   *   operator() (const Eigen::VectorXf& x, Eigen::VectorXf& fvec)
   * dependening on the choosen _Scalar
   */
  template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
  struct BaseFunctor
  {
    typedef _Scalar Scalar;
    
    enum
    {
      InputsAtCompileTime = NX,
      ValuesAtCompileTime = NY
    };
    
    typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
    typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
    typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

    int m_inputs, m_values;

    BaseFunctor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
    BaseFunctor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

    int inputs() const { return m_inputs; }
    int values() const { return m_values; }
  };
  
  /** \brief Given a set of 3D oriented points and an initial 3D rotational
   * symmetry axis find the true symmetry axis. This is done by minimizing the 
   * symmetry fitness score. Given a point and a symmetry axis, symmetry fitness
   * score is calculated as the sine of the angle between the point normal and 
   * the plane containing the symmetry axis and the point.
   * 
   * Both points and symmetry axis are stored in \ref{PointNormal} objects.
   * Symmetry is represented using a point on the symmetry axis and a direction
   * vector. Note that the direction vector can be non-unit.
   */
  struct SymDetectFunctorAxisFixed : BaseFunctor<float>
  {
    /** \brief Empty constructor */
    SymDetectFunctorAxisFixed ()
      : max_point_fitness_score_ (1.0f)
    {};
    
    
    /** \brief Compute fitness for each input point.
     *  \param[in]  x symmetry axis
     *  \param[out] fvec error vector
     */
    int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
    {
      // Get current rotational symmetry
      RotationalSymmetry symmetry (x, sym_axis_direction_);
      
      // Compute fitness
      for(size_t i = 0; i < this->points_.size(); i++)
      {        
        fvec(i) = pointRotationalSymmetryFitError ( symmetry,
                                                    this->points_[i].head (3),
                                                    this->points_[i].tail (3),
                                                    max_point_fitness_score_  );
      }
      return 0;
    }
    
    /** \brief Input points. */
    PointNormalVector points_;
    
    /** \brief Symmetry axis direction. */
    Eigen::Vector3f sym_axis_direction_;
    
    /** \brief Maximum fitness score of a point. */
    float max_point_fitness_score_;
    
    /** \brief Dimensionality of the optimization parameter vector. */
    int inputs() const { return 3; }
    
    /** \brief Number of points. */
    int values() const { return this->points_.size (); }
  };

  struct SymDetectFunctorAxisFixedDiff : Eigen::NumericalDiff<SymDetectFunctorAxisFixed> {};  
  
  //----------------------------------------------------------------------------
  // Rotational symmetry detection
  //----------------------------------------------------------------------------
  
  /** \brief Detect rotational symmetrty in an input pointcloud with normals. 
   * The direction of the symmetry axis is fixed
   * 
   *  \param[in]  cloud                   input cloud
   *  \param[out] symmetry                final symmetry
   *  \param[out] fitness_score           final symmetry fitness score
   *  \param[out] perpendicular_score     final symmetry perpendicular score
   *  \param[in]  max_point_fitness_score maximum point fitness score [0; 1] (default = 1.0)
   *  \param[in]  perpendicular_angle_threshold    minimum angle between point normal and symmetry axis that are considered perpendicular (default = M_PI / 4)
   *  \param[in]  min_non_parallel_score  minimal perpendicularity score for a valid symmetry axis [0; 1] (default = 0.5)
   */
  template <typename PointT>
  inline
  void detectRotationalSymmetry ( const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                  sym::RotationalSymmetry &symmetry,
                                  Eigen::Vector3f &symmetry_direction,
                                  float &fitness_score,
                                  float &perpendicular_score,
                                  const float max_point_fitness_score = 1.0f,
                                  const float perpendicular_angle_threshold = M_PI / 4
                                )
  {  
    //----------------------------------------------------------------------------
    // Prepare intial symmetries and input data
    
    // Get initial symmetries from the major axes of the input cloud
    pcl::PCA<PointT> pcaSolver;
    pcaSolver.setInputCloud (cloud);
    Eigen::Vector3f cloud_mean = pcaSolver.getMean().head (3);

    // Generate lines from cloud points and their normals
    int numPoints = cloud->size();
    sym::PointNormalVector points (numPoints);
    for (size_t pointId = 0; pointId < numPoints; pointId++)
      points[pointId] = sym::pointPCL2PointNormal<PointT> (cloud->points[pointId]);
    
    //----------------------------------------------------------------------------
    // Optimize!
    
    // Create optimization object
    sym::SymDetectFunctorAxisFixedDiff functor;
    functor.points_ = points;
    functor.max_point_fitness_score_ = max_point_fitness_score;
    functor.sym_axis_direction_ = symmetry_direction;
    Eigen::LevenbergMarquardt<sym::SymDetectFunctorAxisFixedDiff, float> lm(functor);
    lm.parameters.ftol = 1e-12;
    lm.parameters.maxfev = 800;
    
    // Minimize
    Eigen::VectorXf x = cloud_mean;
    int status = lm.minimize (x);
    
    // Get final symmetry
    symmetry = RotationalSymmetry (x, symmetry_direction);
    symmetry.setOriginProjected (cloud_mean);
    
    // Score final symmetry
    fitness_score = cloudRotationalSymmetryFitError<PointT>  ( symmetry,
                                                              *cloud,
                                                              max_point_fitness_score
                                                            );
    
    perpendicular_score = cloudRotationalSymmetryPerpendicularity<PointT> ( symmetry,
                                                                            *cloud,
                                                                            perpendicular_angle_threshold
                                                                          );
  }  
}

#endif    // ROTATIONAL_SYMMETRY_DETECTION_HPP