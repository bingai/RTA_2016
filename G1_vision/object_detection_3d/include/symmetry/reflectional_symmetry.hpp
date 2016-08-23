#ifndef REFLECTIONAL_SYMMETRY_HPP
#define REFLECTIONAL_SYMMETRY_HPP

// STD includes
#include <math.h>
#include <fstream>

// PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>

// Boost includes
// #include <boost/make_shared.hpp>

// CPP tools
#include "utilities/math.hpp"
#include "utilities/pcl_visualization.hpp"

namespace symseg
{  
  /** \brief Class representing a reflectional symmetry in 3D space. A symmetry
   * is represented as a 3D plane.
   */  
  class ReflectionalSymmetry
  {
  public:
    
    /** \brief An empty constructor */
    ReflectionalSymmetry ()
      : origin_ (Eigen::Vector3f::Zero())
      , normal_ (Eigen::Vector3f::Zero())
      , distanceToOrigin_(0.0f)
    {};
    
    /** \brief A constructor from origin point and normal
     *  \param[in] origin origin point
     *  \param[in] normal symmetry plane normal
     */    
    ReflectionalSymmetry (const Eigen::Vector3f &origin, const Eigen::Vector3f &normal)
      : origin_ (origin)
      , normal_ (normal)
    {
      computeDistanceToOrigin();
      reorientNormal();
    };

    /** \brief A constructor from a weighted normal. Weighted normal is plane
     * normal weigted by the distance of the plane to the symmetry origin point
     *  \param[in] weighted_normal weighted normal
     */    
    ReflectionalSymmetry (const Eigen::Vector3f &weighted_normal)
    { 
      fromWeightedNormal(weighted_normal);
    };
    
    /** \brief Get the normal vector describing the symmetry
     *  \return normal of the symmetry plane
     */    
    Eigen::Vector3f getNormal () const { return normal_; }
    
    /** \brief Get the origin point of the symmetry
     *  \return symmetry origin point
     */        
    Eigen::Vector3f getOrigin () const { return origin_; }
    
    /** \brief Get distance to origin of the coordinate system (not the symmetry 
     * origin point)
     *  \return distance to the origin of the coordinate system
     */            
    float getDistanceToOrigin () const { return distanceToOrigin_; }

    /** \brief Get weighted normal describing the symmetry
     *  \return weighted normal
     */    
    Eigen::Vector3f toWeightedNormal () const {return normal_ * distanceToOrigin_; }
    
    /** \brief Set symmetry from a weighted normal
     *  \param[in] weighted_normal weighted normal
     */
    void fromWeightedNormal (const Eigen::Vector3f &weighted_normal)
    {
      distanceToOrigin_ = weighted_normal.norm();
      normal_ = weighted_normal / distanceToOrigin_;
      reorientNormal();
      origin_ = normal_ * distanceToOrigin_;
    }
    
    /** \brief Return the coefficients of equation of symmetry plane in the form
     * of ax + by + cz = d
     *  \return coefficients of a plane
     */
    Eigen::Vector4f toPlane () const 
    {
      Eigen::Vector4f plane;
      plane.head(3) = normal_;
      plane[3] = -distanceToOrigin_;
      return plane;
    }
    
    /** \brief Set the origin point of the symmetry
     *  \param[in] origin symmetry origin point
     */
    void setOrigin   (const Eigen::Vector3f origin ) { origin_  = origin;  }
    
    /** \brief Set the normal point of the symmetry
     *  \param[in] normal symmetry normal
     */    
    void setNormal  (const Eigen::Vector3f normal) { normal_ = normal; }

    /** \brief Generate a symmetry from two points
     *  \param[in] point1 first point
     *  \param[in] point2 second point
     */
    inline  
    void fromTwoPoints  ( const Eigen::Vector3f &point1, const Eigen::Vector3f &point2)
    {  
      Eigen::Vector3f origin = (point1 + point2) / 2;
      Eigen::Vector3f normal = point1 - point2;
      normal.normalize();
      *this = ReflectionalSymmetry(origin, normal);
    }
    
    /** \brief Transform the symmetry axis with an 3D rigid transformation.
     *  \param[in]  transform 3D affine transform
     */
    inline
    ReflectionalSymmetry transform  ( const Eigen::Affine3f &transform  ) const
    {
      return ReflectionalSymmetry(transform * origin_, transform.rotation() * normal_);
    }    
        
    /** \brief Calculate angle between normal 1 and normal 2 reflected by a symmetry
     *  \param[in] normal1 first normal
     *  \param[in] normal2 second normal
     *  \param[in] consistent_normals flag indicating if normals are consistently oriented or not (default true)
     *  \return angle (in radians) between two normals given the symmetry
     *  \note normals are assumed to be uniquely oriented (i.e. n and -n are different normals)
     */
    template <typename PointT>    // NOTE: this is only required so that there is no conflict with the next method
    inline  
    float reflectedNormalAngle  ( const Eigen::Vector3f &normal1,
                                  const Eigen::Vector3f &normal2,
                                  const bool &consisntent_normals = true
                                ) const
    { 
      Eigen::Vector3f normal2reflected = this->reflectNormal(normal2);
      float dotProd = utl::math::clampValue(normal1.dot(normal2reflected), -1.0f, 1.0f);
      if (consisntent_normals)
        return std::acos(dotProd);
      else
        return std::acos(std::abs(dotProd));
    }

    /** \brief Calculate angle between normal 1 and normal 2 reflected by a symmetry
     *  \param[in] point1 point containing first normal
     *  \param[in] point2 point containing second normal
     *  \param[in] consistent_normals flag indicating if normals are consistently oriented or not (default true)
     *  \return angle (in radians) between two normals given the symmetry
     */  
    template <typename PointT>
    inline  
    float reflectedNormalAngle  ( const PointT &point1,
                                  const PointT &point2,
                                  const bool &consisntent_normals = true
                                ) const
    {    
      return this->reflectedNormalAngle<PointT> ( point1.getNormalVector3fMap(), point2.getNormalVector3fMap(), consisntent_normals);
    }    
    
    /** \brief Write symmetry parameters to a filestream in ASCII format
     *  \param[in] file output filestream
     */
    void writeToFileASCII (std::ofstream &file) const
    {
      file << "point:\n";
      file << "   " << origin_[0] << "  " << origin_[1] << "  " << origin_[2] << "  " << std::endl;
      file << "normal:\n";
      file << "   " << normal_[0] << "  " << normal_[1] << "  " << normal_[2] << "  " << std::endl;
      file << "distance to origin:\n";
      file << "   " << distanceToOrigin_ << std::endl;
    }
    
    /** \brief Read symmetry parameters from an ASCII file
     *  \param[in] file input filestream
     */
    bool readFromFileASCII (std::ifstream &file)
    {
      std::string line;
      
      // Read point
      file >> line;
      if (line != "point:")
      {
        std::cout << "[symseg::ReflectionalSymmetry::readFromFileASCII] first line does not match the expected format." << std::endl;
        std::cout << "[symseg::ReflectionalSymmetry::readFromFileASCII] Line: " << line << std::endl;
        return false;
      }
      file >> origin_[0]; file >> origin_[1]; file >> origin_[2];

      // Read normal
      file >> line;
      file >> normal_[0]; file >> normal_[1]; file >> normal_[2];

      // Read distance to origin
      file >> line; file >> line; file >> line;
      file >> distanceToOrigin_;
      
      // Check that last chatacter is a new line
      char c;
      file.get(c);
      if (c != '\n')
      {
        std::cout << "[symseg::ReflectionalSymmetry::readFromFileASCII] Unexpected character at the end of symmetry data block. Probably something is corrupted." << std::endl;
        return false;
      }
      
      return true;
    }
    
    /** \brief Reflect a point around a symmetry plane
     *  \param[in] point original cloud
     *  \return reflected point
     */  
    inline
    Eigen::Vector3f reflectPoint (const Eigen::Vector3f &point) const
    {
      return (point - 2 * normal_ * (normal_.dot(point - origin_)));    
    }
    
    /** \brief Reflect a normal around a symmetry plane
     *  \param[in] normal original cloud
     *  \return reflected normal
     */  
    inline
    Eigen::Vector3f reflectNormal (const Eigen::Vector3f &normal) const
    {
      return (normal - 2 * (normal.dot(normal_) * normal_));
    }
    
    /** \brief Reflect a given pointcloud around a symmetry plane
     *  \param[in] cloud_in original cloud
     *  \param[in] cloud_out reflected cloud
     */
    template <typename PointT>
    inline
    void reflectCloud(const typename pcl::PointCloud<PointT> &cloud_in, typename pcl::PointCloud<PointT> &cloud_out)  const
    {
      // Prepare output cloud
      cloud_out.resize(cloud_in.size());
      
      for (size_t i = 0; i < cloud_in.size(); i++)
        cloud_out.points[i].getVector3fMap() = reflectPoint(cloud_in.points[i].getVector3fMap());
    }

    /** \brief Reflect a given pointcloud around a symmetry plane
     *  \param[in] cloud_in original cloud
     *  \param[in] cloud_out reflected cloud
     */
    template <typename PointT>
    inline
    void reflectCloudWithNormals(const typename pcl::PointCloud<PointT> &cloud_in, typename pcl::PointCloud<PointT> &cloud_out) const
    {
      pcl::copyPointCloud(cloud_in, cloud_out);   // NOTE: why do we need to copy the cloud here? Why not just resize the cloud?
          
      for (size_t i = 0; i < cloud_in.size(); i++)
      {
        cloud_out.points[i].getVector3fMap()        = reflectPoint  (cloud_in.points[i].getVector3fMap());
        cloud_out.points[i].getNormalVector3fMap()  = reflectNormal (cloud_in.points[i].getNormalVector3fMap());
      }
    }
    
  private:

    /** \brief Reoerient normal such that it's x coordinate is non-negative */
    void reorientNormal()
    {
      if (normal_[0] < 0)
      {
        normal_ = -normal_;
        distanceToOrigin_ = -distanceToOrigin_;
      }
    }
    
    /** \brief Compute the distance from the symmety plane to the coordinate system origin */
    void computeDistanceToOrigin ()
    {
      distanceToOrigin_ = origin_.dot(normal_);
    }
        
    // Member variables
    Eigen::Vector3f origin_;  ///< Point belonging to the symmetry plane. Symmetry is visualized around this point.
    Eigen::Vector3f normal_;  ///< Normal of the symmetry plane. Note that normal is always reoriented such that it's x coordinate is non-negative
    float distanceToOrigin_;  ///< Distance of the symmetry plane to the origin of the coordinate system. 
  };

  /** \brief Vetcor of reflectional symmetries */
  typedef std::vector<ReflectionalSymmetry>  ReflectionalSymmetries;
  
  /** \brief Print symmetry details to ostream */
  std::ostream& operator<< ( std::ostream& os, const ReflectionalSymmetry& symmetry)
  {
    os << "origin:             " << symmetry.getOrigin().transpose();
    os << std::endl;
    os << "normal:             " << symmetry.getNormal().transpose();
    os << std::endl;
    os << "Distance to origin: " << symmetry.getDistanceToOrigin();
      
    return os;
  }  
    
  /** \brief Given two symmetries calculate the angle between their normals and 
   *  difference between their distance to origin.
   *  \param[in] sym1 first symmetry
   *  \param[in] sym2 second symmetry
   *  \param[out] normal_angle_diff angle between symmetry normals
   *  \param[out] origin_dist_diff difference between distance to origin
   *  \note we assume that both symmetries normals are consistently oriented
   */  
  inline
  void symmetryDifference (const ReflectionalSymmetry &sym1, const ReflectionalSymmetry &sym2, float &normal_angle_diff, float &origin_dist_diff)
  {
    // Get angle between normals
    float dotProd = sym1.getNormal().dot(sym2.getNormal());
    dotProd = std::min(1.0f, dotProd);
    dotProd = std::max(-1.0f, dotProd);
    normal_angle_diff = std::acos(dotProd);
    
    // Get origin distance difference
    origin_dist_diff = std::abs(sym1.getDistanceToOrigin() - sym2.getDistanceToOrigin());
  }
  
  /** \brief Visualize symmetry as a rectangular polygon
   *  \param[in] visualizer object
   *  \param[in] symmetry reflectional symmetry
   *  \param[in] id symmetry plane object id (default: symmetry)
   *  \param[in] side_width side length of the symmetry plane square
   */
  void showReflectionalSymmetry ( pcl::visualization::PCLVisualizer &visualizer,
                                  const ReflectionalSymmetry &symmetry,
                                  const std::string id = "symmetry",
                                  float side_width = 0.05,
                                  utl::pclvis::Color color = utl::pclvis::Color(0.0, 1.0, 0.0),
                                  float opacity = -1.0f
                                )
  {
    // Get symmetry plane parameters
    Eigen::Vector3f origin   = symmetry.getOrigin();
    Eigen::Vector3f normal  = symmetry.getNormal();
    
    // Show plane
    utl::pclvis::showPlane(visualizer, origin, normal, id, side_width, color, opacity);
  }
}

#endif // REFLECTIONAL_SYMMETRY_HPP
