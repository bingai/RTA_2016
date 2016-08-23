#ifndef ROTATIONAL_SYMMETRY_SUPPORTED_HPP
#define ROTATIONAL_SYMMETRY_SUPPORTED_HPP

// STD includes

// PCL includes
#include <pcl/io/pcd_io.h>

// CPP tools
#include "namaris/utilities/filesystem.hpp"

// Symmetry tools
#include "symmetry/rotational_symmetry.hpp"

namespace sym
{
  /** \brief Class representing a rotational symmetry and its support. */  
  template <typename PointT>
  class RotationalSymmetrySupported : public RotationalSymmetry
  {    
  public:
    // Empty constructor
    RotationalSymmetrySupported ()
      : RotationalSymmetry ()
      , support_ (new pcl::PointCloud<PointT>)
    { };
    
    // Copy constructor
    RotationalSymmetrySupported ( const RotationalSymmetry &symmetry, const pcl::PointCloud<PointT> &support)
      : RotationalSymmetry (symmetry)
    { 
      support_.reset(new pcl::PointCloud<PointT>);
      pcl::copyPointCloud<PointT>(support, *support_);
    };
    
    // Getters
    RotationalSymmetry getSymmetry  ()  const
    {
      return RotationalSymmetry(origin_, direction_);
    }
    
    typename pcl::PointCloud<PointT>::Ptr getSupport  ()  const
    {
      typename pcl::PointCloud<PointT>::Ptr support (new pcl::PointCloud<PointT>);
      pcl::copyPointCloud<PointT>(*support_, *support);
      
      return support;
    }    
    
    // Save
    bool save (const std::string &folder_name)  const
    {
      // Create folder 
      std::string parentDirname = utl::fs::getParentDir(folder_name);
      if (!utl::fs::exists(parentDirname))
      {
        std::cout << "[sym::RotationalSymmetrySupported::save] Parent folder does not exist ('" << parentDirname << "')." << std::endl;
        return false;
      }
      
      // Create support directory
      if (utl::fs::exists(folder_name))
        utl::fs::deleteDir(folder_name);
      if (!utl::fs::createDir(folder_name))
      {
        std::cout << "[sym::RotationalSymmetrySupported::save] Could not create symmetry directory ('" << folder_name << "')." << std::endl;
        return false;
      }

      // Create files
      std::string symmetryFilename = utl::fs::fullfile(folder_name, "symmetry.txt");
      std::string supportFilename = utl::fs::fullfile(folder_name, "support.pcd");
      
      if (pcl::io::savePCDFileBinary(supportFilename, *support_) == -1)
      {
        std::cout << "[sym::RotationalSymmetrySupported::save] Could not save support cloud 1 ('" << supportFilename << "')." << std::endl;
        return false;        
      }
      
      std::ofstream file(symmetryFilename);
      if (!file.is_open())
      {
        std::cout << "[sym::RotationalSymmetrySupported::save] Could not open symmetry file for writing ('" << symmetryFilename << "')." << std::endl;
        return false;
      }
      else
      {
        writeASCII(file);
      }
      file.close();
      
      return true;
    }
    
    // Load
    bool load (const std::string &folder_name)
    {
      //Check that folder exists
      if (!utl::fs::exists(folder_name))
      {
        std::cout << "[sym::RotationalSymmetrySupported::load] symmetry folder does not exist('" << folder_name << "')." << std::endl;
        return false;
      }
      
      // Read files
      std::string symmetryFilename = utl::fs::fullfile(folder_name, "symmetry.txt");
      std::string supportFilename = utl::fs::fullfile(folder_name, "support.pcd");
     
      if (pcl::io::loadPCDFile(supportFilename, *support_) == -1)
      {
        std::cout << "[sym::RotationalSymmetrySupported::load] Could not load support cloud 1 ('" << supportFilename << "')." << std::endl;
        return false;        
      }
      
      std::ifstream file(symmetryFilename);
      if (!file.is_open())
      {
        std::cout << "[sym::RotationalSymmetrySupported::load] Could not open symmetry file for reading ('" << symmetryFilename << "')." << std::endl;
        return false;
      }
      else
      {
        readASCII(file);
      }
      file.close();
      
      return true;
    }
        
  private:
    
    // Mermbers
    typename pcl::PointCloud<PointT>::Ptr support_;
  };
  
  /** \brief Visualize a rotational symmetry as a line segment centered at the symmetry origin point
   *  \param[in] visualizer object
   *  \param[in] symmetry rotational symmetry
   *  \param[in] id symmetry axis object id (default: symmetry)
   *  \param[in] length length of the line segment
   *  \param[in] line_width line segment width
   *  \param[in] color color of the line segment
   *  \param[in] opacity of the line segment
   */
  template <typename PointT>
  inline
  void showRotationalSymmetry ( pcl::visualization::PCLVisualizer &visualizer,
                                const RotationalSymmetrySupported<PointT> &symmetry,
                                const std::string id = "symmetry",
                                float length = 0.2,
                                float line_width = 2.0,
                                utl::pclvis::Color color = utl::pclvis::green,
                                float opacity = -1.0f
                              )
  {
    showRotationalSymmetry(visualizer, symmetry.getSymmetry(), id, length, line_width, color, opacity);
  }
  
}

#endif  // ROTATIONAL_SYMMETRY_SUPPORTED_HPP