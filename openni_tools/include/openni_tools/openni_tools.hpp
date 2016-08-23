#ifndef OPENNI2_TOOLS_HPP
#define OPENNI2_TOOLS_HPP

// STD includes
#include <iostream>
#include <deque>

// OpenCV includes
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Utilities
#include <namaris/utilities/filesystem.hpp>
#include <namaris/utilities/std_vector.hpp>
#include <namaris/utilities/opencv_calibration.hpp>
#include <namaris/utilities/string.hpp>

#if (defined(CV_VERSION_EPOCH) && CV_VERSION_EPOCH < 3) || CV_VERSION_MAJOR < 3
  #define USE_CUSTOM_IMCODECS
#endif

#ifdef USE_CUSTOM_IMCODECS
  #include <openni_tools/custom_imcodecs.hpp>
  #define imwrite(x,y) custom_cv::imwrite(x,y)
#else
  #define imwrite(x,y) cv::imwrite(x,y)
#endif

namespace utl
{
  namespace kinect
  {
    //--------------------------------------------------------------------------
    // Kinect image I/O
    //--------------------------------------------------------------------------
    
    /** \brief Image types from a Kinect camera */
    enum ImageType { RGB, DEPTH, IR };
    static const char * ImageTypeStrings[] = { "rgb", "depth", "ir" };

    /** \brief Generate a filename given for a frame
     *  \param[in]  frame_number  frame number of an image
     *  \param[in]  image_type    type of the image to be saved
     *  \return filename for the frame
     */
    inline
    std::string generateFilename(const int frame_id, const ImageType image_type, const std::string &prefix = "capture")
    {
      std::string filename = prefix + "_" + utl::str::to_padded_string(frame_id, 3) + "_";
      
      switch (image_type)
      {
        case RGB:
          filename += "rgb";
          break;
          
        case DEPTH:
          filename += "depth";
          break;
          
        case IR:
          filename += "ir";
          break;
          
        default:
          std::cout << "Unknown image type" << std::endl;
          filename += "WTF";
      }
      
      filename += ".png";
      
      return filename;
    }

    /** \brief Given a filename figure out if it is a valid frame and if so what
     * is it's image type and frame number
     *  \param[in]  filename      name of the file
     *  \param[out] frame_id      id of the frame to be saved
     *  \param[out] image_type    type of the image to be saved
     *  \return TRUE if a file for the specified frame exists
     */
    inline
    bool getImageInfo(const std::string &filename, int &frame_id, ImageType &image_type, const std::string &prefix = "capture")
    {      
      // Check if prefix is correct
      if (filename.substr(0, prefix.length()) != prefix)
        return false;
      
      // Get image type
      int startPos = filename.find("_", prefix.length()+1, 1) + 1;
      int endPos   = filename.find(".png");
      std::string imageTypeString = filename.substr(startPos, endPos-startPos);
      
      if (imageTypeString == ImageTypeStrings[RGB])
        image_type = RGB;
      else if (imageTypeString == ImageTypeStrings[DEPTH])
        image_type = DEPTH;
      else if (imageTypeString == ImageTypeStrings[IR])
        image_type = IR;
      else
      {
        std::cout << "[utl::kinect::getImageInfo] unknown image type postfix '" << imageTypeString << "'." << std::endl;
        return false;
      }
      
      // Get frame ID
      startPos = prefix.length()+1;
      endPos = filename.find("_", startPos, 1);
      frame_id = std::stoi(filename.substr(startPos, endPos-startPos));
      
      return true;
    }

    /** \brief Write a frame of specified type, frame id and size
     *  \param[in]  dirname     path to directory where frame should be saved
     *  \param[in] frame_id     frame id
     *  \param[in] image_type   frame image type
     *  \param[out] image       image
     *  \return TRUE if image was saved successfully
     */
    inline
    bool writeFrame ( const std::string &dirname,
                      const int frame_id,
                      const ImageType image_type,
                      cv::Mat &image,
                      const std::string &prefix = "frame"
                    )
    {
      std::string filename = generateFilename(frame_id, image_type, prefix);
      filename = utl::fs::fullfile(dirname, filename);
      
      if (!imwrite(filename, image))
      {
        std::cout << "[utl::kinect::writeFrame] couldn't save image to file '" << filename << "'." << std::endl;
        return false;
      }
      
      return true;
    }
    
    /** \brief Read a frame of specified type, frame id and size
     *  \param[in]  dirname     path to directory containing the file
     *  \param[in] frame_id     frame id
     *  \param[in] image_type   frame image type
     *  \param[out] image       image
     *  \return TRUE if image was loaded successfully
     */
    inline
    bool readFrame  ( const std::string &dirname,
                      const int frame_id,
                      const ImageType image_type,
                      cv::Mat &image
                    )
    {
      // Read image
      std::string filename = generateFilename(frame_id, image_type);
      image = cv::imread(utl::fs::fullfile(dirname, filename), CV_LOAD_IMAGE_ANYDEPTH+CV_LOAD_IMAGE_ANYCOLOR);
      
      if (image.empty())
        return false;
      
      return true;
    }
    
    /** \brief Generate a filename given for an oni capture.
     *  \param[in]  capture_id  id of the capture file
     *  \return filename for the oni file
     */
    inline
    std::string generateOniFilename(const int capture_id)
    {
      return "capture_" + std::to_string(capture_id) + ".oni";
    }
    
    /** \brief Given an oni filename figure out the index of the capture file.
     *  \param[in]  filename      name of the file
     *  \param[out] capture_id    id of the frame to be saved
     *  \return TRUE if input filename is a correctly formatted oni capture filename
     */
    inline
    bool getOniCaptureInfo(const std::string &filename, int &capture_id)
    {
      // Check if prefix is correct
      if (filename.substr(0, 8) != "capture_")
        return false;

      // Get image type
      int endPos   = filename.find(".oni");
      capture_id = std::stoi(filename.substr(8, endPos-8));
      
      return true;
    }
    
    //--------------------------------------------------------------------------
    // Depth to cloud conversion
    //--------------------------------------------------------------------------
    
    /** \brief Convert a vector of OpenCV Point3f to a PCL pointcloud.
     *  \param[in]  cv_cloud opencv cloud
     *  \param[out] pcl_cloud PCL cloud
     */
    template <typename PointT>
    void cvCloud2pclCloud (const std::vector<cv::Point3f> &cv_cloud, pcl::PointCloud<PointT> &pcl_cloud)
    {
      pcl_cloud.resize(cv_cloud.size());
      for (size_t pointId = 0; pointId < cv_cloud.size(); pointId++)
        pcl_cloud.points[pointId] = pcl::PointXYZ(cv_cloud[pointId].x, cv_cloud[pointId].y, cv_cloud[pointId].z);
    }
    
    /** \brief Convert PCL cloud to a vector of OpenCV Point3f.
     *  \param[in]  pcl_cloud PCL cloud
     *  \param[out] cv_cloud OpenCV cloud
     */    
    template <typename PointT>
    void pclCloud2cvCloud (const pcl::PointCloud<PointT> &pcl_cloud, std::vector<cv::Point3f> &cv_cloud)
    {
      cv_cloud.resize(pcl_cloud.size());
      for (size_t pointId = 0; pointId < pcl_cloud.size(); pointId++)
        cv_cloud[pointId] = cv::Point3f(pcl_cloud.points[pointId].x, pcl_cloud.points[pointId].y, pcl_cloud.points[pointId].z);      
    }
    
    /** \brief Convert a depth image and corresponding ideal point coordinates
     * of a PCL pointcloud. Invalid depth values are converted to NaN.
     *  \param[in]  depth OpenCV depth image (CV_16U where depth is expressed in milimetres)
     *  \param[in]  ideal_points a 1xN CV_32FC2 matrix where of ideal coordinates of depth image pixels. Pixels are ordered by scanning the rows.
     *  \param[out] cloud PCL pointcloud
     *  \param[in]  min_z minimum valid depth value
     *  \param[in]  max_z maximum valid depth value
     */
    template <typename PointT>
    void cvDepth2pclCloud_ideal ( const cv::Mat& depth,
                                  const cv::Mat& ideal_pixel_points,
                                  pcl::PointCloud<PointT>& cloud,
                                  const float min_z = 0.2f,
                                  const float max_z = 10.0f
                                )
    {
      if (depth.type() != CV_16UC1)
      {
        std::cout << "[utl::kinect::cvDepth2pclCloud] depth image must be of type CV_16UC1." << std::endl;
        abort();
      }

      if (ideal_pixel_points.type() != CV_32FC2)
      {
        std::cout << "[utl::kinect::cvDepth2pclCloud] ideal point matrix must be of type CV_32FC2." << std::endl;
        abort();
      }

      if (ideal_pixel_points.size().area() != depth.size().area())
      {
        std::cout << "[utl::kinect::cvDepth2pclCloud] number of points in depth image and ideal point matrix is different." << std::endl;
        abort();
      }
      
      // Prepare cloud      
      cloud.resize(depth.size().area());
      cloud.width = depth.cols;
      cloud.height = depth.rows;
      bool isDense = true;
      
      int pointId = 0;
      for (size_t x = 0; x < static_cast<size_t>(depth.cols); x++)
      {
        for (size_t y = 0; y < static_cast<size_t>(depth.rows); y++)
        {
          float z = static_cast<float>(depth.at<unsigned short>(y,x)) / 1000;   // Convert milimetres to metres here
          if (z < min_z || z > max_z)
          {
            cloud.at(x,y).x = std::numeric_limits<float>::quiet_NaN();
            cloud.at(x,y).y = std::numeric_limits<float>::quiet_NaN();
            cloud.at(x,y).z = std::numeric_limits<float>::quiet_NaN();          
            isDense = false;
          }
          else
          {
            cloud.at(x,y).x = ideal_pixel_points.at<cv::Vec2f>(pointId)[0] * z;
            cloud.at(x,y).y = ideal_pixel_points.at<cv::Vec2f>(pointId)[1] * z;
            cloud.at(x,y).z = z;
          }
          pointId++;
        }
      }
      cloud.is_dense = isDense;
    }
    
    /** \brief Convert an OpenCV depth image to a pointcloud. Invalid depth
     * values are converted to NaN.
     *  \param[in] depth OpenCV depth image (CV_16U where depth is expressed in milimetres)
     *  \param[in] K    depth camera calibration matrix
     *  \param[out] cloud PCL pointcloud
     *  \param[in]  min_z minimum valid depth value
     *  \param[in]  max_z maximum valid depth value
     */
    template <typename PointT>
    void cvDepth2pclCloud_undistorted ( const cv::Mat& depth,
                                        const cv::Mat& _K,
                                        pcl::PointCloud<PointT>& cloud,
                                        const float min_z = 0.2f,
                                        const float max_z = 10.0f
                                      )
    {
      // Check input
      if (depth.type() != CV_16UC1)
      {
        std::cout << "[utl::kinect::cvDepth2pclCloud] depth image must be of type CV_16UC1." << std::endl;
        abort();
      }
      
      // Convert camera matrix to double
      cv::Mat K;
      if (_K.depth() != CV_64F)
        _K.convertTo(K, CV_64F);
      else
        K = _K.clone();
      
      // Prepare cloud
      const int width   = depth.size().width;
      const int height  = depth.size().height;
      
      cloud.resize(width * height);
      cloud.width = width;
      cloud.height = height;
        
      const double inv_fx  = 1.0 / K.at<double>(0, 0);
      const double inv_fy  = 1.0 / K.at<double>(1, 1);
      const double ox      = K.at<double>(0, 2);
      const double oy      = K.at<double>(1, 2);
      
      bool isDense = true;
      
      for (size_t x = 0; x < width; x++)
      {
        for (size_t y = 0; y < height; y++)
        {
          float z = static_cast<float>(depth.at<unsigned short>(y,x)) / 1000;   // Convert milimetres to metres here
          
          if (z < 0.2 || z > 5.0)
          {
            cloud.at(x,y).x = std::numeric_limits<float>::quiet_NaN();
            cloud.at(x,y).y = std::numeric_limits<float>::quiet_NaN();
            cloud.at(x,y).z = std::numeric_limits<float>::quiet_NaN();          
            isDense = false;
          }
          else
          {
            cloud.at(x,y).x = (x-ox)*z*inv_fx;
            cloud.at(x,y).y = (y-oy)*z*inv_fy;
            cloud.at(x,y).z = z;
          }
        } 
      }   
      
      cloud.is_dense = isDense;
    }

    /** \brief Smooth a depth image using a moving average filter.
     *  \param[in]  depth_images a vector containing depth images
     *  \return  a smoothed depth image
     */
    inline
    cv::Mat depthImageTemporalSmoothing ( const std::deque<cv::Mat> &depth_images)
    {
      if (depth_images.size() < 1)
      {
        std::cout << "[utl::kinect::smoothDepthImage] input depth images are empty." << std::endl;
        std::abort();
      }
      
      // Prepare variables
      cv::Mat depthSmooth = cv::Mat::zeros(depth_images[0].size(), CV_32F);
      cv::Mat pixelWeights = cv::Mat::zeros(depth_images[0].size(), CV_32F);
      
      // Update cumulative image
      for (size_t imId = 0; imId < depth_images.size(); imId++)
      {
        // Convert to float
        cv::Mat imDepth;
        depth_images[imId].convertTo(imDepth, CV_32F);
        
        // Get valid pixel mask
        cv::Mat validDepth;
        cv::threshold(imDepth, validDepth, 0, 1.0, CV_THRESH_BINARY);
        
        // Update weights
        cv::add(pixelWeights, validDepth, pixelWeights);

        // Add valid pixels only
        validDepth.convertTo(validDepth, CV_8U);
        cv::add(depthSmooth, imDepth, depthSmooth, validDepth);
        
      }
      
      // Normalize
      float minNumValidPixels = static_cast<float>(depth_images.size()) / 2.0f;
      for (size_t x = 0; x < static_cast<size_t>(depthSmooth.cols); x++)
        for (size_t y = 0; y < static_cast<size_t>(depthSmooth.rows); y++)
        {
          float curPixelWeight = pixelWeights.at<float>(y,x);
          if (curPixelWeight < minNumValidPixels)
            depthSmooth.at<float>(y,x) = 0.0f;
          else
            depthSmooth.at<float>(y,x) = depthSmooth.at<float>(y,x) / curPixelWeight;
        }
        
      // Convert back to CV_16U
      cv::Mat depthSmooth_16U;
      depthSmooth.convertTo(depthSmooth_16U, CV_16U);
      
      return depthSmooth_16U;
    }
    
    //--------------------------------------------------------------------------
    // Depth image radial indistortion
    //--------------------------------------------------------------------------
    
    /** \brief Find discontinuities in a depth image by removing all pixels for 
     * which the maximum difference between the center pixel and it's neighbours
     * is higher than a threshold
     *  \param[in] depth input depth image
     *  \param[in] depth difference threshold
     *  \return cleaned depth image
     */
    inline
    void getDepthDiscontinuities  ( const cv::Mat &depth, cv::Mat &depth_disc, const float depth_diff_thresh = 50.0f)
    {
      cv::Mat dDepth;
      cv::Mat kernel = - cv::Mat::ones(3, 3, CV_32F);
      kernel.at<float>(1,1) = 8;
      
      cv::filter2D(depth, dDepth, CV_32F, kernel, cv::Point(1,1), 0.0, cv::BORDER_CONSTANT);
      dDepth = cv::abs(dDepth);
      cv::threshold(dDepth, dDepth, depth_diff_thresh, 1.0, CV_THRESH_BINARY);
      
      dDepth.convertTo(depth_disc, CV_8U, 255);
    }    

    /** \brief Rectify radial distortion in a depth image using a precomputed 
     * rectification map (see cv::InitUndistortRectifyMap()). To avoid depth 
     * interpolation across object boundaries only valid pixels in the original
     * depth image are used to generate the undistorted image. A pixel is
     * considered valid if it is non-zero and it is not a depth discontinuity.
     *  \param[in]  depth   input depth image
     *  \param[in]  map_x   ideal x coordinates of the undistorted image
     *  \param[in]  map_y   ideal y coordinates of the undistorted image
     *  \param[in]  depth_diff_thresh threshold for detecting depth discontinuities
     *  \return undistorted depth image
     */    
    inline
    cv::Mat undistortDepthImage ( const cv::Mat &depth, const cv::Mat &map_x, const cv::Mat &map_y, const float depth_diff_thresh = 50.0f )
    {
      // Get non-zero pixels
      cv::Mat validPixels;
      cv::Mat tmp;
      depth.convertTo(tmp, CV_8U, 1 /255.0f);
      cv::threshold(tmp, validPixels, 0, 255, CV_THRESH_BINARY);
      
      // Get discontinuity pixels
      cv::Mat depthDisc;
      utl::kinect::getDepthDiscontinuities(depth, depthDisc, depth_diff_thresh);
      
      // Combine
      cv::bitwise_and(validPixels, 255 - depthDisc, validPixels);

      // Undistort image      
      return utl::ocvcalib::undistort(depth, map_x, map_y, validPixels);
    }
    
    //--------------------------------------------------------------------------
    // Depth distortion correction
    //--------------------------------------------------------------------------
    
    /** \brief Write depth distortion parameters.
     *  \param[in]  dirname directory where depth distortion maps are saved
     *  \param[in]  dm_maps depth distortion maps
     *  \param[in]  dm_distances depth distortion map distances
     */
    inline
    bool writeDepthDistortionParameters ( const std::string &dm_filename, const std::vector<cv::Mat> &dm_maps, const std::vector<float> &dm_distances)
    {
      // Check distortion map validity
      if (dm_maps.size() == 0)
      {
        std::cout << "[utl::kinect::writeDepthDistortionParameters] depth multiplier maps are empty. Nothing to save." << std::endl;
        return false;
      }
      
      if (dm_maps.size() != dm_distances.size())
      {
        std::cout << "[utl::kinect::writeDepthDistortionParameters] number of depth multiplier maps and distances must be the same." << std::endl;
        return false;
      }
      
      cv::FileStorage fs(dm_filename, cv::FileStorage::WRITE);
      if( fs.isOpened() )
      {
        fs << "num_dm_maps" << static_cast<int>(dm_distances.size());
        for (size_t dmId = 0; dmId < dm_distances.size(); dmId++)
        {
          fs << "dm_" + std::to_string(dmId) + "_distance"    << dm_distances[dmId];
          fs << "dm_" + std::to_string(dmId) + "_multipliers" << dm_maps[dmId];
        }
      }
      else
      {
        std::cout << "[utl::kinect::writeDepthDistortionParameters] Could not open file to save parameters." << std::endl;
        std::cout << "[utl::kinect::writeDepthDistortionParameters] Calibration file: '" << dm_filename << "'." << std::endl;
        return false;
      }
      
      fs.release();
      return true;      
    }
    
    /** \brief Write depth distortion parameters.
     *  \param[in]  dirname directory where depth distortion maps are saved
     *  \param[in]  dm_maps depth distortion maps
     *  \param[in]  dm_distances depth distortion map distances
     */
    inline
    bool readDepthDistortionParameters ( const std::string &dm_filename, std::vector<cv::Mat> &dm_maps, std::vector<float> &dm_distances)
    {
      cv::FileStorage fs(dm_filename, cv::FileStorage::READ);
      if( fs.isOpened() )
      {
        int num_dm_maps;
        fs["num_dm_maps"]    >> num_dm_maps;
        
        dm_distances.resize(num_dm_maps);
        dm_maps.resize(num_dm_maps);
        
        for (size_t dmId = 0; dmId < static_cast<size_t>(num_dm_maps); dmId++)
        {
          fs["dm_" + std::to_string(dmId) + "_distance"]    >> dm_distances[dmId];
          fs["dm_" + std::to_string(dmId) + "_multipliers"] >> dm_maps[dmId];
        }
      }
      else
      {
        std::cout << "[utl::kinect::readDepthDistortionParameters] Could not open file to load parameters." << std::endl;
        std::cout << "[utl::kinect::readDepthDistortionParameters] Calibration file: '" << dm_filename << "'." << std::endl;
        return false;
      }
      
      fs.release();
      return true;
    }    
    
    /** \brief Correct depth distortion in a kinect depth image.
     *  \param[in]  depth OpenCV depth image (CV_16U where depth is expressed in milimetres)
     *  \param[out] depth_undistorted a 1xN CV_32FC2 matrix where of ideal coordinates of depth image pixels. Pixels are ordered by scanning the rows.
     *  \param[out] cloud PCL pointcloud
     */
    inline
    cv::Mat correctDepthDistortion ( const cv::Mat& depth, cv::Mat& depth_corrected, const std::vector<cv::Mat> &dm_maps, const std::vector<float> &dm_distances)
    {
      cv::Mat dm_used = cv::Mat::ones(depth.size(), CV_32F);
      
      // Check input
      if (depth.type() != CV_16U)
      {
        std::cout << "[utl::kinect::correctDepthDistortion] depth image must be of type CV_16U." << std::endl;
        std::abort();
      }
      
      if (dm_maps.size() == 0)
      {
        std::cout << "[utl::kinect::correctDepthDistortion] depth multiplier maps are empty returning depth unmodified." << std::endl;
        depth_corrected = depth.clone();
        return dm_used;
      }
      
      if (dm_maps.size() != dm_distances.size())
      {
        std::cout << "[utl::kinect::correctDepthDistortion] number of depth multiplier maps and distances must be the same." << std::endl;
        std::abort();
      }
      
      if (dm_maps[0].size() != depth.size())
      {
        std::cout << "[utl::kinect::correctDepthDistortion] size of depth map and depth multiplier maps must be the same." << std::endl;
        std::abort();
      }
      
      // Correct depth
      depth_corrected = cv::Mat(depth.size(), depth.type());
      
      for (size_t x = 0; x < static_cast<size_t>(depth.cols); x++)
      {
        for (size_t y = 0; y < static_cast<size_t>(depth.rows); y++)
        {
          float d_estimated, d_modifier;
          d_estimated = static_cast<float>(depth.at<unsigned short>(y,x)) / 1000.0f;
          d_modifier = 1.0f;
          
          if (d_estimated != 0.0)
          {
            // Update map
            std::pair<int,int> mapIndices = utl::stdvec::nearestValues(dm_distances, d_estimated);
            
            if (mapIndices.first != -1 && mapIndices.second != -1)
            {
              if (mapIndices.first == mapIndices.second)
              {
                d_modifier = dm_maps[mapIndices.first].at<float>(y,x);
              }
              else
              {
                float weight = (d_estimated - dm_distances[mapIndices.first]) / (dm_distances[mapIndices.second] - dm_distances[mapIndices.first]);
                d_modifier = dm_maps[mapIndices.first].at<float>(y,x) * (1-weight) + dm_maps[mapIndices.second].at<float>(y,x) * weight;
              }
            }
            else if (mapIndices.first != -1 && mapIndices.second == -1)
            {
              d_modifier = dm_maps[mapIndices.first].at<float>(y,x);
            }
            else if (mapIndices.first == -1 && mapIndices.second != -1)
            {
              d_modifier = dm_maps[mapIndices.second].at<float>(y,x);
            }
          }
          
          depth_corrected.at<unsigned short>(y,x) = static_cast<unsigned short>(d_estimated * d_modifier * 1000.0f);
          dm_used.at<float>(y,x) = d_modifier;
        }
      }
      
      return dm_used;
    }
  }
}

#endif    // OPENNI2_TOOLS_HPP
