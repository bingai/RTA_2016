#ifndef RECONSTRUCTION_UTILITIES_HPP
#define RECONSTRUCTION_UTILITIES_HPP

#include <sisyphus/sift_rgbd_slam.hpp>
#include <reconstruction_msgs/Reconstruction.h>

void SLAMObjectToReconstructionMessage(const SLAM &slam, reconstruction_msgs::Reconstruction &rec_msg);

void reconstructionMessageToSLAMObject(const reconstruction_msgs::Reconstruction &rec_msg, SLAM &slam);

#endif /* RECONSTRUCTION_UTILITIES_HPP */
