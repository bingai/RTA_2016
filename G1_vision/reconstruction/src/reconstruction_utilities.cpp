#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>

#include <reconstruction/reconstruction_utilities.hpp>

void SLAMObjectToReconstructionMessage(const SLAM &slam, reconstruction_msgs::Reconstruction &rec_msg) {

	pcl::toROSMsg(*slam.model.pointCloud, rec_msg.model_point_cloud);

	rec_msg.model_keypoint_world_coordinates.resize(3*slam.model.keypointWorldCoordinates.cols());
	Eigen::MatrixXf::Map(rec_msg.model_keypoint_world_coordinates.data(), 3, slam.model.keypointWorldCoordinates.cols()) = slam.model.keypointWorldCoordinates;

	rec_msg.model_keypoint_descriptors = slam.model.keypointDescriptors;

	rec_msg.registered_views.resize(slam.model.registeredViews.size());
	for (int i = 0; i < slam.model.registeredViews.size(); ++i) {
		std_msgs::Header header;
		header.seq = i;
		header.stamp = ros::Time::now();
		cv_bridge::CvImage img_bridge;
		img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, slam.model.registeredViews[i].imageRGB);
		img_bridge.toImageMsg(rec_msg.registered_views[i].image);

		Eigen::MatrixXf K, d;
		cv::cv2eigen(slam.model.registeredViews[i].intrinsicsMatrix, K);
		cv::cv2eigen(slam.model.registeredViews[i].distortionCoefficients, d);
		rec_msg.registered_views[i].intrinsics_matrix.resize(K.size());
		rec_msg.registered_views[i].distortion_coefficients.resize(d.size());
		Eigen::MatrixXf::Map(rec_msg.registered_views[i].intrinsics_matrix.data(), 3, 3) = K;
		Eigen::MatrixXf::Map(rec_msg.registered_views[i].distortion_coefficients.data(), d.rows(), d.cols()) = d;

		// Eigen::Affine3d pose;
		// pose.matrix() = slam.model.registeredViews[i].pose.cast<double>();
		// tf::poseEigenToMsg(pose, rec_msg.registered_views[i].pose);
		rec_msg.registered_views[i].pose.resize(16);
		Eigen::MatrixXf::Map(rec_msg.registered_views[i].pose.data(), 4, 4) = slam.model.registeredViews[i].pose;

		rec_msg.registered_views[i].frustum.resize(16);
		Eigen::Matrix4f frustum_mat;
		for (int j = 0; j < 4; ++j) {
			frustum_mat.col(j) = slam.model.registeredViews[i].frustum[j];
		}
		Eigen::MatrixXf::Map(rec_msg.registered_views[i].frustum.data(), 4, 4) = frustum_mat;
	}
}

void reconstructionMessageToSLAMObject(const reconstruction_msgs::Reconstruction &rec_msg, SLAM &slam) {
	slam.reset();

	pcl::fromROSMsg(rec_msg.model_point_cloud, *slam.model.pointCloud);

	slam.model.keypointWorldCoordinates = Eigen::MatrixXf::Map(rec_msg.model_keypoint_world_coordinates.data(), 3, rec_msg.model_keypoint_world_coordinates.size()/3);

	slam.model.keypointDescriptors = rec_msg.model_keypoint_descriptors;

	slam.model.registeredViews.resize(rec_msg.registered_views.size());
	for (int i = 0; i < rec_msg.registered_views.size(); ++i) {
		slam.model.registeredViews[i].imageRGB = cv_bridge::toCvCopy(rec_msg.registered_views[i].image)->image;

		Eigen::MatrixXf K, d;
		K = Eigen::MatrixXf::Map(rec_msg.registered_views[i].intrinsics_matrix.data(), 3, 3);
		d = Eigen::MatrixXf::Map(rec_msg.registered_views[i].distortion_coefficients.data(), rec_msg.registered_views[i].distortion_coefficients.size(), 1);
		cv::eigen2cv(K, slam.model.registeredViews[i].intrinsicsMatrix);
		cv::eigen2cv(d, slam.model.registeredViews[i].distortionCoefficients);

		// Eigen::Affine3d pose;
		// tf::poseMsgToEigen(rec_msg.registered_views[i].pose, pose);
		// slam.model.registeredViews[i].pose = pose.matrix().cast<float>();
		slam.model.registeredViews[i].pose = Eigen::MatrixXf::Map(rec_msg.registered_views[i].pose.data(), 4, 4);

		Eigen::Matrix4f frustum_mat = Eigen::MatrixXf::Map(rec_msg.registered_views[i].frustum.data(), 4, 4);
		slam.model.registeredViews[i].frustum.resize(4);
		for (int j = 0; j < 4; ++j) {
			slam.model.registeredViews[i].frustum[j] = frustum_mat.col(j);
		}
	}
}
