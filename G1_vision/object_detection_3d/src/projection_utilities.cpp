#include <object_detection_3d/object_detection_3d.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

Eigen::MatrixXf g1::vision::projectPointCloudToImageView  ( const pcl::PointCloud<PointNC>::ConstPtr &cloud_in,
						                                                const cv::Mat &intrinsics,
						                                                const cv::Mat &distortion,
						                                                const Eigen::Matrix4f &tform
						                                              )
{
	Eigen::MatrixXf pts_2d;
	pcl::PointCloud<PointNC>::Ptr cloud(new pcl::PointCloud<PointNC>);
	*cloud = *cloud_in;
	std::vector<int> tmp;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, tmp);

	if (cloud->empty())
	{
		pts_2d.resize(2,0);
		return pts_2d;
	}

	cv::Mat rvec, tvec, pts_3d_cv;

	Eigen::Matrix3f R = tform.block<3,3>(0,0);
	Eigen::Vector3f t = tform.block<3,1>(0,3);
	cv::eigen2cv(R, rvec);
	cv::eigen2cv(t, tvec);
	cv::Rodrigues(rvec, rvec);

	Eigen::MatrixXf pts_3d = cloud->getMatrixXfMap().topRows(3).transpose();
	cv::eigen2cv(pts_3d, pts_3d_cv);

	std::vector<cv::Point2f> pts_2d_cv;
	cv::projectPoints(pts_3d_cv, rvec, tvec, intrinsics, distortion, pts_2d_cv);

	pts_2d.resize(2, pts_2d_cv.size());
	for (int i = 0; i < pts_2d_cv.size(); ++i)
	{
		pts_2d(0,i) = pts_2d_cv[i].x;
		pts_2d(1,i) = pts_2d_cv[i].y;
	}
	
	return pts_2d;
}

void g1::vision::boundingBox2DCorners(Eigen::MatrixXf pts, int &xmin, int &ymin, int &xmax, int &ymax)
{
	xmin = std::round(pts.row(0).minCoeff());
	ymin = std::round(pts.row(1).minCoeff());
	xmax = std::round(pts.row(0).maxCoeff());
	ymax = std::round(pts.row(1).maxCoeff());
}
