#ifndef WORKSPACE_DETECTION_UTILITIES_HPP
#define WORKSPACE_DETECTION_UTILITIES_HPP

#include <pcl/point_cloud.h>
#include <namaris/utilities/pcl_typedefs.hpp>

namespace g1 {
	namespace vision {

		struct Box {
			Eigen::Affine3f pose_;
			Eigen::Vector3f size_;
			std::string id_;
		};

		bool findTableBox	(	const pcl::PointCloud<PointNC>::ConstPtr &cloud,
								float table_height,
								const std::string &table_id,
								g1::vision::Box &table_box
						  	);

		bool findFridgeBox	(	const pcl::PointCloud<PointNC>::ConstPtr &cloud,
								bool interior,
								const std::string &fridge_box_id,
								g1::vision::Box &fridge_box
							);

	}
}

#endif /* WORKSPACE_DETECTION_UTILITIES_HPP */
