#include <cb_vision/cb_image_converter.h>

namespace enc = sensor_msgs::image_encodings;

CBImageConverter::CBImageConverter(ros::NodeHandle nh) : it_(nh)
{
	image_pub_ = it_.advertise("/cb_image_out", 1);
	image_sub_ = it_.subscribe("/gscam/image_raw", 1, &CBImageConverter::image_cb, this);
}

void CBImageConverter::image_cb(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;

	try {
		cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	image_pub_.publish(cv_ptr->toImageMsg());
}

