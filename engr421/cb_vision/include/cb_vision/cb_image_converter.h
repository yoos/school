#ifndef CB_IMAGE_CONVERTER_H
#define CB_IMAGE_CONVERTER_H

#include <ros/ros.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

class CBImageConverter
{
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

public:
	CBImageConverter(ros::NodeHandle nh);
	void image_cb(const sensor_msgs::ImageConstPtr& msg);
};

#endif // CB_IMAGE_CONVERTER_H

