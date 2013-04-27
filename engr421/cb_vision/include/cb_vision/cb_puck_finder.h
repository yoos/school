#ifndef CB_IMAGE_CONVERTER_H
#define CB_IMAGE_CONVERTER_H

#include <ros/ros.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <cb_vision/cb_puck_coordinates.h>


namespace enc = sensor_msgs::image_encodings;
using namespace cv;

static const char RAW_WINDOW[] = "cb_vision_raw_frames";
static const char BW_WINDOW[]  = "cb_vision_bw_frames";

class CBPuckFinder
{
	/**
	 * Video frames.
	 */
	Mat hsv_frame;   // Converted to HSV space from raw_frame.
	Mat bw_frame;    // Converted to black/white from hsv_frame.

	/**
	 * Thresholds.
	 */
	int puck_hue_low;
	int puck_hue_high;
	int puck_sat_low;
	int puck_sat_high;
	int puck_val_low;
	int puck_val_high;

	cv_bridge::CvImagePtr cv_ptr;
	cb_vision::cb_puck_coordinates pc;

	image_transport::ImageTransport it;
	image_transport::Subscriber cb_vision_sub;
	ros::Publisher cb_vision_pub;

	void image_cb(const sensor_msgs::ImageConstPtr& msg);

public:
	CBPuckFinder(ros::NodeHandle nh);
};

#endif // CB_IMAGE_CONVERTER_H

