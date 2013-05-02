#ifndef CB_PUCK_FINDER_H
#define CB_PUCK_FINDER_H

#include <ros/ros.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <cb_vision/cb_puck_coordinates.h>
#include <rqt_cb_gui/cb_params.h>


namespace enc = sensor_msgs::image_encodings;
using namespace cv;

static const char RAW_WINDOW[] = "cb_vision_raw_frames";
static const char BW_WINDOW[]  = "cb_vision_bw_frames";

class CBPuckFinder
{
	ros::NodeHandle nh_;

	/**
	 * Video frames.
	 */
	Mat hsv_frame;   // Converted to HSV space from raw_frame.
	Mat bw_frame;    // Converted to black/white from hsv_frame.

	/**
	 * Thresholds.
	 */
	uint8_t puck_hue_low;
	uint8_t puck_hue_high;
	uint8_t puck_sat_low;
	uint8_t puck_sat_high;
	uint8_t puck_val_low;
	uint8_t puck_val_high;

	cv_bridge::CvImagePtr cv_ptr;
	cb_vision::cb_puck_coordinates pc;

	image_transport::ImageTransport it;
	image_transport::Subscriber cb_vision_sub;
	ros::Subscriber cb_vision_params_sub;
	ros::Publisher cb_vision_pub;

	/**
	 * Callback for input video.
	 */
	void image_cb(const sensor_msgs::ImageConstPtr& msg);

	/**
	 * Callback for parameters set by GUI.
	 */
	void params_cb(const rqt_cb_gui::cb_params& msg);

public:
	CBPuckFinder(ros::NodeHandle nh);
};

#endif // CB_PUCK_FINDER_H

