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
static const char PUCKS_WINDOW[] = "cb_vision_pucks";

class CBPuckFinder
{
	ros::NodeHandle nh_;

	/**
	 * Video frames.
	 */
	Mat blurred_frame;
	Mat hsv_frame;   // Converted to HSV space from raw_frame.
	Mat bw_frame;    // Converted to black/white from hsv_frame.
	Mat canny_frame;   // Converted to edges from bw_frame.

	/**
	 * Stuff to keep track of.
	 */
	vector<vector<Point> > contours;   // Keep track of all contours found in a frame.
	vector<Vec4i> contours_hierarchy;   // Used to store hierarchy of contours found.
	vector<Point> maybe_puck;   // Used to temporarily store a contour when determining whether or not it's a puck.
	vector<vector<Point> > pucks;   // Pucks identified.
	//vector<Point2f> center;   // Centers of minimum enclosing circles around pucks.
	//vector<float> radius;   // Radii of enclosing circles.

	/**
	 * Thresholds.
	 */
	uint8_t puck_hue_low;
	uint8_t puck_hue_high;
	uint8_t puck_sat_low;
	uint8_t puck_sat_high;
	uint8_t puck_val_low;
	uint8_t puck_val_high;
	uint16_t puck_min_size;
	uint16_t puck_max_size;
	uint8_t blur_size;
	uint8_t canny_lower_threshold;

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

