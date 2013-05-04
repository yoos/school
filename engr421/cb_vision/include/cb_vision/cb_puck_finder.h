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
	 * Stuff to keep track of.
	 */
	vector<vector<Point> > pucks_contours;   // Keep track of all contours found in a frame of pucks.
	vector<Vec4i> pucks_contours_hierarchy;   // Used to store hierarchy of contours found in frame of pucks.
	vector<Point> maybe_puck;   // Used to temporarily store a contour when determining whether or not it's a puck.
	vector<vector<Point> > pucks_closed_contours;
	vector<vector<Point> > target_pucks;   // Pucks to shoot at.
	vector<Point2f> pucks_encircle_centers;   // Centers of minimum enclosing circles around pucks.
	vector<float> pucks_encircle_radii;   // Radii of enclosing circles.

	/**
	 * Thresholds.
	 */
	// Board
	uint8_t board_hue_low;
	uint8_t board_hue_high;
	uint8_t board_sat_low;
	uint8_t board_sat_high;
	uint8_t board_val_low;
	uint8_t board_val_high;
	uint8_t board_erosion_iter;   // Erode this many times.
	uint8_t board_dilation_iter;   // Dilate this many times.
	uint8_t board_canny_lower_threshold;

	// Puck
	uint8_t puck_hue_low;
	uint8_t puck_hue_high;
	uint8_t puck_sat_low;
	uint8_t puck_sat_high;
	uint8_t puck_val_low;
	uint8_t puck_val_high;
	uint16_t encircle_min_size;   // Maximum size of the enclosing circle around contours.
	uint16_t encircle_max_size;   // Minimum size of the enclosing circle around contours.
	uint8_t puck_erosion_iter;   // Erode this many times.
	float puckiness_min_ratio;   // Minimum ratio of contour-to-enclosing-circle. This helps us filter out noise.
	uint8_t puck_canny_lower_threshold;

	cv_bridge::CvImagePtr cv_ptr;
	cb_vision::cb_puck_coordinates pc;

	image_transport::ImageTransport it;
	image_transport::Subscriber cb_vision_sub;
	ros::Subscriber cb_vision_params_sub;
	ros::Publisher cb_vision_pub;

	/**
	 * Rectify the board. Takes orig_image as input and outputs to rect_image.
	 */
	void rectify_board(Mat image, Mat rect_image);

	/**
	 * Find pucks in input image and output vector of puck coordinates.
	 */
	void find_pucks(Mat image, vector<vector<Point> > pucks);

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

