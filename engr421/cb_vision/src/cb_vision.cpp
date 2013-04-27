/**
 * @file cb_vision.cpp
 */

#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cb_vision/cb_image_converter.h>
#include <cb_vision/cb_puck_coordinates.h>


using namespace cv;


ros::Publisher cb_vision_pub;

static const char RAW_WINDOW[] = "cb_vision_raw_frames";
static const char BW_WINDOW[]  = "cb_vision_bw_frames";

/**
 * Video frames.
 */
Mat raw_frame;   // Raw video frame.
Mat hsv_frame;   // Converted to HSV space from raw_frame.
Mat bw_frame;    // Converted to black/white from hsv_frame.

/**
 * Thresholds.
 */
int puck_hue_low  = 100;
int puck_hue_high = 160;
int puck_sat_low  = 10;
int puck_sat_high = 80;
int puck_val_low  = 60;
int puck_val_high = 255;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cb_vision");
	ros::NodeHandle nh;

	CBImageConverter ic(nh);

	cb_vision_pub = nh.advertise<cb_vision::cb_puck_coordinates>("cb_puck_coordinates", 1);
	cb_vision::cb_puck_coordinates pc;
	pc.x = 0;
	pc.y = 0;

	// Set up windows.
	cvNamedWindow(RAW_WINDOW, 1);
	cvMoveWindow(RAW_WINDOW, 20, 20);

	cvNamedWindow(BW_WINDOW, 1);
	cvMoveWindow(BW_WINDOW, 20, 270);

	while (true) {
		// Convert frame to HSV space and save to hsv_frame.
		cvtColor(raw_frame, hsv_frame, CV_BGR2HSV);

		// Threshold hsv_frame for color of pucks and save to bw_frame.
		inRange(hsv_frame, Scalar(puck_hue_low, puck_sat_low, puck_val_low),
				Scalar(puck_hue_high, puck_sat_high, puck_val_high), bw_frame);

		// Show images.
		imshow(RAW_WINDOW, raw_frame);
		imshow(BW_WINDOW, bw_frame);

		// Wait 2 ms for a keypress.
		int c = waitKey(2);
		// Exit if the spacebar is pressed. NOTE: killing the program with
		// Ctrl+C sometimes stops OpenCV at a bad place and effects a kernel
		// panic! If you really like Ctrl+C, do so at your own risk.
		if ((char) c == 32) {
			return 0;
		}

		cb_vision_pub.publish(pc);
	}

	return 0;
}

