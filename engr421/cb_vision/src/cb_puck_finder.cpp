#include <cb_vision/cb_puck_finder.h>

CBPuckFinder::CBPuckFinder(ros::NodeHandle nh) : it(nh)
{
	nh_ = nh;

	cb_vision_sub = it.subscribe("cb_vision_image_in", 1, &CBPuckFinder::image_cb, this);
	cb_vision_params_sub = nh_.subscribe("cb_vision_params_in", 1, &CBPuckFinder::params_cb, this);
	cb_vision_pub = nh_.advertise<cb_vision::cb_puck_coordinates>("cb_puck_coordinates", 1);

	pc.x[0] = 0;
	pc.y[0] = 0;
	pc.x[1] = 0;
	pc.y[1] = 0;

	// Initialize HSV parameters rather arbitrarily. These will be updated from
	// the ROS parameter server later.
	puck_hue_low  = 100;
	puck_hue_high = 160;
	puck_sat_low  = 10;
	puck_sat_high = 80;
	puck_val_low  = 60;
	puck_val_high = 255;
	encircle_min_size = 0;
	encircle_max_size = 1000;
	erosion_iter = 0;
	puckiness_min_ratio = 0.0;
	canny_lower_threshold = 0;

	// Set up windows.
	cvNamedWindow(RAW_WINDOW, 1);
	cvMoveWindow(RAW_WINDOW, 50, 50);

	cvNamedWindow(BW_WINDOW, 1);
	cvMoveWindow(BW_WINDOW, 380, 50);

	cvNamedWindow(PUCKS_WINDOW, 1);
	cvMoveWindow(PUCKS_WINDOW, 710, 50);
}

void CBPuckFinder::image_cb(const sensor_msgs::ImageConstPtr& msg)
{
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// Clear old stuff.
	contours.clear();
	closed_contours.clear();
	pucks.clear();

	// Rectify image.
	perspectiveTransform(cv_ptr->image, rectified_frame, getPerspectiveTransform(Point(0,40), Point(0,200)));
	// TODO: Get board and do polygon recognition on board. Erosion+dilation
	// steps should get us a clean trapezoid, from which we can extract corners
	// and use for perspective transform. Some good HSV thresholds for the
	// board:
	//     HL: 0   HH: 40
	//     SL: 0   SH: 255
	//     VL: 80  VH: 200

	// Convert frame to HSV space and save to hsv_frame.
	cvtColor(rectified_frame, hsv_frame, CV_BGR2HSV);

	// Threshold hsv_frame for color of pucks and save to bw_frame.
	inRange(hsv_frame, Scalar(puck_hue_low, puck_sat_low, puck_val_low),
			Scalar(puck_hue_high, puck_sat_high, puck_val_high), bw_frame);

	// Erode image to get sharper corners.
	erode(bw_frame, eroded_frame, Mat(), Point(-1, -1), erosion_iter);

	// Find edges with Canny.
	Canny(eroded_frame, canny_frame, canny_lower_threshold, canny_lower_threshold*2, 5);

	// Find contours.
	findContours(canny_frame, contours, contours_hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	/**
	 * Test each contour for puckiness.
	 */

	// Find closed contours.
	for (uint16_t i=0; i<contours.size(); i++) {
		// Approximate contour with accuracy proportional to contour perimeter.
		approxPolyDP(Mat(contours[i]), maybe_puck, arcLength(Mat(contours[i]), true)*0.02, true);
		closed_contours.push_back(maybe_puck);
	}

	// Calculate centers of contours.
	vector<Point2f> center(closed_contours.size());
	vector<float> radius(closed_contours.size());
	for (uint16_t i=0; i<closed_contours.size(); i++) {
		minEnclosingCircle((Mat) closed_contours[i], center[i], radius[i]);
	}

	for (uint16_t i=0; i<closed_contours.size(); i++) {
		// Pucks should have a certain minimum area-to-enclosing-circle ratio.
		if (fabs(contourArea(Mat(closed_contours[i]))) / (3.14159*radius[i]*radius[i]) > puckiness_min_ratio) {
			pucks.push_back(closed_contours[i]);
		}
	}

	// Generate drawing of puck locations.
	Mat drawing = Mat::zeros(240, 320, CV_8UC3);
	for (uint16_t i=0; i<pucks.size(); i++) {
		Scalar color = Scalar(0, 255, 0);   // Green!
		if (radius[i] > encircle_min_size && radius[i] < encircle_max_size) {
			drawContours(drawing, pucks, i, color, 1, 8, vector<Vec4i>(), 0, Point());
			circle(drawing, center[i], (int) radius[i], color, 2, 8, 0);
		}
	}

	// Show images.
	imshow(RAW_WINDOW, cv_ptr->image);
	imshow(BW_WINDOW, eroded_frame);
	imshow(PUCKS_WINDOW, drawing);

	// Wait 2 ms for a keypress.
	int c = waitKey(2);
	// Exit if the spacebar is pressed. NOTE: killing the program with
	// Ctrl+C sometimes stops OpenCV at a bad place and effects a kernel
	// panic! If you really like Ctrl+C, do so at your own risk.
	if ((char) c == 32) {
		return;
	}

	pc.x[0] = pucks.size();
	pc.x[1] = erosion_iter;

	cb_vision_pub.publish(pc);
}

void CBPuckFinder::params_cb(const rqt_cb_gui::cb_params& msg)
{
	puck_hue_low  = msg.puck_hue_low;
	puck_hue_high = msg.puck_hue_high;
	puck_sat_low  = msg.puck_sat_low;
	puck_sat_high = msg.puck_sat_high;
	puck_val_low  = msg.puck_val_low;
	puck_val_high = msg.puck_val_high;
	encircle_min_size = msg.encircle_min_size;
	encircle_max_size = msg.encircle_max_size;
	erosion_iter  = msg.erosion_iter;
	puckiness_min_ratio = msg.puckiness_min_ratio;
	canny_lower_threshold = msg.canny_lower_threshold;
}

