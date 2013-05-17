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
	board_hue_low  = 100;
	board_hue_high = 160;
	board_sat_low  = 10;
	board_sat_high = 80;
	board_val_low  = 60;
	board_val_high = 255;
	board_erosion_iter = 0;
	board_dilation_iter = 0;
	board_canny_lower_threshold = 0;

	puck_hue_low  = 100;
	puck_hue_high = 160;
	puck_sat_low  = 10;
	puck_sat_high = 80;
	puck_val_low  = 60;
	puck_val_high = 255;
	encircle_min_size = 0;
	encircle_max_size = 1000;
	puck_erosion_iter = 0;
	puckiness_min_ratio = 0.0;
	puck_canny_lower_threshold = 0;

	// Board corner locations. Should be grabbed later from ROS parameter
	// server.
	board_corner_0_x = 0;
	board_corner_0_y = 0;
	board_corner_1_x = 0;
	board_corner_1_y = 0;
	board_corner_2_x = 0;
	board_corner_2_y = 0;
	board_corner_3_x = 0;
	board_corner_3_y = 0;

	// Set up windows.
	cvNamedWindow(RAW_WINDOW, 1);
	cvMoveWindow(RAW_WINDOW, 50, 50);

	cvNamedWindow(BW_WINDOW, 1);
	cvMoveWindow(BW_WINDOW, 380, 50);

	cvNamedWindow(PUCKS_WINDOW, 1);
	cvMoveWindow(PUCKS_WINDOW, 710, 50);
}

void CBPuckFinder::rectify_board(Mat* image, Mat* rect_image)
{
	// Wait for the user to select board corners.
	while (!nh_.hasParam("corner0/x")) {
		ros::Duration(0.2).sleep();
	}
	get_parameters();   // Is it bad to call this every loop?

	// Rectify image.
	//perspectiveTransform(orig_image, rect_frame, getPerspectiveTransform(Point(0,40), Point(0,200)));
	// Board size is 22.3125 x 45 inches.
	int dpi = 20;
	Point2f src = {Point2f(board_corner_0_x, board_corner_0_y),
				   Point2f(board_corner_1_x, board_corner_1_y),
				   Point2f(board_corner_2_x, board_corner_2_y),
				   Point2f(board_corner_3_x, board_corner_3_y)};
	Point2f dst = {Point2f(0, 0),
				   Point2f(0, 45*dpi),
				   Point2f(22.3125*dpi, 0),
				   Ponit2f(22.3125*dpi, 45*dpi)};

	Mat trans = getPerspectiveTransform(src, dst);

	perspectiveTransform(image, rect_image, trans);
}

void CBPuckFinder::find_pucks(Mat* image, vector<vector<Point> >* pucks)
{
	// Clear old vectors.
	pucks_contours.clear();
	pucks_closed_contours.clear();
	target_pucks.clear();
	pucks_encircle_centers.clear();
	pucks_encircle_radii.clear();

	// Convert image to HSV space and save to hsv_image.
	static Mat hsv_image;
	cvtColor(*image, hsv_image, CV_BGR2HSV);

	// Threshold image for color of pucks and save to bw_image.
	static Mat bw_image;
	inRange(*hsv_image, Scalar(puck_hue_low, puck_sat_low, puck_val_low),
			Scalar(puck_hue_high, puck_sat_high, puck_val_high), bw_image);

	// Erode image to get sharper corners.
	static Mat eroded_image;
	erode(bw_image, eroded_image, Mat(), Point(-1, -1), puck_erosion_iter);

	// Find edges with Canny.
	static Mat canny_image;
	Canny(eroded_image, canny_image, puck_canny_lower_threshold, puck_canny_lower_threshold*2, 5);

	// Find contours.
	findContours(canny_image, pucks_contours, pucks_contours_hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	/**
	 * Test each contour for puckiness.
	 */

	// Check that the contour is closed.
	for (uint16_t i=0; i<pucks_contours.size(); i++) {
		// Approximate contour with accuracy proportional to contour perimeter.
		approxPolyDP(Mat(pucks_contours[i]), maybe_puck, arcLength(Mat(pucks_contours[i]), true)*0.02, true);
		pucks_closed_contours.push_back(maybe_puck);
	}

	// Calculate centers of contours.
	static Point2f center;
	static float radius;
	for (uint16_t i=0; i<pucks_closed_contours.size(); i++) {
		minEnclosingCircle((Mat) pucks_closed_contours[i], center, radius);

		pucks_encircle_centers.push_back(center);
		pucks_encircle_radii.push_back(radius);
	}

	// Check that the contour has a certain minimum area-to-enclosing-circle ratio that makes it pucky.
	for (uint16_t i=0; i<pucks_closed_contours.size(); i++) {
		if (fabs(contourArea(Mat(pucks_closed_contours[i]))) / (3.14159*pucks_encircle_radii[i]*pucks_encircle_radii[i]) > puckiness_min_ratio) {
			(*pucks).push_back(pucks_closed_contours[i]);
		}
	}
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

	static Mat rectified_image;
	rectify_board(&cv_ptr->image, &rectified_image);

	find_pucks(&rectified_image, &target_pucks);

	// Draw puck locations.
	Mat pucks_drawing = Mat::zeros(240, 320, CV_8UC3);
	for (uint16_t i=0; i<target_pucks.size(); i++) {
		Scalar color = Scalar(0, 255, 0);   // Green!
		if (pucks_encircle_radii[i] > encircle_min_size && pucks_encircle_radii[i] < encircle_max_size) {
			drawContours(pucks_drawing, target_pucks, i, color, 1, 8, vector<Vec4i>(), 0, Point());
			circle(pucks_drawing, pucks_encircle_centers[i], (int) pucks_encircle_radii[i], color, 2, 8, 0);
		}
	}

	// Draw the board.
	Scalar board_color = Scalar(0, 0, 255);   // Red.
	drawContours(pucks_drawing, board_closed_contours, 0, board_color, 1, 8, vector<Vec4i>(), 0, Point());

	// Show images.
	imshow(RAW_WINDOW, cv_ptr->image);
	imshow(BW_WINDOW, debug_image_1);
	imshow(PUCKS_WINDOW, pucks_drawing);

	// Wait 2 ms for a keypress.
	int c = waitKey(2);
	// Exit if the spacebar is pressed. NOTE: killing the program with
	// Ctrl+C sometimes stops OpenCV at a bad place and effects a kernel
	// panic! If you really like Ctrl+C, do so at your own risk.
	if ((char) c == 32) {
		return;
	}

	pc.x[0] = target_pucks.size();
	pc.x[1] = puck_erosion_iter;

	cb_vision_pub.publish(pc);
}

void CBPuckFinder::params_cb(const rqt_cb_gui::cb_params& msg)
{
	// Board-related parameters.
	board_hue_low  = msg.board_hue_low;
	board_hue_high = msg.board_hue_high;
	board_sat_low  = msg.board_sat_low;
	board_sat_high = msg.board_sat_high;
	board_val_low  = msg.board_val_low;
	board_val_high = msg.board_val_high;
	board_min_size = msg.board_min_size;
	board_erosion_iter  = msg.board_erosion_iter;
	board_dilation_iter = msg.board_dilation_iter;
	board_canny_lower_threshold = msg.board_canny_lower_threshold;

	// Puck-related parameters.
	puck_hue_low  = msg.puck_hue_low;
	puck_hue_high = msg.puck_hue_high;
	puck_sat_low  = msg.puck_sat_low;
	puck_sat_high = msg.puck_sat_high;
	puck_val_low  = msg.puck_val_low;
	puck_val_high = msg.puck_val_high;
	encircle_min_size = msg.encircle_min_size;
	encircle_max_size = msg.encircle_max_size;
	puck_erosion_iter  = msg.puck_erosion_iter;
	puckiness_min_ratio = msg.puckiness_min_ratio;
	puck_canny_lower_threshold = msg.puck_canny_lower_threshold;
}

void CBPuckFinder::get_parameters()
{
	nh_.getParam("corner0/x", board_corner_0_x);
	nh_.getParam("corner0/y", board_corner_0_y);
	nh_.getParam("corner1/x", board_corner_1_x);
	nh_.getParam("corner1/y", board_corner_1_y);
	nh_.getParam("corner2/x", board_corner_2_x);
	nh_.getParam("corner2/y", board_corner_2_y);
	nh_.getParam("corner3/x", board_corner_3_x);
	nh_.getParam("corner3/y", board_corner_3_y);
}

