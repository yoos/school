#ifndef CB_PUCK_NBC_H
#define CB_PUCK_NBC_H

#include <ros/ros.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <cb_vision/cb_math.h>


using namespace cv;

typedef struct {
	float encircle_size;   // Size of minimum enclosing circle.
	float puck_encircle_ratio;   // Ratio of contour to enclosing circle.
	float dist_last_closest_puck;   // Distance to the last closest puck.
} puck_features;

typedef struct {
	float encircle_size;
	float puck_encircle_ratio;
	float dist_last_closest_puck;
} puck_probabilities;

typedef struct
{
	Point2f            loc;   // Location of puck so we can use it later.
	puck_probabilities prob;
	float              puckiness;   // Probability from 0.0 to 1.0 that this is a puck.
} puck_parameter;


class CBNaiveBayesPuckifier
{
	puck_features ipf;   // Ideal puck features.
	vector<puck_parameter> pp;   // Puck parameters.
	puck_parameter puckiest_pucks[2];

public:
	CBNaiveBayesPuckifier(void);

	void set_ideal_puck_features(puck_features pf);
	void add_potential_puck(Point2f loc, puck_features pf);
	void get_puckiest_pucks(Point2f pucks[2]);
};

#endif // CB_PUCK_NBC_H

