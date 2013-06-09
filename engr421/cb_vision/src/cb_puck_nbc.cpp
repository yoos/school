#include <cb_vision/cb_puck_nbc.h>

CBNaiveBayesPuckifier::CBNaiveBayesPuckifier(void)
{
	// Do nothing
}

void CBNaiveBayesPuckifier::set_ideal_puck_features(puck_features pf)
{
	ipf.encircle_size = pf.encircle_size;
	ipf.puck_encircle_ratio = pf.puck_encircle_ratio;
	ipf.dist_last_closest_puck = pf.dist_last_closest_puck;
}

void CBNaiveBayesPuckifier::add_potential_puck(Point2f loc, puck_features pf)
{
	puck_parameter new_pp;
	new_pp.loc = loc;

	new_pp.prob.encircle_size  = 1.0 - MIN(1.0, ABS(ipf.encircle_size - pf.encircle_size)/5);
	new_pp.prob.puck_encircle_ratio = 1.0 - MIN(1.0, ABS(ipf.puck_encircle_ratio - pf.puck_encircle_ratio));
	new_pp.prob.dist_last_closest_puck = 1.0 - MIN(1.0, 0.0);   // TODO

	new_pp.puckiness = new_pp.prob.encircle_size * new_pp.prob.puck_encircle_ratio * new_pp.prob.dist_last_closest_puck;

	// Keep track of all the potential pucks, just in case. I might remove this
	// later.
	pp.push_back(new_pp);

	// Keep track of the two puckiest pucks.
	if (new_pp.puckiness > puckiest_pucks[0].puckiness) {
		puckiest_pucks[1] = puckiest_pucks[0];
		puckiest_pucks[0] = new_pp;
	}
	else if (new_pp.puckiness > puckiest_pucks[1].puckiness) {
		puckiest_pucks[1] = new_pp;
	}
}

void CBNaiveBayesPuckifier::get_puckiest_pucks(Point2f pucks[2])
{
	pucks[0] = puckiest_pucks[0].loc;
	pucks[1] = puckiest_pucks[1].loc;
}

