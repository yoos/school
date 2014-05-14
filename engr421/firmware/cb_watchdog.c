#include <cb_watchdog.h>

uint8_t feed_linear_rail_watchdog(float dc)
{
	static float cumulative_load = 0.0;   /* Electrical load over time */

	cumulative_load += (dc - LINEAR_RAIL_WATCHDOG_COOLDOWN) * LINEAR_RAIL_DT;

	if (cumulative_load < 0) {
		cumulative_load == 0;
	}
	else if (cumulative_load > LINEAR_RAIL_WATCHDOG_LD) {
		return 1;
	}

	return 0;
}

