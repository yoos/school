#include <cb_watchdog.h>
#include <chsprintf.h>

static float dbg_load;
static uint8_t dbg_alive;

uint8_t feed_linear_rail_watchdog(uint8_t dir, float dc)
{
	static float cumulative_load = 0.0;   /* Electrical load over time */
	static uint8_t is_alive = 1;

	if (dir) {
		dc = 1.0 - dc;
	}

	cumulative_load += (dc - LINEAR_RAIL_WATCHDOG_COOLDOWN) * LINEAR_RAIL_DT;

	if (is_alive) {
		/* Cap to zero. */
		if (cumulative_load < 0) {
			cumulative_load = 0;
		}
		/* Check for lethal cumulative load. */
		else if (cumulative_load > LINEAR_RAIL_WATCHDOG_LD) {
			is_alive = 0;
		}
	}
	else {
		/* Check if ready to revive. */
		if (cumulative_load < 0.001) {
			is_alive = 1;
		}
	}

	/* Debug */
	dbg_load = cumulative_load;
	dbg_alive = is_alive;

	return is_alive;

}

void linear_rail_watchdog_debug_output(uint8_t *buffer)
{
	chsprintf(buffer, "LR Watchdog load: %u   Alive: %u\r\n", (uint16_t) (dbg_load*1000), dbg_alive);
}

