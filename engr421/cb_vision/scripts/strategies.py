import cb_config as cfg

def two_on_one(puck_count, puck_loc):
    if puck_count >= 2:   # One shooter per puck.
        if puck_loc[0][0] < puck_loc[1][0]:
            left, right = puck_loc[0][0], puck_loc[1][0]
        else:
            left, right = puck_loc[1][0], puck_loc[0][0]
    elif puck_count == 1:   # Both shooters on one puck.
        left  = puck_loc[0][0] - cfg.RAIL_MIN_SEPARATION/2
        right = puck_loc[0][0] + cfg.RAIL_MIN_SEPARATION/2
    else:   # Standby stance.
        left  = 0.0
        right = 1.0

    # Cap outputs.
    left  = min(1.0, max(0.0, left))
    right = max(0.0, min(1.0, right))

    return left, right

