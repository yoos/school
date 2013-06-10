def two_on_one(pc):
    if pc.puck_count == 2:   # One shooter per puck.
        if pc.x[0] < pc.x[1]:
            left, right = pc.x[0], pc.x[1]
        else:
            left, right = pc.x[1], pc.x[0]
    elif pc.puck_count == 1:   # Both shooters on one puck.
        left  = max(0.0, pc.x[0] - RAIL_MIN_SEPARATION/2)
        right = min(1.0, pc.x[0] + RAIL_MIN_SEPARATION/2)
    else:   # Standby stance.
        left  = 0.0
        right = 1.0

    return left, right

