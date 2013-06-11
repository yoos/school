import cb_config as cfg

def transform(puck_loc, calib_x):
    calib_loc = [[0.0, 0.0], [0.0, 0.0]]
    for i in range(2):
        calib_loc[i][0] = (puck_loc[i][0] - calib_x[0]) / (calib_x[1] - calib_x[0])
        calib_loc[i][1] = puck_loc[i][1]

    return calib_loc

puck_loc_filter_count = [cfg.PUCK_LOC_CONFIRM] * 2

def filter_puck_loc(puck_loc_old, puck_loc_new):
    puck_loc_filtered = [[0.0] * 2] * 2

    for i in range(2):
#        if abs(puck_loc_old[i][0] - puck_loc_new[i][0]) < cfg.PUCK_MAX_MOVEMENT_STEP and abs(puck_loc_old[i][1] - puck_loc_new[i][1]) < cfg.PUCK_MAX_MOVEMENT_STEP and puck_loc_filter_count[i] == 0:
        if True:
            puck_loc_filtered[i][0] = puck_loc_new[i][0]
            puck_loc_filtered[i][1] = puck_loc_new[i][1]
            puck_loc_filter_count[i] = cfg.PUCK_LOC_CONFIRM
        else:
            puck_loc_filtered[i][0] = puck_loc_old[i][0]
            puck_loc_filtered[i][1] = puck_loc_old[i][1]
            puck_loc_filter_count[i] -= 1

    return puck_loc_filtered

# vim: expandtab

