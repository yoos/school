def transform(pc_coord, calib_x):
    calib_loc = [0.0] * 2
    for i in range(2):
        calib_loc[i] = (pc_coord[i] - calib_x[0]) / (calib_x[1] - calib_x[0])

    return [calib_loc[0], calib_loc[1]]

# vim: expandtab

