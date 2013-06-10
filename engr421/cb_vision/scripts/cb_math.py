def transform(pc_coord, calib_x, calib_y):
    # Shift to the origin.
    pc_coord[0] -= calib_x[0]
    pc_coord[1] -= calib_y[0]



# vim: expandtab

