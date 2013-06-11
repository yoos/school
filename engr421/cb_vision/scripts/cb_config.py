import strategies as s

#serialPort = '/dev/ttyUSB0'
baudrate = '460800'
strategy = s.two_on_one

PUCK_COUNT_CONFIRM = 5
PUCK_LOC_CONFIRM = 10
PUCK_MAX_MOVEMENT_STEP = 0.1

### RAIL CONFIG ###
RAIL_OFFSET = [0.030, 0.920]
#RAIL_OFFSET = [0.0, 1.0]
RAIL_MIN_SEPARATION = 0.112   # This should agree with the firmware value in cb_config.h.

