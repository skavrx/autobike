import odrive
from odrive.enums import *

import time
from time import sleep, perf_counter_ns

print("finding an odrive...")
odrv0 = odrive.find_any()

print("starting calibration...")
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while odrv0.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)

odrv0.axis0.requested_state = AXIS_STATE_IDLE
