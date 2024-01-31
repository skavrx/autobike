#################################################
# Reaction Wheel Inverted Pendulum
# Created by: Brick Experiment Channel
# Hardware:
#    MPU9250 IMU
#    TB6612FNG H-Bridge
#    Lego EV3 Medium motor
#################################################

#settings
ANGLE_FILTER_G    = 0.999 #gyro portion of complementary filter [0...1]
ANGLE_LIMIT       = 10    #stop program / start rise up sequence [deg]
RISEUP_END_ANGLE  = 5     #stop rise up sequence and start balancing [deg]
ANGLE_FIXRATE     = 1.0   #variate target angle [deg/s]
ANGLE_FIXRATE_2   = 0.1   #reduce continuous rotation
KP                = 13   #PID proportional factor
KI                = 0   #PID integral factor
KD                = 0 #PID derivative factor
MOTOR_R           = 13    #motor resistance [Ohm]
MOTOR_Ke          = 0.26  #motor back EMF constant [Vs/rad]
SUPPLY_VOLTAGE    = 8.1   #battery box voltage [V]
WHEEL_AV_INTERVAL = 100   #wheel angular velocity calculation interval [ms]
SLEEP_TIME        = 1     #main loop sleep [ms]

############################################################

import odrive
from odrive.enums import *

import time
from time import sleep, perf_counter_ns
from datetime import datetime
import math
import signal
import smbus

from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

imu = MPU9250(bus=1, gfs=GFS_250, afs=AFS_2G)
imu.abias = [-0.036993408203125, 0.0659057671875, 0.007368039843749911]
imu.gbias = [-0.5022048950195312, 0.7293076171874999, -0.01735687255859375]
imu.configure()

#exit program when Ctrl-C is pressed
exitRequested = False
def sigintHandler(sig, frame):
    print("Ctrl-C pressed, exit program")
    global exitRequested
    exitRequested = True
signal.signal(signal.SIGINT, sigintHandler)
signal.signal(signal.SIGTERM, sigintHandler)

print("program started")

print("finding an odrive...")
odrv0 = odrive.find_any()

# Calibrate motor and wait for it to finish
#print("starting calibration...")
#odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
#while odrv0.axis0.current_state != AXIS_STATE_IDLE:
#    time.sleep(0.1)

odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

sleep(0.1)

print("Bus voltage is: " + str(odrv0.vbus_voltage) + "V")

startTime = perf_counter_ns()
prevLoopTime = perf_counter_ns()
prevTachoTime = perf_counter_ns()
prevPrintTime = perf_counter_ns()
loopCount = 0
gyroAngle = float('nan')
measuredAngle = float('nan')
risingUp = False
prevTachoCount = 0
wheelAV = 0
targetAngle = 0
error = 0
prevError = 0
integral = 0
derivative = 0
PIDoutput = 0
motorCtrl = 0
logData = [["secondsSinceStart","accAngle","gyroAngle","measuredAngle","targetAngle",
            "tachoCount","wheelAV","error","integral","derivative",
            "PIDoutput","motorCtrl"]]

while not exitRequested:
    timeDelta = (perf_counter_ns() - prevLoopTime) / 1e9  #[sec]
    prevLoopTime = perf_counter_ns()
    secondsSinceStart = (perf_counter_ns() - startTime) / 1e9

    #read accelerometer
    axr, ayr, azr = imu.readAccelerometerMaster()
    print(f"Axr: {axr} Ayr: {ayr} Azr: {azr}")
    
    ax, ay, az = ayr, -1 * azr, -1 * axr
    accAngle = math.atan(-ax / math.sqrt(pow(ay, 2) + pow(az, 2))) * 180 / math.pi  #[deg]
    print("AccAngle: " + str(accAngle))
    
    #read gyroscope
    gxr, gyr, gzr = imu.readGyroscopeMaster()
    gx, gy, gz = gyr, -1 * gzr, -1 * gxr
    gyroAngleDelta = gy * timeDelta
    if math.isnan(gyroAngle): gyroAngle = accAngle
    gyroAngle += gyroAngleDelta  #[deg]
    print("GyroAngle: "+str(gyroAngle))
    #calculate arm angle (complementary filter)
    if math.isnan(measuredAngle): measuredAngle = accAngle
    measuredAngle = (ANGLE_FILTER_G * (measuredAngle + gyroAngleDelta) +
                     (1-ANGLE_FILTER_G) * accAngle)  #[deg]
    
    #safety check
    if abs(measuredAngle) >= ANGLE_LIMIT:
        if secondsSinceStart < 0.001:
            print("START RISE UP SEQUENCE")
        elif not risingUp:
            print("PROGRAM STOPPED, angle is too large\nmeasuredAngle: %.2f\naccAngle: %.2f\ngyroAngleDelta: %.2f" % (measuredAngle, accAngle, gyroAngleDelta))
            break    
    
    #calculate wheel angular velocity
    if (perf_counter_ns() - prevTachoTime) / 1e6 >= WHEEL_AV_INTERVAL:
        tachoTimeDelta = (perf_counter_ns() - prevTachoTime) / 1e9  #[sec]
        prevTachoTime = perf_counter_ns()
        
        tps = odrv0.axis0.pos_vel_mapper.vel 
        wheelAV = tps * 2 * math.pi 

    if not risingUp:
    
        #variate target angle
        if measuredAngle < targetAngle:
            targetAngle += ANGLE_FIXRATE * timeDelta
        else:
            targetAngle -= ANGLE_FIXRATE * timeDelta
        
        #reduce continuous rotation
        targetAngle -= ANGLE_FIXRATE_2 * wheelAV * timeDelta
        
        #PID controller
        error = targetAngle - measuredAngle
        integral += error * timeDelta
        derivative = (error - prevError) / timeDelta
        prevError = error
        PIDoutput = KP * error + KI * integral + KD * derivative

        print("Target:" + str(PIDoutput))
        odrv0.axis0.controller.input_vel = PIDoutput # Should be in [turns/s] for odrive input
    
    #log data for post analysis
    logData.append([secondsSinceStart, accAngle, gyroAngle, measuredAngle, targetAngle,
                wheelAV, error, integral, derivative,
                    PIDoutput, motorCtrl])
    
    #debug print
    if (perf_counter_ns() - prevPrintTime) / 1e9 >= 1.0:
        secondsSinceLastPrint = (perf_counter_ns() - prevPrintTime) / 1e9
        prevPrintTime = perf_counter_ns()
        loopInterval = secondsSinceLastPrint / loopCount * 1000
        loopCount = 0
        print("measuredAngle: %.2f, motorCtrl: %.2f, loopInterval: %.2f ms"
              % (measuredAngle, motorCtrl, loopInterval))
    
    sleep(SLEEP_TIME / 1000)
    loopCount += 1

#stop motor
odrv0.axis0.controller.input_vel = 0 # Should be in [turns/s] for odrive input

#write log data to file
print("log size", len(logData), "rows")
if len(logData) > 0:
    filename = "datalog.dat"
    #filename = "datalog_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".dat"
    print("write to file:", filename)
    file = open(filename, "w")
    for logLine in logData:
        for value in logLine:
            file.write(str(value) + ' ')
        file.write('\n')
    file.close()

odrv0.axis0.requested_state = AXIS_STATE_IDLE
print("program ended")
