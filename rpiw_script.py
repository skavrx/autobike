from machine import Pin, I2C, UART, Timer
import time
import struct
import math

# Configuration for PID Controller
class PIDConfig:
    Kp = 3.5    # Proportional gain
    Ki = 0.45    # Integral gain
    Kd = 0.25    # Derivative gain

# Configuration for System Parameters
class SystemConfig:
    # Complementary filter coefficient
    COMP_FILTER_ALPHA = 0.9991

    # Sampling time in seconds
    SAMPLE_TIME = 0.01  # 100 Hz

    # Damping coefficient
    DAMPING_COEFF = 0.24  # Nm per rad/s

    # Thonny Plotter Configuration
    DATA_PRINT_INTERVAL = 0.1  # seconds
                    
    ANGLE_FIXRATE_BASE = 0.04
    ANGLE_FIXRATE_WHEEL_SPEED_DAMPING = 0.05
    FIXRATE_TIME_COEFF = 0.5
    
    MAX_TORQUE = 4.0  # Maximum torque in Nm
    MAX_TILT = 13    # degrees
    MIN_TILT = -13   # degrees
    MAX_SPEED = 0.01 
                
    # PID Controller Configuration
    PID_KP = PIDConfig.Kp
    PID_KI = PIDConfig.Ki
    PID_KD = PIDConfig.Kd

# MPU6050 Class
class MPU6050:
    def __init__(self, i2c, addr=0x68):
        self.i2c = i2c
        self.addr = addr
        self.gyro_offset = {'z': -19.525, 'y': 112.35, 'x': -54.9405} 
        self.accel_offset = {'z': -64.52246, 'y': 919.67, 'x': -1847.318}
        
    def begin(self):
        try:
            # Wake up MPU6050
            self.i2c.writeto_mem(self.addr, 0x6B, bytes([0]))
            # Set accelerometer range to ±2g
            self.i2c.writeto_mem(self.addr, 0x1C, bytes([0x00]))
            # Set gyro range to ±250°/s
            self.i2c.writeto_mem(self.addr, 0x1B, bytes([0x00]))
            return True
        except Exception as e:
            print(f"Error initializing MPU6050: {e}")
            return False

    def read_raw_data(self):
        try:
            # Read 14 bytes from MPU6050 starting at register 0x3B
            data = self.i2c.readfrom_mem(self.addr, 0x3B, 14)
            vals = struct.unpack('>hhhhhhh', data)
            ax = vals[0]
            ay = vals[1]
            az = vals[2]
            gx = vals[4]
            gy = vals[5]
            gz = vals[6]
            return ax, ay, az, gx, gy, gz
        except Exception as e:
            print(f"Error reading MPU6050 data: {e}")
            return 0, 0, 0, 0, 0, 0

    def calibrate(self, num_samples=1000, delay=0.005):
        print("Calibrating MPU6050...")
        gyro_bias = {'x': 0, 'y': 0, 'z': 0}
        accel_bias = {'x': 0, 'y': 0, 'z': 0}

        for _ in range(num_samples):
            ax, ay, az, gx, gy, gz = self.read_raw_data()
            gyro_bias['x'] += gx
            gyro_bias['y'] += gy
            gyro_bias['z'] += gz
            accel_bias['x'] += ax
            accel_bias['y'] += ay
            accel_bias['z'] += az
            time.sleep(delay)

        self.gyro_offset = {k: v / num_samples for k, v in gyro_bias.items()}
        self.accel_offset = {k: v / num_samples for k, v in accel_bias.items()}
        self.accel_offset['z'] -= 16384  # Adjust for gravity

        print("Calibration complete.")
        print(f"Gyro biases: {self.gyro_offset}")
        print(f"Accel biases: {self.accel_offset}")

    def get_calibrated_data(self):
        ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw = self.read_raw_data()
        # Apply calibration offsets
        ax = (ax_raw - self.accel_offset['x']) / 16384.0  # Convert to g
        ay = (ay_raw - self.accel_offset['y']) / 16384.0
        az = (az_raw - self.accel_offset['z']) / 16384.0
        gx = (gx_raw - self.gyro_offset['x']) / 131.0  # Convert to °/s
        gy = (gy_raw - self.gyro_offset['y']) / 131.0
        gz = (gz_raw - self.gyro_offset['z']) / 131.0
        return ax, ay, az, gx, gy, gz

# ODriveController Class
class ODriveController:
    def __init__(self, uart, motor_number=0):
        self.uart = uart
        self.motor_number = motor_number

    def begin(self):
        try:
            # Clear errors
            self.send_command('sc\n')
            time.sleep(0.1)

            # Set control mode to torque control
            self.send_command(f'w axis{self.motor_number}.controller.config.control_mode 1\n')
            time.sleep(0.1)

            # Set input mode to torque
            self.send_command(f'w axis{self.motor_number}.controller.config.input_mode 1\n')
            time.sleep(0.1)

            # Request closed-loop control
            self.send_command(f'w axis{self.motor_number}.requested_state 8\n')
            time.sleep(0.1)

            # Verify state
            self.send_command(f'r axis{self.motor_number}.current_state\n')
            response = self.read_response(timeout=100)
            if '8' not in response:
                print(f"Axis{self.motor_number} failed to enter CLOSED_LOOP_CONTROL state.")
                return False

            print(f"Axis{self.motor_number} initialized in CLOSED_LOOP_CONTROL state.")
            return True
        except Exception as e:
            print(f"ODrive initialization error: {e}")
            return False
    
    def calibrate_motor(self):
        print("Starting ODrive calibration sequence...")
        try:
            # Enter motor calibration mode
            self.send_command(f'w axis{self.motor_number}.requested_state 3\n')  # 3: MOTOR_CALIBRATION
            time.sleep(7)  # Wait for motor calibration to complete
            
            # Check calibration result
            self.send_command(f'r axis{self.motor_number}.motor.is_calibrated\n')
            response = self.read_response(timeout=100)
            if '1' not in response:
                print("Motor calibration failed!")
                return False
            
            print("Motor calibration completed successfully.")
            return True
        except Exception as e:
            print(f"Motor calibration error: {e}")
            return False

    def calibrate_encoder(self):
        print("Starting encoder calibration...")
        try:
            # Enter encoder calibration mode
            self.send_command(f'w axis{self.motor_number}.requested_state 7\n')  # 7: ENCODER_CALIBRATION
            time.sleep(10)  # Wait for encoder calibration
            
            # Check calibration result
            self.send_command(f'r axis{self.motor_number}.encoder.is_ready\n')
            response = self.read_response(timeout=100)
            if '1' not in response:
                print("Encoder calibration failed!")
                return False
            
            print("Encoder calibration completed successfully.")
            return True
        except Exception as e:
            print(f"Encoder calibration error: {e}")
            return False

    def run_index_search(self):
        print("Starting encoder index search...")
        try:
            # Start index search
            self.send_command(f'w axis{self.motor_number}.requested_state 6\n')  # 6: ENCODER_INDEX_SEARCH
            time.sleep(10)  # Wait for index search to complete
            
            # Verify index was found
            self.send_command(f'r axis{self.motor_number}.encoder.index_found\n')
            response = self.read_response(timeout=100)
            if '1' not in response:
                print("Index search failed!")
                return False
            
            print("Index search completed successfully.")
            return True
        except Exception as e:
            print(f"Index search error: {e}")
            return False
        
    def set_torque(self, torque):
        # Limit torque
        torque = max(min(torque, SystemConfig.MAX_TORQUE), -SystemConfig.MAX_TORQUE)
        # Send torque command
        self.send_command(f'c {self.motor_number} {torque}\n')

    def send_command(self, command):
        self.uart.write(command.encode())

    def read_response(self, timeout=100):
        """
        Reads response from ODrive with a specified timeout in milliseconds.
        Shorter timeout for faster reading.
        """
        response = b""
        start_time = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start_time) < timeout:  # timeout in ms
            if self.uart.any():
                response += self.uart.read(self.uart.any())
                if b'\n' in response:
                    break
            # Minimal sleep to prevent busy-waiting
            time.sleep(0.01)
        return response.decode()

    def get_wheel_speed(self):
        """
        Retrieves the wheel speed from ODrive.
        """
        # Send command to read wheel speed
        self.send_command(f'r encoder_estimator0.vel_estimate\n')
        response = self.read_response(timeout=100)  # Faster timeout

        try:
            # Extract numeric value
            response_value = response.strip()
            q2_dot_odrive_units = float(response_value)
            # The units depend on the ODrive firmware settings
            # Assume units are in counts/s, convert to radians/s
            ENCODER_CPR = 8192  # Example value; replace with your encoder's CPR
            q2_dot_rad_per_sec = q2_dot_odrive_units * (2 * math.pi / ENCODER_CPR)
            return q2_dot_rad_per_sec
        except ValueError:
            print(f"Failed to parse wheel speed from response: '{response}'")
            return 0.0

    def emergency_stop(self):
        # Set motor to idle state
        self.send_command(f'w axis{self.motor_number}.requested_state 1\n')  # 1: IDLE
        print(f"Axis{self.motor_number} set to IDLE state.")
        # Optionally, clear errors
        self.send_command('sc\n')

# PID Controller Class
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint

        self.integral = 0.0
        self.previous_error = 0.0

    def compute(self, measurement, dt):
        error = self.setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
        self.previous_error = error

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return output
    
    def setsetpoint(self, point):
        self.setpoint = point

# Main Function
def main():
    # Initialize I2C for MPU6050
    i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)

    # Initialize UART for ODrive
    uart = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))

    # Initialize MPU6050
    mpu = MPU6050(i2c)
    if not mpu.begin():
        print("Failed to initialize MPU6050!")
        return

    # Calibrate MPU6050
    # mpu.calibrate()

    # Initialize ODrive
    motor = ODriveController(uart)
    
    initialization_successful = False
    
    while not initialization_successful:
        if motor.begin():
            initialization_successful = True
        else:
            print("ODrive initialization failed. Starting calibration sequence...")
            
            # Perform calibration sequence
            if not motor.calibrate_motor():
                print("Motor calibration failed! Retrying in 5 seconds...")
                time.sleep(5)
                continue
                
            if not motor.calibrate_encoder():
                print("Encoder calibration failed! Retrying in 5 seconds...")
                time.sleep(5)
                continue
                
            if not motor.run_index_search():
                print("Index search failed! Retrying in 5 seconds...")
                time.sleep(5)
                continue
                
            print("Calibration sequence completed. Waiting 5 seconds before starting control...")
            time.sleep(5)
            
            # Try initialization again
            continue

    print("ODrive initialization successful. Starting control loop...")

    if not motor.begin():
        print("Failed to initialize ODrive!")
        return

    # Initialize PID Controller
    pid = PIDController(Kp=SystemConfig.PID_KP, Ki=SystemConfig.PID_KI, Kd=SystemConfig.PID_KD)

    # Complementary filter parameters
    alpha = SystemConfig.COMP_FILTER_ALPHA
    rad_to_deg = 57.2958  # Conversion from radians to degrees

    # Initialize angles
    gyro_angle = 0.0
    comp_angle = 0.0

    # Timing
    last_time = time.ticks_ms()
    last_print_time = time.ticks_ms()

    print("\nStarting data stream. Press Ctrl+C to stop.")
    print("Time(s),Accel_Angle(deg),Gyro_Angle(deg),Comp_Angle(deg),Wheel_Speed(rad/s),Torque(Nm)")

    try:
        while True:
            # Read sensor data
            ax, ay, az, gx, gy, gz = mpu.get_calibrated_data()

            # Calculate accelerometer angle (roll)
            accel_angle = math.atan2(ay, az) * rad_to_deg

            # Calculate time delta
            current_time = time.ticks_ms()
            dt = (time.ticks_diff(current_time, last_time)) / 1000.0  # Convert ms to s
            last_time = current_time

            # Integrate gyroscope data to get gyro angle
            gyro_angle += gx * dt

            # Complementary filter to get filtered angle
            comp_angle = alpha * (comp_angle + gx * dt) + (1 - alpha) * accel_angle

            # PID controller to compute torque
            torque = pid.compute(comp_angle, dt)

            # Get wheel speed from ODrive
            wheel_speed = motor.get_wheel_speed()  # radians per second

            # Compute damping torque
            torque_damping = SystemConfig.DAMPING_COEFF * abs(wheel_speed)

            # Apply damping to the torque command
            torque -= torque_damping

            
            # Enforce torque limits after damping
            torque = max(min(torque, SystemConfig.MAX_TORQUE), -SystemConfig.MAX_TORQUE)
            ANGLE_FIXRATE = abs(SystemConfig.ANGLE_FIXRATE_BASE - (abs(wheel_speed) * SystemConfig.ANGLE_FIXRATE_WHEEL_SPEED_DAMPING))
            targetAngle = pid.setpoint
            dtp = dt * SystemConfig.FIXRATE_TIME_COEFF
            if (comp_angle < targetAngle):
                targetAngle += ANGLE_FIXRATE * dtp
            else:
                targetAngle -= ANGLE_FIXRATE * dtp
            #REDO
            # Only move angle opposite to the wheel speed correction
            if (wheel_speed < 0):
                targetAngle -= ANGLE_FIXRATE * dtp * 1.5 
            else:
                targetAngle += ANGLE_FIXRATE * dtp * 1.5
                
            pid.setsetpoint(targetAngle)
            # Emergency Stop: Check if pendulum angle exceeds safety limits
            if comp_angle > SystemConfig.MAX_TILT or comp_angle < SystemConfig.MIN_TILT or abs(wheel_speed) > SystemConfig.MAX_SPEED:
                print(f"Emergency Stop Triggered! Angle: {comp_angle:.2f}°")
                motor.emergency_stop()
                break  # Exit the control loop

            # Send torque to ODrive
            motor.set_torque(torque)

            # Get elapsed time in seconds
            elapsed_time = current_time / 1000.0

            # Print data for Thonny plotter at defined intervals
            if (time.ticks_diff(current_time, last_print_time) / 1000.0) > SystemConfig.DATA_PRINT_INTERVAL:
                #print(f"{accel_angle:.2f},{gyro_angle:.2f},{comp_angle:.2f},{wheel_speed:.f},{torque:.2f}")
                last_print_time = current_time

            # Sleep to maintain loop rate
            time.sleep(SystemConfig.SAMPLE_TIME)

    except KeyboardInterrupt:
        # Handle emergency stop on manual interruption
        motor.emergency_stop()
        print("\nEmergency stop triggered. System halted.")

if __name__ == "__main__":
    main()