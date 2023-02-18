import os
import time
import math
import atexit
import logging

import rossros as rr
from logdecorator import log_on_start , log_on_end , log_on_error

import greyscale as gs
import ultrasonic as us
from bus import Bus, ConcurrentHelper

try:
    from robot_hat import *
    from robot_hat import reset_mcu

    reset_mcu()
    time.sleep (0.01)

    # user and User home directory
    User = os.popen('echo ${SUDO_USER:-$LOGNAME}').readline().strip()
    UserHome = os.popen('getent passwd %s | cut -d: -f 6'%User).readline().strip()
    # print(User)  # pi
    # print(UserHome) # /home/pi
    config_file = '%s/.config/picar-x/picar-x.conf'%UserHome

except ImportError:

    print("""This computer does not appear to be a PiCar -X system
    (robot_hat is not present). Shadowing hardware calls with
    substitute functions""")

    from sim_robot_hat import *


# Constants

SPEED_FACTOR = 3.45  # Measured speed for motor at %50 motor torque for 1 second


def controlled_movement(func):
    """Modifies the movement methods, so it the car can be
    controlled more precisely.
    """

    def ctrl_move_wrapper(self, dist, angle: int = 0):

        # Command car to move in direction
        self.set_dir_servo_angle(angle)
        func()

        # Wait for adequate time to pass to travel the given distance
        travel_time = dist/SPEED_FACTOR
        time.sleep(travel_time)
        
        # Turn off motors and reset direction servo
        self.stop()
        self.set_dir_servo_angle(0)
    
    return ctrl_move_wrapper


class Picarx(object):

    CAR_L = 3.75  # Inches
    WHEELBASE_L = 4.5  # Inches
    PERIOD = 4095
    PRESCALER = 10
    TIMEOUT = 0.02

    # servo_pins: direction_servo, camera_servo_1, camera_servo_2 
    # motor_pins: left_swicth, right_swicth, left_pwm, right_pwm
    # grayscale_pins: 3 adc channels
    # ultrasonic_pins: tring, echo
    # config: path of config file
    def __init__(self, 
                servo_pins:list=['P0', 'P1', 'P2'], 
                motor_pins:list=['D4', 'D5', 'P12', 'P13'],
                config:str=config_file,
                ):

        self.angular_vel = 0  # Constant angular velocity the car should run for

        # config_flie
        self.config_flie = fileDB(config, 774, User)

        # servos init 
        self.camera_servo_pin1 = Servo(PWM(servo_pins[0]))
        self.camera_servo_pin2 = Servo(PWM(servo_pins[1]))   
        self.dir_servo_pin = Servo(PWM(servo_pins[2])) 
        self.dir_cal_value = int(self.config_flie.get("picarx_dir_servo", default_value=0))
        self.cam_cal_value_1 = int(self.config_flie.get("picarx_cam_servo1", default_value=0))
        self.cam_cal_value_2 = int(self.config_flie.get("picarx_cam_servo2", default_value=0))
        self.dir_servo_pin.angle(self.dir_cal_value)
        self.camera_servo_pin1.angle(self.cam_cal_value_1)
        self.camera_servo_pin2.angle(self.cam_cal_value_2)

        # motors init
        self.left_rear_dir_pin = Pin(motor_pins[0])
        self.right_rear_dir_pin = Pin(motor_pins[1])
        self.left_rear_pwm_pin = PWM(motor_pins[2])
        self.right_rear_pwm_pin = PWM(motor_pins[3])
        self.motor_direction_pins = [self.left_rear_dir_pin, self.right_rear_dir_pin]
        self.motor_speed_pins = [self.left_rear_pwm_pin, self.right_rear_pwm_pin]
        self.cali_dir_value = self.config_flie.get("picarx_dir_motor", default_value="[1,1]")
        self.cali_dir_value = [int(i.strip()) for i in self.cali_dir_value.strip("[]").split(",")]
        self.cali_speed_value = [0, 0]
        self.dir_current_angle = 0
        for pin in self.motor_speed_pins:
            pin.period(self.PERIOD)
            pin.prescaler(self.PRESCALER)

        atexit.register(self.stop)
    
    @log_on_start(logging.DEBUG , "Set motor speed started")
    @log_on_error(logging.DEBUG , "Set motor speed encountered an error")
    @log_on_end(logging.DEBUG , "Set motor speed ended")
    def calibrate_steering(self):

        print("The car will now move forward")

        self.forward(20)

        try:
            side_dist = float(input(f"Your car traveled a distance of 20 cm. How far did it travel from the straight line (in cm)? "))
            direction = input("In what direction (left [l] or right [r])?")
            if direction not in ("l", "r"):
                raise TypeError
            sign_bias =  1 if direction == 'r' else -1
            forward_dist = float(input("How much did it travel in the forward direction? "))

            # Set new angle and update config file
            self.set_dir_servo_angle(self.dir_servo_pin.angle(self.dir_servo_pin.angle - math.degrees(math.atan(sign_bias * (side_dist/forward_dist)))))


        except TypeError:
            print("Encountered an error in your input. Failed to calibrate")
        

    @log_on_start(logging.DEBUG , "Set motor speed started")
    @log_on_error(logging.DEBUG , "Set motor speed encountered an error")
    @log_on_end(logging.DEBUG , "Set motor speed ended")
    def set_motor_speed(self,motor,speed):
        # global cali_speed_value,cali_dir_value
        motor -= 1
        if speed >= 0:
            direction = 1 * self.cali_dir_value[motor]
        elif speed < 0:
            direction = -1 * self.cali_dir_value[motor]
        speed = abs(speed)
        if speed != 0:
            speed = int(speed /2 ) + 50
        speed = speed - self.cali_speed_value[motor]
        if direction < 0:
            self.motor_direction_pins[motor].high()
            self.motor_speed_pins[motor].pulse_width_percent(speed)
        else:
            self.motor_direction_pins[motor].low()
            self.motor_speed_pins[motor].pulse_width_percent(speed)

    @log_on_start(logging.DEBUG , "Motor speed calibration started")
    @log_on_error(logging.DEBUG , "Motor speed calibration encountered an error")
    @log_on_end(logging.DEBUG , "Motor speed calibration ended")
    def motor_speed_calibration(self,value):
        # global cali_speed_value,cali_dir_value
        self.cali_speed_value = value
        if value < 0:
            self.cali_speed_value[0] = 0
            self.cali_speed_value[1] = abs(self.cali_speed_value)
        else:
            self.cali_speed_value[0] = abs(self.cali_speed_value)
            self.cali_speed_value[1] = 0

    @log_on_start(logging.DEBUG , "Motor direction calibration started")
    @log_on_error(logging.DEBUG , "Motor direction calibration encountered an error")
    @log_on_end(logging.DEBUG , "Motor direction calibration ended")
    def motor_direction_calibration(self,motor, value):
        # 1: positive direction
        # -1:negative direction
        motor -= 1
        # if value == 1:
        #     self.cali_dir_value[motor] = -1 * self.cali_dir_value[motor]
        # self.config_flie.set("picarx_dir_motor", self.cali_dir_value)
        if value == 1:
            self.cali_dir_value[motor] = 1
        elif value == -1:
            self.cali_dir_value[motor] = -1
        self.config_flie.set("picarx_dir_motor", self.cali_dir_value)

    @log_on_start(logging.DEBUG , "Servo direction angle calibration started")
    @log_on_error(logging.DEBUG , "servo direction angle calibration encountered an error")
    @log_on_end(logging.DEBUG , "servo direction angle calibration ended")
    def dir_servo_angle_calibration(self,value):
        self.dir_cal_value = value
        self.config_flie.set("picarx_dir_servo", "%s"%value)
        self.dir_servo_pin.angle(value)

    @log_on_start(logging.DEBUG , "Set servo direction angle started")
    @log_on_error(logging.DEBUG , "Set servo direction angle encountered an error")
    @log_on_end(logging.DEBUG , "Set servo direction angle ended")
    def set_dir_servo_angle(self,value):
        self.dir_current_angle = value
        angle_value  = value + self.dir_cal_value
        self.dir_servo_pin.angle(angle_value)

    @log_on_start(logging.DEBUG , "Camera servo 1 angle calibration started")
    @log_on_error(logging.DEBUG , "Camera servo 1 angle calibration encountered an error")
    @log_on_end(logging.DEBUG , "Camera servo 1 angle calibration ended")
    def camera_servo1_angle_calibration(self,value):
        self.cam_cal_value_1 = value
        self.config_flie.set("picarx_cam_servo1", "%s"%value)
        self.camera_servo_pin1.angle(value)

    @log_on_start(logging.DEBUG , "Camera servo 2 angle calibration started")
    @log_on_error(logging.DEBUG , "Camera servo 2 angle calibration encountered an error")
    @log_on_end(logging.DEBUG , "Camera servo 2 angle calibration ended")
    def camera_servo2_angle_calibration(self,value):
        self.cam_cal_value_2 = value
        self.config_flie.set("picarx_cam_servo2", "%s"%value)
        self.camera_servo_pin2.angle(value)

    @log_on_start(logging.DEBUG , "Set camera servo 1 angle started")
    @log_on_error(logging.DEBUG , "Set camera servo 1 encountered an error")
    @log_on_end(logging.DEBUG , "Set camera servo 1 ended")
    def set_camera_servo1_angle(self,value):
        self.camera_servo_pin1.angle(-1*(value + -1*self.cam_cal_value_1))

    @log_on_start(logging.DEBUG , "Set camera servo 2 angle started")
    @log_on_error(logging.DEBUG , "Set camera servo 2 encountered an error")
    @log_on_end(logging.DEBUG , "Set camera servo 2 ended")
    def set_camera_servo2_angle(self,value):
        self.camera_servo_pin2.angle(-1*(value + -1*self.cam_cal_value_2))

    @log_on_start(logging.DEBUG , "Set power started")
    @log_on_error(logging.DEBUG , "Set power an error")
    @log_on_end(logging.DEBUG , "Set power ended")
    def set_power(self,speed):
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed) 

    @log_on_start(logging.DEBUG , "Backward movement started")
    @log_on_error(logging.DEBUG , "Backward movement encountered an error")
    @log_on_end(logging.DEBUG , "Backward movement ended")
    @controlled_movement
    def backward(self,speed):

        motor_orients = (1, -1)
        if self.dir_current_angle != 0:
            self._move_angular(motor_orients)
        else:
            self._move_linear(motor_orients)

    @log_on_start(logging.DEBUG , "Forward movement started")
    @log_on_error(logging.DEBUG , "Forward movement encountered an error")
    @log_on_end(logging.DEBUG , "Forward movement ended")
    @controlled_movement
    def forward(self):
        """Perform a forward movement"""

        motor_orients = (-1, 1)  # Motor orientation
        if self.dir_current_angle != 0:
            self._move_angular(motor_orients)
        else:
            self._move_linear(motor_orients)

    @log_on_start(logging.DEBUG, "Parallel movement started")
    @log_on_error(logging.DEBUG, "Parallel movement encountered an error")
    @log_on_end(logging.DEBUG, "Parallel movement ended")
    def parallel_park(self, angle):
        """Perform parallel parking maneuver for pi car"""

        self.forward(1, angle)
        self.backward(1, -angle)

    @log_on_start(logging.DEBUG, "Three point turn started")
    @log_on_error(logging.DEBUG, "Three point turn encountered an error")
    @log_on_end(logging.DEBUG, "Three point turn ended")
    def three_point_turn(self, angle):
        """Three point turn for pi car"""

        self.forward(1, angle)
        self.backward(1, 10)
        self.forward(1, -angle)

    @log_on_start(logging.DEBUG , "Stop movements started")
    @log_on_error(logging.DEBUG , "Stop movements encountered an error")
    @log_on_end(logging.DEBUG , "Stop movements ended")
    def stop(self):
        self.set_motor_speed(1, 0)
        self.set_motor_speed(2, 0)

    def _move_linear(self, orients):
        """Perform a linear translation"""

        for i, orient in enumerate(orients, start=1):
            self.set_motor_speed(i, orient * 50)
    
    def _move_angular(self, orients):
        """Perform an angular translation"""

        # Saturate the desired angle
        abs_current_angle = abs(self.dir_current_angle)
        if abs_current_angle > 40:
            abs_current_angle = 40

        R_rot = self.CAR_L/math.tan(math.radians(abs_current_angle))  # Radius of rotation

        # Calculate respective motor speeds
        ratio_speed = 2 * SPEED_FACTOR * R_rot + SPEED_FACTOR
        motor_speeds = (SPEED_FACTOR, ratio_speed) if self.dir_current_angle > 0 else (ratio_speed, SPEED_FACTOR)
        motor_speeds = ((speed/SPEED_FACTOR) * 50 for speed in motor_speeds)

        # Set motor speeds
        for i, motor_orient, motor_speed in enumerate(zip(motor_speeds, orients), start=1):
            self.set_motor_speed(i, motor_orient * motor_speed)


# Main loops for the course

def main_loop():
    """Main loop for week 3"""

    px = Picarx()

    sensor = gs.Sensor()
    interpreter = gs.Interpreter()
    controller = gs.Controller()

    while True:
        adc_read = sensor.read_adcs()
        scaled_output = interpreter.interpret(adc_read)
        print(controller.control(px, 2, scaled_output))
        time.sleep(.2)


def main_concurrent():
    """Main loop for week 4"""

    px = Picarx()

    gs_sensor = gs.Sensor()
    gs_interpreter = gs.Interpreter()
    gs_controller = gs.Controller(-30)

    bus_1 = Bus()
    bus_2 = Bus()

    # Create consumers/producers for busses
    bus_1.producer = ConcurrentHelper(lambda : bus_1.write(gs_sensor.read_adcs()), .1)
    bus_1.consumer = ConcurrentHelper(lambda : gs_interpreter.interpret(bus_1.read()), .1)
    bus_2.producer = ConcurrentHelper(lambda : bus_2.write(gs_interpreter.get_scaled_output()), .1)
    bus_2.consumer = ConcurrentHelper(lambda : gs_controller.control(bus_2.read(), px, 2), .1)

    # Run busses
    bus_1.run()
    bus_2.run()
    while True:
        pass


def main_ross_concurrent():
    """Main loop for week 5"""

    px = Picarx()

    # ADC stuff TODO: Rename later for the other classes
    gs_sensor = gs.Sensor()
    gs_interpreter = gs.Interpreter()
    gs_controller = gs.Controller(-30)

    # Create busses

    adc_read_bus = rr.Bus(name="ADC reader")
    adc_interpret_bus = rr.Bus(name="ADC interpreter")
    sonar_read_bus = rr.Bus(name="Sonar reader")
    sonar_interpret_bus = rr.Bus(name="Sonar interpreter")
    control_bus = rr.Bus(name="Car controller bus")
    terminate_bus = rr.Bus(name="Terminate tasks")

    # Create consumer/producers

    read_adc = rr.Producer(
        gs_sensor.read_adcs,
        adc_read_bus,
        0.05,
        terminate_bus,
        "Read greyscale sensor"
    )

    read_sonar = rr.Producer(
        lambda : 0,  # TODO: Replace with actual reader later
        sonar_read_bus,
        0.05,
        terminate_bus,
        "Read sonar sensor"
    )

    interpret_adc = rr.ConsumerProducer(
        gs_interpreter.interpret,
        adc_read_bus,
        adc_interpret_bus,
        .05,
        terminate_bus,
        "Interpret greyscale sensor"
    )

    interpret_sonar = rr.ConsumerProducer(
        lambda : 0,  # TODO: Replace with actual sonar interpretor later
        sonar_read_bus,
        sonar_interpret_bus,
        .05,
        terminate_bus,
        "Interpret Sonar sensor"
    )

    line_control = rr.ConsumerProducer(
        gs_controller.control,
        (interpret_adc, interpret_sonar),
        control_bus,
        .1,
        terminate_bus
    )

    stop_control = rr.ConsumerProducer(

    )

    bus_printer = rr.Printer(
        (read_adc, read_sonar, interpret_adc, interpret_sonar, line_control),  # input data buses
        0.25,
        terminate_bus,
        "Print raw and derived data",
        "Data bus readings are: "
        )
    
    termination_timer = rr.Timer(
        terminate_bus,
        3,
        0.01,
        terminate_bus,
        "Termination timer"
        )

    # Create a list of producer-consumers to execute concurrently
    producer_consumer_list = [
        read_adc,
        read_sonar,
        interpret_adc,
        interpret_sonar,
        line_control,
        stop_control,
        termination_timer,
        bus_printer
        ]

    # Execute the list of producer-consumers concurrently
    rr.runConcurrently(producer_consumer_list)


if __name__ == "__main__":

    px = Picarx()
    # px.forward(1, 1)
    # px.forward(50)
    time.sleep(1)
    px.stop()
