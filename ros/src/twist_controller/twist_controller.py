import rospy

from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MIN_SPEED = 0

class Controller(object):
    def __init__(self, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        self.yaw_controller = YawController(wheel_base, steer_ratio, MIN_SPEED, max_lat_accel, max_steer_angle)
        self.vel_filter = LowPassFilter(tau=0.5, ts=0.02)
        self.throttle_pid = PID(kp=1.0, ki=0.1, kd=1.0, mn=0.0, mx=1.0)
        self.last_time = rospy.get_time()

        self.is_enabled = False

    def control(self, lin_velocity, angular_velocity, cur_velocity):
        if not self.is_enabled: 
            self.throttle_pid.reset()
            return 0, 0, 0

        brake = 0.0

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        cur_velocity = self.vel_filter.filt(cur_velocity)
        steer = self.yaw_controller.get_steering(lin_velocity, angular_velocity, cur_velocity)
        throttle = self.throttle_pid.step(lin_velocity - cur_velocity, sample_time)
        rospy.logwarn("Current velocity delta: %f", lin_velocity - cur_velocity)
        
        return throttle, brake, steer
