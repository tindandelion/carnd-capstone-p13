import rospy
import os 

from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter


GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MIN_SPEED = 0


class BrakeController:
    def __init__(self, decel_limit, vehicle_mass, wheel_radius):
        self.decel_limit = decel_limit
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius

    def get_brake(self, velocity_error):
        decel = max(velocity_error, self.decel_limit)
        return abs(decel) * self.vehicle_mass * self.wheel_radius


class Controller(object):
    def __init__(self, wheel_base, steer_ratio,
                 max_lat_accel, max_steer_angle, decel_limit, 
                 vehicle_mass, wheel_radius):
        self.brake_controller = BrakeController(decel_limit, vehicle_mass, wheel_radius)        
        self.yaw_controller = YawController(wheel_base, steer_ratio, MIN_SPEED, max_lat_accel, max_steer_angle)
        self.vel_filter = LowPassFilter(tau=0.5, ts=0.02)
        self.throttle_pid = PID(kp=0.3, ki=0.02, kd=0.1, mn=0.0, mx=1.0)

        self.last_time = rospy.get_time()
        self.is_enabled = False
        self.velocities = []

    def control(self, lin_velocity, angular_velocity, cur_velocity):
        if not self.is_enabled:
            self.throttle_pid.reset()
            return 0, 0, 0

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        cur_velocity = self.vel_filter.filt(cur_velocity)
        velocity_error = lin_velocity - cur_velocity
        steer = self.yaw_controller.get_steering(lin_velocity, angular_velocity, cur_velocity)
        throttle = self.throttle_pid.step(velocity_error, sample_time)
        self.velocities.append([lin_velocity, cur_velocity])

        brake = 0.0
        # if lin_velocity == 0.0 and cur_velocity < 0.1:
        #     throttle = 0
        #     brake = 400
        # elif throttle < 0.1 and velocity_error < 0:
        #     throttle = 0
        #     brake = self.brake_controller.get_brake(velocity_error)

        return throttle, brake, steer
    def dump(self, dump_dir):
        dump_file = os.path.join(dump_dir, 'velocities.csv')
        with open(dump_file, 'w') as f:
            for lin, cur in self.velocities:
                f.write("%f;%f\n" % (lin, cur))
