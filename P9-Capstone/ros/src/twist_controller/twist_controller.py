import rospy

from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        # Yaw controller for steering
        self.yaw_controller = YawController(kwargs['wheel_base'],
                                            kwargs['steer_ratio'],
                                            0.1,
                                            kwargs['max_lat_accel'],
                                            kwargs['max_steer_angle'])

        # PID controller for throttle
        kp = 0.3
        ki = 0.1
        kd = 0.
        mn = 0.   # Minimum throttle value
        mx = 0.2  # Maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        # Low pass filter for incoming noisy velocity
        tau = 0.5  # 1 / (2*pi*tau) = cutoff frequency
        ts = 0.02  # Sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = kwargs['vehicle_mass']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.brake_deadband = kwargs['brake_deadband']
        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']
        self.wheel_radius = kwargs['wheel_radius']

        self.last_time = rospy.get_time()

    # Called by the main controller in dbw_node.py
    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        target_linear_vel = kwargs['linear_velocity']
        angular_vel = kwargs['angular_velocity']
        current_vel = kwargs['current_vel']

        # Resets the integral value if not in DBW mode
        if not kwargs['dbw_enabled']:
            self.throttle_controller.reset()
            return 0., 0., 0.

        # Get the current velocity filtered using Low Pass filter
        current_vel = self.vel_lpf.filt(current_vel)

        # Get the steering
        steering = self.yaw_controller.get_steering(
            target_linear_vel, angular_vel, current_vel)

        # Difference between desired and current velocity
        vel_error = target_linear_vel - current_vel
        self.last_vel = current_vel

        # Sample time for PID control step
        current_time = rospy.get_time()
        sampling_time = current_time - self.last_time
        self.last_time = current_time

        # Use PID control to get the next throttle output
        throttle = self.throttle_controller.step(vel_error, sampling_time)

        # Apply simple control logic for braking
        brake = 0
        # If target velocity = 0, let go of throttle and apply braking
        if target_linear_vel == 0. and current_vel < 0.1:
            throttle = 0
            brake = 700  # Torque: Nm - hold the car stationary when stopped

        # If error is negative and not applying throttle, braking required
        elif throttle < 0.1 and vel_error < 0:
            throttle = 0
            # Deceleration required
            decel = max(vel_error, self.decel_limit)
            # Corresponding braking required
            # Use of abs as decel is negative
            brake = abs(decel) * self.vehicle_mass * \
                self.wheel_radius  # Torque: Nm

        # return 1., 0., 0.
        return throttle, brake, steering
