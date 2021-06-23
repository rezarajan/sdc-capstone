
import rospy
from pid import PID
from lowpass import LowPassFilter


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_weight, wheel_radius, decel_limit):
        # TODO: Implement
        self.last_time = None

        # Config Variables
        ## PID
        kp = 0.3
        ki = 0.1
        kd = 0.0
        mn = 0 # min throttle
        mx = 0.2 # max throttle
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        ## Low Pass
        tau = 0.5 # cutoff freq. 1/(2*pi*tau)
        ts = 0.02 # 50Hz refresh rate
        self.lowpass = LowPassFilter(tau, ts)

        ## Vehicle Params
        self.vehicle_weight = vehicle_weight
        self.wheel_radius = wheel_radius
        self.decel_limit = decel_limit

    def control(self, linear_vel, angular_vel, current_lin_vel, current_ang_vel, dbw_enabled):
        '''
        Arguments:
            linear_vel: target linear velocity
            angular_vel: target angular velocity
            current_lin_vel: current linear velocity
            current_ang_vel: current angular velocity
            dbw_enabled: dbw status
        Returns:
            tuple: (throttle, brake, steering)
        '''
        # TODO: Change the arg, kwarg list to suit your needs
        # TODO: Implement PID Controller(throttle, braking) and Yaw Controller (Steering)
        if not dbw_enabled:
            self.throttle_controller.reset()
            self.last_time = None
            return 0., 0., 0. # 0 for compatibility with simulator

        # Filter velocity to remove noise
        lowpass_vel = self.lowpass.filt(current_lin_vel)

        vel_err = linear_vel - lowpass_vel
        dt = rospy.get_time() 
        if self.last_time is None:
            self.last_time = dt
        else:
            dt -= self.last_time

        throttle = self.throttle_controller.step(vel_err, dt)
        # rospy.logwarn('vel_err: {}'.format(vel_err))
        # rospy.logwarn('dt: {}'.format(dt))
        # rospy.logwarn('Throttle: {}'.format(throttle))
        # rospy.logwarn('Target/Current Velocity: {}/{}'.format(linear_vel, current_lin_vel))

        # Braking Logic
        brake = 0 # default no braking
        # When vehicle is stopped, apply brakes to prevent rolling,
        # or if above target velocity, brake to decelerate to target
        if linear_vel == 0 and current_lin_vel < 0.1:
            throttle = 0
            brake = 700 # Nm
        elif vel_err < 0:
            throttle = 0
            decel = max(vel_err, self.decel_limit)
            brake = self.vehicle_weight*self.wheel_radius*abs(decel) # Torque

        # Return throttle, brake, steer
        return throttle, brake, 0.
