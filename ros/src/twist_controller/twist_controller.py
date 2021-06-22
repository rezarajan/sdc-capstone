
from pid import PID


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement

        # Config Variables
        ## PID
        kp = 0
        ki = 0
        kd = 0
        mn = 0 # min throttle
        mx = 0.2 # max throttle
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        ## Low Pass
        tau = 0.5 # cutoff freq. 1/(2*pi*tau)
        ts = 0.02 # 50Hz refresh rate

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
            return 0., 0., 0. # 0 for compatibility with simulator
        # Return throttle, brake, steer
        return 1., 0., 0.
