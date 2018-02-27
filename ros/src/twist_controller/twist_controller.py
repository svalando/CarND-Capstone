
from pid import PID
from yaw_controller import YawController
import math
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self,
                 vehicle_mass,
                 fuel_capacity,
                 brake_deadband,
                 decel_limit,
                 accel_limit,
                 wheel_radius,
                 wheel_base,
                 steer_ratio,
                 max_lat_accel,
                 max_steer_angle):
        # TODO: Implement
 
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle 

        self.throttle_controller = PID(1.0, 0.0, -1.23978)
        self.steer_controller = YawController(self.wheel_base,
                                              self.steer_ratio,
                                              0.001,
                                              self.max_lat_accel,
                                              self.max_steer_angle)

    def control(self, 
                cmd_linear_x,
                cmd_angular_z,
                current_linear_x,
                current_angular_z,
                sample_internal):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        #speed_error = current_linear_x - cmd_linear_x
        speed_error = cmd_linear_x - current_linear_x

        rospy.loginfo('speed_error is %f', speed_error)

        throttle = self.throttle_controller.step(speed_error, sample_internal)
        rospy.loginfo('pid result is %f', throttle)

        steer = self.steer_controller.get_steering(cmd_linear_x, cmd_angular_z, current_linear_x)

        if throttle >= 0.0:
            brake = 0.0
            if throttle > self.accel_limit:
                throttle = self.accel_limit
        else:
            throttle = 0.0
            #brake = math.fabs(speed_error/sample_internal)*self.vehicle_mass*self.wheel_radius
            brake = 0.0

        return 0.01, brake, steer
