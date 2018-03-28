#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import copy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg      import Int32
from std_msgs.msg      import String
         
'''
This node will publish waypoints from the car's current position to some `x` distance ahead.
'''

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish
PREDICT_TIME = 1.0
NEAR_ZERO = 0.00001
STOP_BEFORE_TL = 2.5
ROSPY_HZ_RATE = 20 # Rospy HZ Rate to determine publishing frequency

class WaypointUpdater(object):

    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscriptions
        rospy.Subscriber(   '/base_waypoints',         Lane, self.waypoints_cb)
        rospy.Subscriber(     '/current_pose',  PoseStamped, self.pose_cb,     queue_size=1)
        rospy.Subscriber( '/traffic_waypoint',        Int32, self.traffic_cb,  queue_size=1)
        rospy.Subscriber('/obstacle_waypoint',         Lane, self.obstacle_cb, queue_size=1)
        rospy.Subscriber( '/current_velocity', TwistStamped, self.current_velocity_cb)

        # Publish
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.debug_waypoint_updater_pub = rospy.Publisher('debug_waypoint_updater', String, queue_size=1)

        # Member variables
        self.lane               = Lane()
        self.first_pose         = True
        self.current_pose = None
        self.last_pose_stamp    = rospy.Time(0)
        self.current_linear_x   = 0.0
        self.current_angular_z  = 0.0
        self.traffic_light_wp   = -1
        self.traffic_light_stop = False
        self.traffic_light_det  = False
        self.delta_v_per_m      = 0.0
        self.max_speed          = rospy.get_param('~velocity', 20.0)

        # Loop that keeps publishing at specified HZ rate
        rate = rospy.Rate(ROSPY_HZ_RATE)
        while not rospy.is_shutdown():
            self.publish_final_waypoints()
            rate.sleep()

    def pose_cb(self, msg):
        self.current_pose = msg.pose

    def publish_final_waypoints(self):    
        if self.current_pose is None:
            rospy.logwarn("Waiting for current_pose.. Nothing there yet")
            return           

        delay_d = self.current_linear_x * PREDICT_TIME
        phi     = math.atan2(self.current_pose.position.y, self.current_pose.position.x) + self.current_pose.orientation.z + self.current_angular_z*PREDICT_TIME
        delta_x = delay_d*math.sin(phi)
        delta_y = delay_d*math.cos(phi)
        
        self.predict_pose = Waypoint().pose.pose
        self.predict_pose.position.x = self.current_pose.position.x + delta_x
        self.predict_pose.position.y = self.current_pose.position.y + delta_y
        self.next_wp = self.find_next_wp(self.lane.waypoints, self.predict_pose)

        # Set default to maximum speed
        self.final_waypoints = []
        next_wp_id = self.next_wp
        for i in range(LOOKAHEAD_WPS):
            p = self.lane.waypoints[next_wp_id]
            self.final_waypoints.append(p)
            self.set_waypoint_velocity(self.final_waypoints, i, self.max_speed)
            next_wp_id += 1
            if next_wp_id == len(self.lane.waypoints):
                next_wp_id = 0

        #self.debug_waypoint_updater_pub.publish(str(self.traffic_light_det))
        
        # Check if there is a red light up ahead
        if self.traffic_light_wp != -1:
            #Get the distance to the traffic light
            dist_to_tl  = self.distance_fwrd(self.lane.waypoints, self.next_wp, self.traffic_light_wp)
            
            # Get desired stopping distance
            # for smooth stop use 1 m/s^2 deaccelaration
            dist_to_stop = (self.current_linear_x**2)/2.0   # s = (v^2 - V^2)/2*a if a is constant

            # Assert stoping cmd if the stoping distance is less than distance to traffic light           
            if dist_to_stop >= (dist_to_tl - STOP_BEFORE_TL):
                self.traffic_light_stop = True
            else:
                self.traffic_light_stop = False
            
        else :
            self.traffic_light_stop = False
            self.traffic_light_det  = False
            
        # Start to brake
        if self.traffic_light_stop :
            next_wp_id = self.next_wp

            # Only run this once per Traffic light
            if not(self.traffic_light_det):
                # Calculate deacceration per meter
                if dist_to_stop - STOP_BEFORE_TL >= 0 :
                    self.delta_v_per_m = self.current_linear_x/(dist_to_stop - STOP_BEFORE_TL) 
                else :
                    self.delta_v_per_m = self.current_linear_x/STOP_BEFORE_TL
                self.traffic_light_det  = True

            new_v = self.current_linear_x
            for i in range(LOOKAHEAD_WPS):
                # Get distance to next wp from current
                dist_to_nxt_wp = self.distance_fwrd(self.lane.waypoints, next_wp_id, next_wp_id +1)
                new_v -= self.delta_v_per_m *  dist_to_nxt_wp
                
                if new_v < 0.0:
                    new_v = 0.0
                # Set new speed for wp
                self.set_waypoint_velocity(self.final_waypoints, i, new_v)

                next_wp_id += 1
                if next_wp_id == len(self.lane.waypoints):
                    next_wp_id = 0
 
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.get_rostime()

        lane.waypoints = self.final_waypoints
        self.final_waypoints_pub.publish(lane)


    def waypoints_cb(self, waypoints):
        self.lane.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        self.traffic_light_wp = int(msg.data)

    def obstacle_cb(self, msg):
        # Not yet implemented
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def distance_fwrd(self, waypoints, wp1, wp2):
        dist = 0
        num_wp = 0
        if wp2 >= wp1 :
            num_wp = wp2-wp1
        else:
            num_wp = len(waypoints) - wp1 + wp2
            
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        next_wp_id = wp1 +1
        prev_wp_id = wp1
        for i in range(num_wp):
            dist += dl(waypoints[prev_wp_id].pose.pose.position, waypoints[next_wp_id].pose.pose.position)
            prev_wp_id  = next_wp_id
            next_wp_id += 1
            if next_wp_id == len(waypoints):
                next_wp_id = 0

        return dist


    def find_next_wp(self, waypoints, pose):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        min_dist = dl(waypoints[0].pose.pose.position, pose.position)
        closest_wp_id = 0
        for i in range(1, len(waypoints)):
            dist = dl(waypoints[i].pose.pose.position, pose.position)
            if dist < min_dist:
                min_dist = dist
                closest_wp_id = i
        cur_x = pose.position.x
        cur_y = pose.position.y
        theta = math.atan2(cur_y, cur_x)
        map_x = waypoints[closest_wp_id].pose.pose.position.x
        map_y = waypoints[closest_wp_id].pose.pose.position.y

        heading = math.atan2(map_y - cur_y, map_x - cur_x)
        angle = math.fabs(theta - heading)
        angle = min(2*math.pi - angle, angle)
        if angle > math.pi/4:
            next_wp_id = closest_wp_id + 1
            if next_wp_id == len(waypoints):
                next_wp_id = 0
        else:
            next_wp_id = closest_wp_id

        return next_wp_id

    def current_velocity_cb(self, velocity):
        self.current_linear_x = velocity.twist.linear.x
        self.current_angular_z = velocity.twist.angular.z

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')