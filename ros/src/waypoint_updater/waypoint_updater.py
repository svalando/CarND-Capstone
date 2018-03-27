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
As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.
Once you have created dbw_node, you will update this node to use the status of traffic lights too.
Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number
PREDICT_TIME = 1.0
NEAR_ZERO = 0.00001
MAX_SPEED = 5.0

class WaypointUpdater(object):

    def __init__(self):
        rospy.init_node('waypoint_updater')

        #subscriptions
        rospy.Subscriber(   '/base_waypoints',         Lane, self.waypoints_cb)
        rospy.Subscriber(     '/current_pose',  PoseStamped, self.pose_cb,     queue_size=1)
        rospy.Subscriber( '/traffic_waypoint',        Int32, self.traffic_cb,  queue_size=1)
        rospy.Subscriber('/obstacle_waypoint',         Lane, self.obstacle_cb, queue_size=1)
        rospy.Subscriber( '/current_velocity', TwistStamped, self.current_velocity_cb)

        #Publish
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.debug_waypoint_updater_pub = rospy.Publisher('debug_waypoint_updater', String, queue_size=1)
        # TODO: Add other member variables you need below
        self.lane               = Lane()
        self.first_pose         = True
        self.last_pose_stamp    = rospy.Time(0)
        self.current_linear_x   = 0.0
        self.current_angular_z  = 0.0
        self.traffic_light_wp   = -1
        self.traffic_light_stop = False
        self.traffic_light_det  = False
        self.base_waypoints_2x  = []
        self.break_wp           = 0
        self.breaking           = False
        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        #pass

        # @kai check from here to....
        self.current_pose = msg.pose
       
        delay_d = self.current_linear_x * PREDICT_TIME
        phi     = math.atan2(self.current_pose.position.y, self.current_pose.position.x) + self.current_pose.orientation.z + self.current_angular_z*PREDICT_TIME
        delta_x = delay_d*math.sin(phi)
        delta_y = delay_d*math.cos(phi)
        
        self.predict_pose = Waypoint().pose.pose
        self.predict_pose.position.x = self.current_pose.position.x + delta_x
        self.predict_pose.position.y = self.current_pose.position.y + delta_y
        self.next_wp = self.find_next_wp(self.lane.waypoints, self.predict_pose)
        #self.next_wp = self.find_next_wp(self.lane.waypoints, self.current_pose)
        
        #rospy.loginfo('Current pose --- x is %f, y is %f', self.current_pose.position.x, self.current_pose.position.y)
        #rospy.loginfo('Predict pose --- x is %f, y is %f', self.predict_pose.position.x, self.predict_pose.position.y)
        #map_x = self.lane.waypoints[self.next_wp].pose.pose.position.x
        #map_y = self.lane.waypoints[self.next_wp].pose.pose.position.y
        #rospy.loginfo('Next waypoint[%d] x is %f, y is %f', self.next_wp, map_x, map_y)
        
        # @kai to here. I think we have some issues with the estimated next way point is not where the car is heading always.
        # You can see this that the car sometimes drives infront of the first way point (the green dotted line) or sometime to far behind


        
        # Set default to maximum speed
        self.final_waypoints = []
        next_wp_id = self.next_wp
        for i in range(LOOKAHEAD_WPS):
            p = self.lane.waypoints[next_wp_id]
            self.final_waypoints.append(p)
            self.set_waypoint_velocity(self.final_waypoints, i, MAX_SPEED)
            next_wp_id += 1
            if next_wp_id == len(self.lane.waypoints):
                next_wp_id = 0

        #self.debug_waypoint_updater_pub.publish(str(self.traffic_light_det))
        
        # Check if there is a red light up head
        if self.traffic_light_wp != -1:
            #Get the distance to the traffic light
            dist_to_tl  = self.distance_fwrd(self.lane.waypoints, self.next_wp, self.traffic_light_wp)
            # Get desired stopping distance
            # for smooth stop use 1 m/s^2 deaccelaration
            dist_to_stop = (self.current_linear_x**2)/2.0   # s = (v^2 - V^2)/2*a if a is constant

            # Assert stoping cmd if the stoping distance is less than distance to traffic light            
            if dist_to_stop <= dist_to_tl:
                self.traffic_light_stop = True
            else:
                self.traffic_light_stop = False

            # This should only be run once per detected traffic light
            # It finds the way point to start breaking at
            if not(self.traffic_light_det):
                self.break_wp = self.get_wp_to_start_break(self.lane.waypoints, self.next_wp, self.traffic_light_wp, dist_to_stop)
                #self.debug_waypoint_updater_pub.publish("NXT_WP: " + str(self.next_wp) + " Break WP: "+str(self.break_wp))
                self.traffic_light_det  = True
        else :
            self.traffic_light_stop = False
            self.traffic_light_det  = False
            self.breaking           = False   

        if self.traffic_light_stop :
            next_wp_id = self.next_wp

            if self.breaking :
                delta_v = self.get_break_delta_v(self.current_linear_x, self.traffic_light_wp-1, self.next_wp)
            else :
                delta_v = self.get_break_delta_v(self.current_linear_x, self.traffic_light_wp-1, self.break_wp)
                
            new_v = self.current_linear_x
            for i in range(LOOKAHEAD_WPS):
                new_v -= delta_v 
                if new_v < 0:
                    new_v = 0.0

                if self.breaking :
                    self.set_waypoint_velocity(self.final_waypoints, i, new_v)
                # Use a range here since it seems not exact 
                elif self.break_wp -1 <= next_wp_id <= self.break_wp +1 :  
                    self.breaking = True
                    self.set_waypoint_velocity(self.final_waypoints, i, new_v)

                if i == 0 :
                    self.debug_waypoint_updater_pub.publish("NXT_WP: " + str(self.next_wp) + " Speed: "+str(new_v))

                next_wp_id += 1
                if next_wp_id == len(self.lane.waypoints):
                    next_wp_id = 0
                
       
            
        lane = Lane()
        lane.header.frame_id = '/world'
        #lane.header.stamp = rospy.Time(0)
        lane.header.stamp = rospy.get_rostime()

        lane.waypoints = self.final_waypoints
        self.final_waypoints_pub.publish(lane)

    def get_break_delta_v(self, speed , wp1 ,wp2):
        delta_v = 0.0
        if wp2 >= wp1 :
            num_wp = wp2-wp1
        else:
            num_wp = len(self.lane.waypoints) - wp1 + wp2
        delta_v = speed/num_wp
        return delta_v




    def get_wp_to_start_break(self, waypoints, wp1 ,wp2, dist):
        if wp2 > wp1 :
            num_wp = wp2-wp1
        else:
            num_wp = len(waypoints) - wp1 + wp2
        
        break_wp   = wp2
        next_wp_id = wp1
        for i in range(num_wp -1):
            temp_dist = self.distance_fwrd(waypoints, next_wp_id, wp2)
            if temp_dist <= dist :
                break_wp = next_wp_id
                
            next_wp_id += 1
            if next_wp_id == len(waypoints):
                next_wp_id = 0

        return break_wp

  

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        #pass
        #rospy.loginfo('Base waypoints Received - size is %d', len(waypoints.waypoints))
        self.lane.waypoints = waypoints.waypoints
        self.base_waypoints_2x = copy.deepcopy(self.lane.waypoints)
        #self.base_waypoints_2x = copy.deepcopy(waypoints.waypoints)
        for i in range(0, len(self.lane.waypoints)):
            p = self.base_waypoints_2x[i]
            self.base_waypoints_2x.append(p)
        
        #rospy.loginfo('Base waypoints Received - size is %d', len(self.lane.waypoints))
        self.debug_waypoint_updater_pub.publish("test1")

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_light_wp = int(msg.data)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
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

    def get_duration(self, stamp1, stamp2):
        if stamp1.secs == stamp2.secs:
            duration = (stamp2.nsecs - stamp1.nsecs)/1000000000.0
        else:
            duration = stamp2.secs - stamp1.secs
            duration += (1000000000 - stamp1.nsecs + stamp2.nsecs)/1000000000.0

        return duration

    def current_velocity_cb(self, velocity):
        self.current_linear_x = velocity.twist.linear.x
        self.current_angular_z = velocity.twist.angular.z

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')