#!/usr/bin/env python

import rospy
import numpy as np
from scipy.spatial import KDTree
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # rospy.Subscriber('/vehicle/obstacle', PoseStamped, self.obstacle_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None # To store vehice pose
        self.base_waypoints = None # To store base waypoints
        self.waypoints_2d = None # 2D array of waypoint coordinates
        self.waypoints_tree = None # KD tree of waypoints for faster coordinate lookup

        self.decel_limit = rospy.get_param('/dbw_node/decel_limit', -5)
        self.max_velocity = rospy.get_param('/waypoint_loader/velocity', 40)*0.277778 #km/h -> m/s
        self.max_braking_dist = -(self.max_velocity**2)/(2*self.decel_limit)

        self.loop()

    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose is not None and self.base_waypoints is not None:
                # Publish the next waypoints ahead of the vehicle
                closest_idx = self.get_next_waypoint_idx()
                self.publish_waypoints(closest_idx)
            rate.sleep()

    def get_next_waypoint_idx(self):
        """Finds the next immediate waypoint to the vehicle.

        Returns:
            closest_idx: the next immediate waypoint to the vehicle
        """
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoints_tree.query([x,y], 1)[1]

        # Find the next waypoint using some linear algebra
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]
        
        # Vectorizing components
        closest_vec = np.array(closest_coord)
        prev_vec = np.array(prev_coord)
        pos_vec = np.array([x,y])

        val = np.dot(closest_vec-prev_vec, pos_vec-closest_vec)

        if(val > 0):
            # Vehicle is ahead of the closest_idx, select next waypoint
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    def publish_waypoints(self, closest_idx):
        """Wrapper function which publishes the next waypoints given the index for the waypoint
        which is immediately ahead of the vehicle.

        Arguments:
            closest_idx {integer}: index of waypoint immediately ahead of the vehicle
        """
        hedr = self.base_waypoints.header
        wpts = self.base_waypoints.waypoints[closest_idx:closest_idx+LOOKAHEAD_WPS]
        final_waypoints = Lane()
        final_waypoints.header = hedr
        final_waypoints.waypoints= wpts

        self.final_waypoints_pub.publish(final_waypoints)

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        if self.base_waypoints is None:
            self.base_waypoints = waypoints
            if self.waypoints_2d is None:
                # Create a KD Tree for faster coordinate lookup
                self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
                self.waypoints_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        # If -1, then proceed, else stop (this node is updated from tl_detector)
        traffic_wp = msg.data # int from Int32 message type
        closest_idx = self.get_next_waypoint_idx()
        current_vel = self.get_waypoint_velocity(self.base_waypoints.waypoints[closest_idx])
        # rospy.logwarn('Traffic Light State: {}'.format(traffic_wp))
        if(traffic_wp != -1):
            # v^2 = u^2 + 2as
            # 0 = current_vel^2 + 2*decel_limit*dist
            # max_dist = -current_vel^2/(2*decel_limit)
            # dist = distance(wp1, wp2)
            # if(dist <= max_dist): update waypoints
            dist = self.distance(self.base_waypoints.waypoints, closest_idx, traffic_wp)
            # rospy.logwarn('Current/Light Waypoint: {}/{}'.format(closest_idx, traffic_wp))
            if dist <= self.max_braking_dist:
                # rospy.logwarn('Stopping')
                _current_vel = current_vel
                for i in range(closest_idx, traffic_wp):
                    next_dist = self.distance(self.base_waypoints.waypoints, i, i+1)
                    next_vel = math.sqrt(_current_vel**2) + 2*self.decel_limit*next_dist
                    _current_vel = next_vel
                    self.set_waypoint_velocity(self.base_waypoints.waypoints, i+1, next_vel)
        else:
            if(current_vel <= 0.1):
                # Resume moving from a stop
                self.set_waypoint_velocity(self.base_waypoints.waypoints, closest_idx, self.max_velocity)
            

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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
