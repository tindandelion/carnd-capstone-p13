#!/usr/bin/env python

import rospy
import math
import json
import os

import numpy as np

from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree

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

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number


def dump_to_json(waypoints):
    out = open("./waypoints.json", "w")
    out.write(str(waypoints))
    rospy.logerr("Waypoints dumped to: " + os.getcwd())


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher(
            'final_waypoints', Lane, queue_size=1)

        self.current_pose = None
        self.waypoints_2d = None
        self.waypoints_tree = None
        self.base_waypoints = None

        self.update_loop()

    def update_loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.current_pose and self.waypoints_tree:
                closest_wp_index = self.closest_waypoint_index()
                self.publish_waypoints(closest_wp_index)
            rate.sleep()

    def publish_waypoints(self, closest_idx):
        lane = Lane()
        lane.waypoints = self.base_waypoints.waypoints[closest_idx:closest_idx+LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)

    def closest_waypoint_index(self):
        pos = (self.current_pose.position.x, self.current_pose.position.y)
        _, idx = self.waypoints_tree.query(pos)
        closest_waypoint = self.waypoints_2d[idx]
        prev_waypoint = self.waypoints_2d[idx-1]
        path_vec = np.array(closest_waypoint) - np.array(prev_waypoint)
        traj_vec = np.array(closest_waypoint) - np.array(pos)
        projection = np.dot(traj_vec, path_vec)

        if projection < 0:
            idx = (idx + 1) % len(self.waypoints_2d)

        return idx

    def pose_cb(self, msg):
        self.current_pose = msg.pose

    def waypoints_cb(self, waypoints):
        if self.waypoints_2d:
            return

        self.base_waypoints = waypoints
        self.waypoints_2d = [(wp.pose.pose.position.x, wp.pose.pose.position.y)
                             for wp in waypoints.waypoints]
        self.waypoints_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0

        def dl(a, b): return math.sqrt(
            (a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position,
                       waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
