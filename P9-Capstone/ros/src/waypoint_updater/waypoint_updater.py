#!/usr/bin/env python

# from types import ClassMethodDescriptorType
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree

import math
import numpy as np

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

LOOKAHEAD_WPS = 50 #200 - car doesn't stay in lane  # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher(
            'final_waypoints', Lane, queue_size=1)

        self.pose = None
        self.base_lane = None
        self.waypoints_2d = None
        self.waypoints_tree = None
        self.stopline_wp_idx = -1
        

        self.loop()

    def loop(self):
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            # if members are initialised
            if self.pose and self.waypoints_2d:
                # Compute and publish the closest waypoint
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints(closest_waypoint_idx)
            # Sleep until rate achieved
            rate.sleep()

    def get_closest_waypoint_idx(self):
        # For the current car pose
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        # Use the KD-tree to get the closest waypoint index
        # Only want the single closest waypoint
        # query() returns tuple as (distance, index)
        closest_idx = self.waypoints_tree.query([x, y], 1)[1]

        # Check if returned waypoint is in front of car
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        # Get the dot product of vectors:
        # previous -> closest, closest -> pose
        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        # If dp is positive, both vectors point in same direction.
        # Therfore, waypoint is behind so get the next waypoint
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    def publish_waypoints(self, closest_idx):
        # lane = Lane()
        # Waypoints to publish are the next LOOKAHEAD_WPS from the current waypoint
        # lane.waypoints = self.base_waypoints.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)

    # Update waypoint velocities based on desired behaviour
    def generate_lane(self):
        lane = Lane()

        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_lane_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]

        # If traffic light is not red or too far, keep waypoints as is
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_lane_waypoints
        else:
            # otherwise, reduce speed
            lane.waypoints = self.decelerate_waypoints(base_lane_waypoints, closest_idx)

        return lane

    # Update a sliced waypoints list to decelerate the car
    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        # Loop over sliced base waypoints creating new waypoints
        # with modified velocities
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose

            # Stopping position for car
            # 2 waypoints back so front of car is behind stop line
            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
            # Calculate piecewise distance between current waypoint and stop line
            dist = self.distance(waypoints, i, stop_idx)
            # Create a curved deceleration profile based on distance to stop line
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.0:
                vel = 0.

            # Update waypoint velocity keeping within speed limit
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)

        return temp


    def pose_cb(self, msg):
        # Store the car's pose
        self.pose = msg
        pass

    def waypoints_cb(self, waypoints):
        self.base_lane = waypoints

        if not self.waypoints_2d:
            # Convert each waypoint to 2D coordinates
            self.waypoints_2d = [[waypoint.pose.pose.position.x,
                                  waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            # Use KDTree to look up closest point in space efficiently (log n)
            self.waypoints_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.stopline_wp_idx = msg.data

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
