#!/usr/bin/env python

import rospy
import numpy as np
from autoware_msgs.msg import Lane, VehicleCmd
from geometry_msgs.msg import PoseStamped
from shapely.geometry import LineString, Point
from shapely import prepare
from tf.transformations import euler_from_quaternion

class PurePursuitFollower:
    def __init__(self):
        # Parameters
        self.global_path = None
        self.current_pose = None
        self.path_linestring = None  # For shapely LineString
        self.lookahead_distance = rospy.get_param("~lookahead_distance", 5.0)  # Default 5 meters
        self.wheel_base = rospy.get_param("/vehicle/wheel_base", 2.5)  # Default wheelbase in meters

        # Publisher
        self.vehicle_cmd_pub = rospy.Publisher('/control/vehicle_cmd', VehicleCmd, queue_size=1)

        # Subscribers
        rospy.Subscriber('path', Lane, self.path_callback, queue_size=1)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1)

    def path_callback(self, msg):
        # Convert waypoints to a shapely LineString
        if msg.waypoints:
            self.path_linestring = LineString([(w.pose.pose.position.x, w.pose.pose.position.y) for w in msg.waypoints])
            # Prepare the path for more efficient spatial queries
            prepare(self.path_linestring)
            rospy.loginfo("Path prepared with %d waypoints", len(msg.waypoints))
        else:
            rospy.logwarn("Received path with no waypoints")

    def current_pose_callback(self, msg):
        if not self.path_linestring:
            rospy.logwarn("Path not available yet")
            return

        # Convert current pose to a shapely Point
        current_pose = Point([msg.pose.position.x, msg.pose.position.y])

        # Calculate the current heading angle from quaternion
        _, _, current_heading = euler_from_quaternion([msg.pose.orientation.x,
                                                       msg.pose.orientation.y,
                                                       msg.pose.orientation.z,
                                                       msg.pose.orientation.w])

        # Find the distance from the path start to the closest point on the path
        d_ego_from_path_start = self.path_linestring.project(current_pose)

        # Interpolate the lookahead point based on the current position + lookahead distance
        lookahead_point = self.path_linestring.interpolate(d_ego_from_path_start + self.lookahead_distance)

        # Calculate lookahead heading
        lookahead_heading = np.arctan2(lookahead_point.y - current_pose.y, lookahead_point.x - current_pose.x)

        # Calculate the lookahead distance (ld) between current_pose and lookahead_point
        ld = current_pose.distance(lookahead_point)

        # Calculate the heading difference (alpha)
        alpha = lookahead_heading - current_heading

        # Calculate the steering angle using Pure Pursuit formula
        steering_angle = np.arctan2(2 * self.wheel_base * np.sin(alpha), ld)

        # Publish the vehicle command with calculated steering angle
        self.publish_vehicle_cmd(msg.header.stamp, steering_angle)

    def publish_vehicle_cmd(self, timestamp, steering_angle):
        # Create vehicle command message
        vehicle_cmd = VehicleCmd()
        vehicle_cmd.header.stamp = timestamp
        vehicle_cmd.header.frame_id = "base_link"

        # Set the calculated steering angle and constant speed
        vehicle_cmd.ctrl_cmd.steering_angle = steering_angle
        vehicle_cmd.ctrl_cmd.linear_velocity = 10.0  # Constant speed for now

        # Publish the command
        self.vehicle_cmd_pub.publish(vehicle_cmd)
        rospy.loginfo("Published vehicle command - steering_angle: %.2f, speed: %.2f", 
                      vehicle_cmd.ctrl_cmd.steering_angle, 
                      vehicle_cmd.ctrl_cmd.linear_velocity)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pure_pursuit_follower')
    node = PurePursuitFollower()
    node.run()
