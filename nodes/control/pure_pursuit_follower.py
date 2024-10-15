#!/usr/bin/env python

import rospy
import numpy as np
from autoware_msgs.msg import Lane, VehicleCmd
from geometry_msgs.msg import PoseStamped
from shapely.geometry import LineString, Point
from shapely import prepare
from tf.transformations import euler_from_quaternion
from scipy.interpolate import interp1d

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

        self.distance_to_velocity_interpolator = None  # Class variable for interpolator

    def path_callback(self, msg):
        # Collect waypoint coordinates
        waypoints_xy = np.array([(w.pose.pose.position.x, w.pose.pose.position.y) for w in msg.waypoints])

        # Calculate cumulative distances between waypoints
        distances = np.cumsum(np.sqrt(np.sum(np.diff(waypoints_xy, axis=0)**2, axis=1)))
        distances = np.insert(distances, 0, 0)  # Adding 0m for the first waypoint

        # Extract velocity values from waypoints
        velocities = np.array([w.twist.twist.linear.x for w in msg.waypoints])

        # Create an interpolator for distance to velocity
        self.distance_to_velocity_interpolator = interp1d(
            distances, velocities, kind='linear', bounds_error=False, fill_value=0.0
        )

        # Prepare path as shapely LineString
        self.path_linestring = LineString(waypoints_xy)
        prepare(self.path_linestring)
        rospy.loginfo("Path prepared with %d waypoints", len(msg.waypoints))

    def current_pose_callback(self, msg):
        if not self.path_linestring:
            rospy.logwarn("Path not available yet")
            return

        if self.distance_to_velocity_interpolator is None:
            rospy.logwarn("Velocity interpolator not ready yet")
            return

        # Convert current pose to shapely Point
        current_pose = Point([msg.pose.position.x, msg.pose.position.y])

        # Calculate distance from path start
        d_ego_from_path_start = self.path_linestring.project(current_pose)

        # Calculate the vehicle's current speed based on its position
        current_speed = self.distance_to_velocity_interpolator(d_ego_from_path_start)

        # Calculate the current heading angle from quaternion
        _, _, current_heading = euler_from_quaternion([msg.pose.orientation.x,
                                                       msg.pose.orientation.y,
                                                       msg.pose.orientation.z,
                                                       msg.pose.orientation.w])

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

        # Now include the current speed in the vehicle command
        self.publish_vehicle_cmd(msg.header.stamp, steering_angle, current_speed)

    def publish_vehicle_cmd(self, timestamp, steering_angle, speed):
        # Create vehicle command message
        vehicle_cmd = VehicleCmd()
        vehicle_cmd.header.stamp = timestamp
        vehicle_cmd.header.frame_id = "base_link"

        # Set the calculated steering angle and the interpolated speed
        vehicle_cmd.ctrl_cmd.steering_angle = steering_angle
        vehicle_cmd.ctrl_cmd.linear_velocity = speed

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