#!/usr/bin/env python
import rospy
from shapely.geometry import LineString, Point
from shapely import prepare
from autoware_msgs.msg import Lane, VehicleCmd
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np

class PurePursuitFollower:
    def __init__(self):
        rospy.loginfo("Initializing PurePursuitFollower")
        
        # Read parameters
        self.lookahead_distance = rospy.get_param("~lookahead_distance")
        self.wheel_base = rospy.get_param("/vehicle/wheel_base")
        
        # Path variable
        self.path_linestring = None

        # Publisher for vehicle commands
        self.vehicle_cmd_pub = rospy.Publisher('/control/vehicle_cmd', VehicleCmd, queue_size=1)

        # Subscribers
        rospy.Subscriber('path', Lane, self.path_callback, queue_size=1)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1)

    def path_callback(self, msg):
        self.path_linestring = LineString([(w.pose.pose.position.x, w.pose.pose.position.y) for w in msg.waypoints])
        prepare(self.path_linestring)
        rospy.loginfo("Path received with %d waypoints", len(msg.waypoints))

    def current_pose_callback(self, msg):
        current_pose = Point([msg.pose.position.x, msg.pose.position.y])
        
        # Calculate heading angle from quaternion
        _, _, heading = euler_from_quaternion([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])
        
        if self.path_linestring is None:
            rospy.logwarn("Path not set yet. Skipping distance calculation.")
            return
        
        # Calculate the distance from the path start
        d_ego_from_path_start = self.path_linestring.project(current_pose)
        rospy.loginfo("Distance from path start: %.2f meters", d_ego_from_path_start)

        # Find the lookahead point
        lookahead_point = self.path_linestring.interpolate(d_ego_from_path_start + self.lookahead_distance)
        
        # Calculate lookahead heading
        lookahead_heading = np.arctan2(lookahead_point.y - current_pose.y, lookahead_point.x - current_pose.x)

        # Recalculate the lookahead distance
        ld = current_pose.distance(lookahead_point)

        # Calculate the difference in heading
        alpha = heading - lookahead_heading

        # Normalize alpha to range [-pi, pi]
        if alpha > np.pi:
            alpha -= 2 * np.pi
        elif alpha < -np.pi:
            alpha += 2 * np.pi

        # Calculate steering angle using the formula
        steering_angle = np.arctan((2 * self.wheel_base * np.sin(alpha)) / ld)

        # Obey max steering angle bounds
        steering_angle = max(min(steering_angle, 1.22), -1.22)

        # Publish vehicle command
        vehicle_cmd = VehicleCmd()
        vehicle_cmd.ctrl_cmd.steering_angle = steering_angle
        vehicle_cmd.ctrl_cmd.linear_velocity = 10.0

        # Set header information
        vehicle_cmd.header.stamp = msg.header.stamp
        vehicle_cmd.header.frame_id = "base_link"

        self.vehicle_cmd_pub.publish(vehicle_cmd)
        rospy.loginfo("Published vehicle command: speed = %.2f, steering angle = %.2f",
                      vehicle_cmd.ctrl_cmd.linear_velocity, vehicle_cmd.ctrl_cmd.steering_angle)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz for publishing vehicle commands
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('pure_pursuit_follower')
    node = PurePursuitFollower()
    node.run()

