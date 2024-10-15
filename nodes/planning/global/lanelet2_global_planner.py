#!/usr/bin/env python

import rospy
import lanelet2
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector
from lanelet2.core import BasicPoint2d
from lanelet2.geometry import findNearest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class Lanelet2GlobalPlanner:
    def __init__(self):
        rospy.init_node('lanelet2_global_planner')
        
        # Load parameters
        self.map_file = rospy.get_param('~lanelet2_map_name')
        self.speed_limit = rospy.get_param('~speed_limit', 40)
        
        # Load map
        self.load_map()
        
        # Setup subscriber to get goal point
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        # Path publisher
        self.path_pub = rospy.Publisher('/planning/global_path', Path, queue_size=1)
        
        rospy.loginfo("Lanelet2GlobalPlanner node initialized.")

    def load_map(self):
        # Loading map
        utm_origin_lat = rospy.get_param('/localization/utm_origin_lat')
        utm_origin_lon = rospy.get_param('/localization/utm_origin_lon')
        use_custom_origin = rospy.get_param('~use_custom_origin', True)
        
        projector = UtmProjector(Origin(utm_origin_lat, utm_origin_lon))
        self.lanelet_map = load(self.map_file, projector)
        
        rospy.loginfo("Lanelet2 map loaded from %s", self.map_file)
    
    def goal_callback(self, msg):
        # Extract goal coordinates
        goal_position = msg.pose.position
        goal_orientation = msg.pose.orientation

        rospy.loginfo("%s - goal position (%f, %f, %f) orientation (%f, %f, %f, %f) in %s frame", 
                      rospy.get_name(), 
                      goal_position.x, goal_position.y, goal_position.z, 
                      goal_orientation.x, goal_orientation.y, goal_orientation.z, goal_orientation.w, 
                      msg.header.frame_id)
        
        # Find the nearest lanelet to the goal point
        goal_point = BasicPoint2d(goal_position.x, goal_position.y)
        nearest_lanelet = findNearest(self.lanelet_map.laneletLayer, goal_point, 1)[0][0]
        
        # Plan the path from the current location to the goal
        self.plan_path(nearest_lanelet)

    def plan_path(self, goal_lanelet):
        # Path planning logic goes here
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"

        # Publish the path
        self.path_pub.publish(path)
        rospy.loginfo("Global path published.")

if __name__ == '__main__':
    try:
        Lanelet2GlobalPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
