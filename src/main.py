# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

WAYPOINT_LIST = [(0.0,0.1,-0.5),
                 (0.6,-0.1,0),
                 (-0.6,0,0.5)] # [m], list of wp, offset from the last wp
SCALE = 1

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("HC_10_controler_python", anonymous=True)

display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

class HC10Robot():
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "hc10_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

    def load_cartesian_path(self,scale : float,waypoint_list : list):
        waypoints = []
        wpose = self.move_group.get_current_pose().pose
        waypoints.append(copy.deepcopy(wpose))
        for wp in waypoint_list : 
            wpose.position.x += wp[0]
            wpose.position.y += wp[1]
            wpose.position.z += wp[2]
            waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = self.move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0  )
        return plan, fraction
    
    def display_trajectory(self, plan, publisher):
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        publisher.publish(display_trajectory)

    def __refresh_scene_object():
        #TODO
        print(" ! Non implémenté ! ")

robot = HC10Robot()
plan, fraction = robot.load_cartesian_path(SCALE,WAYPOINT_LIST)
robot.display_trajectory(plan,display_trajectory_publisher)