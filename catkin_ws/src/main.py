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

WAYPOINT_LIST = [(0,0.5,0),
                 (0,-0.5,0)] # [m], list of wp, offset from the last wp

class HC10Robot():
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "hc10_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.publisher = rospy.Publisher("/move_group/display_planned_path",
                                         moveit_msgs.msg.DisplayTrajectory,
                                         queue_size=20,
                                        )
        rospy.sleep(2)

    def load_cartesian_path(self, waypoint_list : list):
        """Load and define a path between a list of points given as a parameter"""
        waypoints = [] # Waypoints in the correct format
        wpose = self.move_group.get_current_pose().pose
        waypoints.append(copy.deepcopy(wpose)) #For each waypoint, we create a new waypoint from the latest.
        for wp in waypoint_list : 
            wpose.position.x += wp[0]
            wpose.position.y += wp[1]
            wpose.position.z += wp[2]
            waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = self.move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0, avoid_collisions = True)
        return plan, fraction
    
    def display_trajectory(self, plan):
        """Send a message to RVIZ to display the trajectory of the robot"""
        # We build the trajectory message
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)

        #Then we publish it
        self.publisher.publish(display_trajectory)

        rospy.sleep(0.1)

    def add_box_manually(self, box_name_user):
        """Add a simple box to the scene."""
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 1.0 
        box_pose.pose.position.y = 1.0 
        box_pose.pose.position.z = 1.0  # above the hc10 hand frame
        box_name = box_name_user
        self.scene.add_box(box_name, box_pose, size=(0.075, 0.075, 0.075))

    def check_object_added(self, name :str, box_is_attached : bool = False, box_is_known : bool = True):
        """Basic function from the tutorial to run a loop and check if a box was added."""
        start = rospy.get_time()
        seconds = rospy.get_time()
        timeout = 10
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            print(self.scene.get_known_object_names(),self.scene.get_attached_objects([name]))
            if (box_is_attached == is_attached) and (box_is_known == is_known):
               return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        
    def execute_plan(self, plan):
        """Execute plan - non implémentée"""
        while not self.move_group.execute(plan):
            continue

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("HC_10_controler_python", anonymous=True)

robot = HC10Robot()
plan, fraction = robot.load_cartesian_path(WAYPOINT_LIST)
robot.add_box_manually("box")
robot.display_trajectory(plan)
robot.execute_plan(plan)