#!/usr/bin/env python3

from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, dist, fabs, cos
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
try:
  from math import tau
except: # For Python 2 compatibility
  from math import pi
  tau = 2.0*pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
    x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
    d = dist((x1, y1, z1), (x0, y0, z0))
    cos_phi_half = fabs(qx0*qx1 + qy0*qy1 + qz0*qz1 + qw0*qw1)
    return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

 


class MoveGroup(object):
  def __init__(self):
    super(MoveGroup, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    self.robot = robot
    self.move_group = move_group
   
  def go_to_pose_goal(self):
    move_group = self.move_group
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = 0.7
    pose_goal.position.y = 0.0
    pose_goal.position.z = 0.4
    
    
    quaternion = quaternion_from_euler(0,0,0)
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]
    
    move_group.set_pose_target(pose_goal)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    roll = pitch = yaw = 0.0
    orientation = [quaternion[0], quaternion[1], quaternion[2], quaternion[3] ]
    #(roll, pitch, yaw) = euler_from_quaternion(orientation)
    #print(roll, pitch, yaw)
    current_pose = self.move_group.get_current_pose().pose
    print(current_pose)
    return all_close(pose_goal, current_pose, 0.01)

def main():
  try:
    print("")
    input("============ Press `Enter` to begin the tutorial by setting up the moveit_commander ...")
    tutorial = MoveGroup()

    input("============ Press `Enter` to execute a movement using a pose goal ...")
    tutorial.go_to_pose_goal()

    print("End")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
