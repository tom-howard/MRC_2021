#!/usr/bin/env python

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import argparse
import os
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
try:
  from math import pi, tau, dist, fabs, cos
except: # For Python 2 compatibility
  from math import pi, fabs, cos, sqrt
  tau = 2.0*pi
  def dist(p, q):
    return sqrt(sum((p_i - q_i)**2.0 for p_i, q_i in zip(p,q)))
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list



def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if the values in two lists are within a tolerance of each other.
  For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
  between the identical orientations q and -q is calculated correctly).
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
    x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
    # Euclidean distance
    d = dist((x1, y1, z1), (x0, y0, z0))
    # phi = angle between orientations
    cos_phi_half = fabs(qx0*qx1 + qy0*qy1 + qz0*qz1 + qw0*qw1)
    return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

  return True

## Set the ROS namespace to be used in this script.
##NOTE A more elegant way is to set up 'ns' in the .launch file, but this will have to do 
ns = '/iiwa'
os.environ["ROS_NAMESPACE"] = ns

## First initialize `moveit_commander`_ and a `rospy`_ node:
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

## Instantiate a `RobotCommander`_ object. Provides information such as the robot's kinematic model and the robot's current joint states
robot = moveit_commander.RobotCommander()

## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface for getting, setting, and updating the robot's internal understanding of the
## surrounding world:
# scene = moveit_commander.PlanningSceneInterface()

## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
## to a planning group (group of joints).  In this tutorial the group is the primary
## arm joints in the KUKA iiwa robot, so we set the group's name to "manipulator".
## This interface can be used to plan and execute motions:
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

## Create a `DisplayTrajectory`_ ROS publisher which is used to display
## trajectories in Rviz:
display_trajectory_publisher = rospy.Publisher(ns + '/move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)


target_tolerance = 0.01
## Some joint states are named for convenience
joints_start = [0] * 7  # array of 7 zeroes
joints_small_shift = [0.3] * 7
## If iiwa were to get in a position to pick up an object from a table, 
## it might look something like the following pose 
pose_ready_to_pick_up = geometry_msgs.msg.Pose()
q = quaternion_from_euler(3.14, 0, 0)
pose_ready_to_pick_up.orientation.x = q[0]
pose_ready_to_pick_up.orientation.y = q[1]
pose_ready_to_pick_up.orientation.z = q[2]
pose_ready_to_pick_up.orientation.w = q[3]
pose_ready_to_pick_up.position.x = 0.4
pose_ready_to_pick_up.position.y = -0.4
pose_ready_to_pick_up.position.z = 0.7


def go_to_joint_state(joint_goal):
  """
  The functions takes joint_goal as input, which is is an array of joint positions expressed in radians.
  In case of KUKA iiwa, there are 7 joints.    
  """
  print("Calling 'Go to joint state' with the following joint goal: {}".format(joint_goal))
  # The go command can be called with joint values, poses, or without any
  # parameters if you have already set the pose or joint target for the group
  # wait=True means the call is blocking
  move_group.go(joint_goal, wait=True)

  # Calling ``stop()`` ensures that there is no residual movement
  move_group.stop()

  # Check if arrived at destination
  current_joints = move_group.get_current_joint_values()
  if all_close(joint_goal, current_joints, target_tolerance):
    print("Target state reached")
  else:
    print("Target state NOT reached")

def go_to_pose_goal(pose):
  print("Calling 'Go to pose' with the following goal: \n{}".format(pose))
  move_group.set_pose_target(pose)

  ## Now, we call the planner to compute the plan and execute it.
  plan = move_group.go(wait=True)

  # It is always good to clear your targets after planning with poses.
  # Note: there is no equivalent function for clear_joint_value_targets()
  move_group.clear_pose_targets()

  # Calling ``stop()`` ensures that there is no residual movement
  move_group.stop()
  
  # Check if arrived at destination
  current_pose = move_group.get_current_pose().pose
  if all_close(pose, current_pose, target_tolerance):
    print("Target pose reached")
  else:
    print("Target pose NOT reached")

def plan_cartesian_path(scale=1):

  ## You can plan a Cartesian path directly by specifying a list of waypoints
  ## for the end-effector to go through. If executing interactively in a
  ## Python shell, set scale = 1.0.
  ##
  waypoints = []

  wpose = move_group.get_current_pose().pose
  wpose.position.z -= scale * 0.1  # First move up (z)
  wpose.position.y += scale * 0.2  # and sideways (y)
  waypoints.append(copy.deepcopy(wpose))

  wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
  waypoints.append(copy.deepcopy(wpose))

  wpose.position.y -= scale * 0.1  # Third move sideways (y)
  waypoints.append(copy.deepcopy(wpose))

  # We want the Cartesian path to be interpolated at a resolution of 1 cm
  # which is why we will specify 0.01 as the eef_step in Cartesian
  # translation.  We will disable the jump threshold by setting it to 0.0,
  # ignoring the check for infeasible jumps in joint space, which is sufficient
  # for this tutorial.
  (plan, fraction) = move_group.compute_cartesian_path(
                                      waypoints,   # waypoints to follow
                                      0.01,        # eef_step
                                      0.0)         # jump_threshold

  # Note: We are just planning, not asking move_group to actually move the robot yet:
  return plan, fraction

  ## END_SUB_TUTORIAL


def display_trajectory(plan):
  ## BEGIN_SUB_TUTORIAL display_trajectory
  ##
  ## Displaying a Trajectory
  ## ^^^^^^^^^^^^^^^^^^^^^^^
  ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
  ## group.plan() method does this automatically so this is not that useful
  ## here (it just displays the same trajectory again):
  ##
  ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
  ## We populate the trajectory_start with our current robot state to copy over
  ## any AttachedCollisionObjects and add our plan to the trajectory.
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()
  display_trajectory.trajectory_start = robot.get_current_state()
  display_trajectory.trajectory.append(plan)
  # Publish
  display_trajectory_publisher.publish(display_trajectory)

  ## END_SUB_TUTORIAL


def main():
  try:
    ## Task 0: ensure the arm is at starting position
    if not all_close(joints_start, move_group.get_current_joint_values(), target_tolerance):
      print("Moving to starting position")
      go_to_joint_state(joints_start)

    ## Task 1: Actuate all the joints a bit to go to a slightly different position 
    go_to_joint_state(joints_small_shift)
    rospy.sleep(1.)
    print("Moving back to starting position")
    go_to_joint_state(joints_start)
 
    ## Task 2: Go to pose using planner 
    go_to_pose_goal(pose_ready_to_pick_up)
    rospy.sleep(1.)

    ## Task 3: Plan a trajectory in Cartesian coordinates and execute
    # Using the previous pose as a starting position,
    # do some movements that might look like we are trying to move something on the table  
    cartesian_plan, fraction = plan_cartesian_path()
    display_trajectory(cartesian_plan)
    move_group.execute(cartesian_plan, wait=True)
    print("Moving back to starting position")
    go_to_joint_state(joints_start)


  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
