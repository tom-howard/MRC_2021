#!/usr/bin/env python
"""
Move Group functionality demo using a KUKA LBR iiwa simulated model. 
"""
# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

# Standard Library
import os
import sys
import copy

# ROS and MoveIt!
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

# Helper functions
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
try:
  from math import pi, tau, dist, fabs, cos
except: # For Python 2 compatibility
  from math import pi, fabs, cos, sqrt
  tau = 2.0*pi
  def dist(p, q):
    return sqrt(sum((p_i - q_i)**2.0 for p_i, q_i in zip(p,q)))

# Set up a ROS namespace to be used in this script.
##NOTE A more elegant way is to set up a dedicated .launch file and use a 'ns' arg,
## but this will have to do 
ns = '/iiwa'
os.environ["ROS_NAMESPACE"] = ns

# First initialise `moveit_commander`_ and a `rospy`_ node:
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

# Instantiate a `RobotCommander`_ object.
# Provides information such as the robot's kinematic model and the robot's current joint states
robot = moveit_commander.RobotCommander()

# Instantiate a `PlanningSceneInterface`_ object.
# This provides a remote interface for getting, setting, and updating the 
# robot's internal understanding of the surrounding world:
scene = moveit_commander.PlanningSceneInterface()

# Instantiate a `MoveGroupCommander`_ object.  This object is an interface
# to a planning group (group of joints).  In this tutorial the group is the primary
# arm joints in the KUKA LBR iiwa robot, so we set the group's name to "manipulator".
# This interface can be used to plan and execute motions:
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

# Relax the goal tolerances a bit when executing a plan
move_group.set_goal_joint_tolerance(0.001) # default 0.0001
move_group.set_goal_position_tolerance(0.001) # default 0.0001
move_group.set_goal_orientation_tolerance(0.01) # default 0.001

# Create a `DisplayTrajectory`_ ROS publisher which is used to display
# trajectories in Rviz:
display_trajectory_publisher = rospy.Publisher(
        ns + '/move_group/display_planned_path',
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20
)


target_tolerance = 0.02
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


def go_to_joint_state(joint_goal):
  """
  Takes joint goal as input, which is is an array (vector) of joint angles expressed in radians.
  The 'go' command is executed immediately, there is no separate planning stage. 
  """
  print("Calling 'Go to joint state' with the following goal: {}".format(joint_goal))
  # The 'go' command can be called with joint values, poses, or without any
  # parameters if you have already set the pose or joint target for the group.
  ##NOTE: wait=True means the call is blocking.
  move_group.go(joint_goal, wait=True)

  # Calling ``stop()`` ensures that there is no residual movement
  move_group.stop()

  # Double check if arrived at destination
  current_joints = move_group.get_current_joint_values()
  if all_close(joint_goal, current_joints, target_tolerance):
    print("Target joint state reached")
  else:
    print("Warning, target joint state NOT reached!")
    print(current_joints)


def go_to_pose_goal(pose_goal):
  """
  Takes a pose goal as input, and sets it as a target for the planner.
  After the resulting plan is executed, the target pose is cleared. 
  """
  print("Calling 'Go to pose' with the following goal: \n{}".format(pose_goal))
  # A pose target is set in the world for the end-effector
  move_group.set_pose_target(pose_goal)

  # Now, we call the planner to compute the plan and execute it.
  plan = move_group.go(wait=True)

  # It is always good to clear your targets after planning with poses.
  ##NOTE: there is no equivalent function for clear_joint_value_targets()
  move_group.clear_pose_targets()

  # Calling ``stop()`` ensures that there is no residual movement
  move_group.stop()
  
  # Check if arrived at destination
  current_pose = move_group.get_current_pose().pose
  if all_close(pose_goal, current_pose, target_tolerance):
    print("Target pose reached")
  else:
    print("Warning, target pose NOT reached!")
    print(current_pose)


def plan_cartesian_path(waypoints):
  """
  Compute a sequence of waypoints that make the end-effector move in straight 
  line segments that follow the poses specified as waypoints. Configurations are 
  computed for every eef_step meters; The jump_threshold specifies the maximum 
  distance in configuration space between consecutive points in the resulting path. 
  """
  # We want the Cartesian path to be interpolated at a resolution of 1 cm
  # which is why we will specify 0.01 as the eef_step in Cartesian
  # translation.  We will disable the jump threshold by setting it to 0.0,
  # ignoring the check for infeasible jumps in joint space, which is sufficient
  # for this tutorial.
  (plan, fraction) = move_group.compute_cartesian_path(
                                      waypoints,   # waypoints to follow
                                      0.01,        # eef_step
                                      0.0)         # jump_threshold

  # The return value is a tuple: the actual RobotTrajectory and 
  # a fraction of how much of the path was followed
  ##NOTE: We are just planning, not asking move_group to actually move the robot yet
  return plan, fraction


def display_trajectory(plan):
  """
  RViz can visualize a plan (aka trajectory) for you. But the
  group.plan() method does this automatically so this is not that useful
  here (it just displays the same trajectory again):
  """
  # A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
  # We populate the trajectory_start with our current robot state to copy over
  # any AttachedCollisionObjects and add our plan to the trajectory.
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()
  display_trajectory.trajectory_start = robot.get_current_state()
  display_trajectory.trajectory.append(plan)
  # Publish
  display_trajectory_publisher.publish(display_trajectory)



def main():
  try:
    # Define some named joint states for convenience
    joints_start = [0.0] * 7  # Array of 7 zeroes, coincides with the starting position
    joints_small_shift = [0.1] * 7 # A slight rotation applied to all joints
    joints_shift_reverse = [-0.3] * 7 # A slight rotation applied to all joints
    joints_ascending_shift = list((x/10.0 for x in range(1, 8, 1))) # Why not


    ## Task 0
    if not all_close(joints_start, move_group.get_current_joint_values(), target_tolerance):
      print('\nTask0: Ensure the arm is at starting position')
      print("Moving to starting position...")
      go_to_joint_state(joints_start)


    ## Task 1
    print('\nTask1: Manipulating joint angles directly to move to a different state')
    input("Press 'Enter' to continue; 'Ctrl+D' to exit\n")
    move_sequence = [
      joints_small_shift, 
      joints_shift_reverse, 
      joints_ascending_shift
    ]
    for i in range(3):
      go_to_joint_state(move_sequence[i])
      rospy.sleep(1.)
    

    ## Task 2
    print('\nTask 2: Setting a pose in space for the end-effector, computing a trajectory to it and executing the plan') 
    input("Press 'Enter' to continue; 'Ctrl+D' to exit\n")
    ## If iiwa were to get in a position to pick up an object from a table, 
    ## it might look something like the following pose 
    pose_ready_to_pick_up = geometry_msgs.msg.Pose()
    q = quaternion_from_euler(pi, 0, 0)
    pose_ready_to_pick_up.orientation.x = q[0]
    pose_ready_to_pick_up.orientation.y = q[1]
    pose_ready_to_pick_up.orientation.z = q[2]
    pose_ready_to_pick_up.orientation.w = q[3]
    pose_ready_to_pick_up.position.x = 0.4
    pose_ready_to_pick_up.position.y = -0.4
    pose_ready_to_pick_up.position.z = 0.5

    go_to_pose_goal(pose_ready_to_pick_up)
    

    ## Task 3
    print('\nTask 3: Plan a trajectory in Cartesian coordinates from a list of waypoints and execute') 
    input("Press 'Enter' to continue; 'Ctrl+D' to exit\n")
    # Using the previous pose as a starting position, specify a few waypoints
    # for the end-effector to go through. The waypoints are selected such that 
    # overall it resembles a Pick-and-Place operation.  

    # Set up some waypoints
    waypoints = []
    scale = 1.0
    wpose = move_group.get_current_pose().pose
    # Waypoint 1
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))
    # Waypoint 2
    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    wpose.orientation.x += 1  # Also change orientation
    waypoints.append(copy.deepcopy(wpose))
    # Waypoint 3
    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    wpose.position.z += scale * 0.1  # and down (z)
    wpose.orientation.x -= 1  # # Restore orientation
    waypoints.append(copy.deepcopy(wpose))

    # Plan trajectory
    cartesian_plan, fraction = plan_cartesian_path(waypoints)
    display_trajectory(cartesian_plan)

    # Execute
    move_group.execute(cartesian_plan, wait=True)

    print("Moving back to starting position...")
    go_to_joint_state(joints_start)

  except (rospy.ROSInterruptException, EOFError, KeyboardInterrupt) as e:
    return

if __name__ == '__main__':
  main()
