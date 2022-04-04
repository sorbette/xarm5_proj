#!/usr/bin/python3
from __future__ import print_function
from itertools import groupby

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import csv
import time
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import *

## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):
    """MoveGroupPythonIntefaceTutorial"""
    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial',
                        anonymous=True)

        ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        ## the robot:
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
        ## to the world surrounding the robot:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to one group of joints.  In this case the group is the joints in the Panda
        ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
        ## you should change this value to the name of your robot arm planning group.
        ## This interface can be used to plan and execute motions on the XARM:

        group_name = "xarm5"
        group = moveit_commander.MoveGroupCommander(group_name)

        planning_frame = group.get_planning_frame()
        print("============ Reference frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()
        print("============ End effector: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Robot Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
    

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        
    def go_to_joint_state(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group
        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        rad = pi/180
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = -22.5 * rad
        joint_goal[1] = -32.2 * rad
        joint_goal[2] = -17.2 * rad 
        joint_goal[3] = 49.4 * rad 
        joint_goal[4] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        group.stop()

        print('hello')
        print(group.get_current_state())
        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
        

    # GO TO CARTESIAN COORDINATES
    def go_to_pose_goal(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group
        joints = group.get_current_joint_values()

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1
        pose_goal.position.x = 0.1
        pose_goal.position.y = 0.2
        pose_goal.position.z = 0.4
        group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()
        print('hello')
        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        # current_pose = self.group.get_current_pose().pose
        # return all_close(pose_goal, current_pose, 0.01)
        
    ## PLANNING WITH WAYPOINTS

    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through:
        ##
        self.waypoints = []

        wpose = group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        self.waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        self.waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        self.waypoints.append(copy.deepcopy(wpose))
        
        
        # We want the Cartesian path to be interpolated at a resolution of 10 CENTIMETERS
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = group.compute_cartesian_path(
                                        self.waypoints,   # waypoints to follow
                                        0.01,        # eef_step 
                                        0.0)         # jump_threshold
        # Note: We are just planning, not asking group to actually move the robot yet:
        print(self.waypoints)
        return plan, fraction

       ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
        group = self.group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        group.execute(plan, wait=True)
        t1 = time.perf_counter()
        ## WHILE EXECUTING THE PLAN, MAKE THE VALUES OF THE COORDINATES AND THE TIMING ONTO A CSV FILE 
        with open('robot_states.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['timing','x','y','z'])
        position = group.get_current_state()
        if position == self.waypoints:
            # LISTING THE INFORMATION WE WANT WHEN A ROBOT HITS A WAYPOINT: COORDINATES OF THE END EFFECTOR AND TIMING
            pose = group.get_current_pose().pose
            # t2 = time.perf_counter()
            # t_diff = t2-t1
            # to_be_printed = [t_diff, pose]
            # print(to_be_printed)
            # robot_state.writerow([to_be_printed])
        else:
          print('hello')    
            
    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL


def main():
    try:
        # print ("============ Press `Enter` to begin the interface by setting up the moveit_commander (press ctrl-d to exit) ...")
        # input()
        tutorial = MoveGroupPythonIntefaceTutorial()

        # input(
        #     "============ Press `Enter` to execute a movement using a joint state goal ..."
        # )
        tutorial.go_to_joint_state()

        # print("============ Press `Enter` to execute a movement using a pose goal ...")
        # input()
        tutorial.go_to_pose_goal()

        # print("============ Press `Enter` to plan and display a Cartesian path ...")
        # input()
        cartesian_plan, fraction = tutorial.plan_cartesian_path()

        # print("============ Press `Enter` to execute a saved path ...")
        # input()
        tutorial.execute_plan(cartesian_plan)

        print("============ Python tutorial demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
  main()