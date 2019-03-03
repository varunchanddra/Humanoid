import rospy
import actionlib
import os
import time
from geometry_msgs.msg import Twist
from math import pi, sin, cos
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal,
                              GripperCommandAction,
                              GripperCommandGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import PointHeadAction, PointHeadGoal




class BaseMovements:

    def __init__(self):
        
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist)
        self.rate = 50
        self.r = rospy.Rate(self.rate)
        

    def move(self, dist, speed):
        
        move_cmd = Twist()
        move_cmd.linear.x = speed
        duration = dist / speed
        ticks = int(duration * self.rate)

        for t in range(ticks):          
            self.cmd_vel.publish(move_cmd)                
            self.r.sleep()
        
        self.cmd_vel.publish( Twist() )


class HeadMovements():

    def __init__(self):
        
        self.joint_names = ["head_pan_joint", "head_tilt_joint"]
        self.joint_positions = [ 0.0, 0.0]
        self.client = actionlib.SimpleActionClient("head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to head_controller")

    def move(self, inp, duration = 5.0):
        
        self.joint_positions = inp
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        traj.points.append(JointTrajectoryPoint()) 
        traj.points[0].positions = self.joint_positions
        traj.points[0].velocities = [0.0] * len(self.joint_positions)
        traj.points[0].accelerations = [0.0] * len(self.joint_positions)
        traj.points[0].time_from_start = rospy.Duration(5.0)

        head_goal = FollowJointTrajectoryGoal()
        head_goal.trajectory = traj
        head_goal.goal_time_tolerance = rospy.Duration(0.0)
        
        self.client.send_goal(head_goal)
        self.client.wait_for_result(rospy.Duration(6.0))

class TorsoMovements():

    def __init__(self):
        
        self.joint_names = ["torso_lift_joint"]
        self.joint_positions = [ 4.0]
        self.client = actionlib.SimpleActionClient("torso_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for torso_controller...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to torso_controller")

    def move(self, dist, duration = 5.0):
        
        self.joint_positions = [ dist ]
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        traj.points.append(JointTrajectoryPoint()) 
        traj.points[0].positions = self.joint_positions
        traj.points[0].velocities = [0.0] * len(self.joint_positions)
        traj.points[0].accelerations = [0.0] * len(self.joint_positions)
        traj.points[0].time_from_start = rospy.Duration(5.0)

        head_goal = FollowJointTrajectoryGoal()
        head_goal.trajectory = traj
        head_goal.goal_time_tolerance = rospy.Duration(0.0)
        
        self.client.send_goal(head_goal)
        self.client.wait_for_result(rospy.Duration(6.0))

class ArmMovements():

    def __init__(self):
        
        self.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
              "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        self.joint_positions = [0, 0, 0, 0, 0, 0, 0]
        self.client = actionlib.SimpleActionClient("torso_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for arm_controller...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to arm_controller")

    def move(self, inp, duration = 5.0):
        
        self.joint_positions = inp
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        traj.points.append(JointTrajectoryPoint()) 
        traj.points[0].positions = self.joint_positions
        traj.points[0].velocities = [0.0] * len(self.joint_positions)
        traj.points[0].accelerations = [0.0] * len(self.joint_positions)
        traj.points[0].time_from_start = rospy.Duration(5.0)

        head_goal = FollowJointTrajectoryGoal()
        head_goal.trajectory = traj
        head_goal.goal_time_tolerance = rospy.Duration(0.0)
        
        self.client.send_goal(head_goal)
        self.client.wait_for_result(rospy.Duration(9.0))



if __name__ == "__main__":
    
    rospy.init_node("motion_demo")
#    task = WholeMovements()
#    task.move_forward( 1.5, 0.2 )
   
    head = HeadMovements()
    head.move( [0, pi/4.0])    #looks down 45 
    head.move( [0, -pi/2.0])   #looks up 90
    head.move( [pi/2.0, 0])    #looks left by 90 
    head.move( [-pi/2.0, 0])   #looks right by 90 
    head.move( [0, 0])         #looks forward

    torso = TorsoMovements()
    torso.move(3.0)
    
    arm = ArmMovements()
    #arm.move([1, 1, 1, 1, 1, 1, 1])
    arm.move([1.32, 0.7, 0.0, -2.0, 0.0, -0.57, 0.0])
