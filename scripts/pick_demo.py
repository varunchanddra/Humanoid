import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list



if __name__ == "__main__":
	moveit_commander.roscpp_initialize(sys.argv)
	print("*******************")
        #rospy.init_node('pick_demo', anonymous=True)
	robot = moveit_commander.RobotCommander()	
	scene = moveit_commander.PlanningSceneInterface()

	print "============ Robot Groups:", robot.get_group_names()
	
	group_name = "arm"
	move_group = moveit_commander.MoveGroupCommander(group_name)
	
	eef_link = move_group.get_end_effector_link()
	print "============ End effector: %s" % eef_link

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.5
        pose_goal.position.y = 0.5
        pose_goal.position.z = 0.5
        
        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        #move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        #move_group.clear_pose_targets()

        move_group.execute(plan, wait=True)
