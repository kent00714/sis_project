## import file 
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from math import pi
from std_msgs.msg import String, Float64
from moveit_commander.conversions import pose_to_list
import ik_4dof

class pick_object(object):
	def __init__(self):
		# initial publisher for gripper command topic which is used for gripper control
		self.pub_gripper = rospy.Publisher("/gripper_joint/command", Float64, queue_size=1)
		self.sub_goal = rospy.Subscriber("/sis_competition/object", Point, callback, queue_size = 1)
		

		############################ Method : Using IK to calculate joint value ############################

		# After determining a specific point where arm should move, we input x,y,z,degree to calculate joint value for each wrist. 

		pose_goal = Pose()
		pose_goal.position.x = self.goal.x
		pose_goal.position.y = self.goal.y
		pose_goal.position.z = self.goal.z
		# ik_4dof.ik_solver(x, y, z, degree)
		print "pose goal"
		print pose_goal.position.x, pose_goal.position.y, pose_goal.position.z

		joint_value = ik_4dof.ik_solver(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z, -90)

		# Reference: https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html


		### Go home
		#self.home() 

	def close(self)
		# close gripper
		rospy.sleep(2)
		grip_data = Float64()
		grip_data.data = 2.0 
		self.pub_gripper.publish(grip_data)
		rospy.sleep(2)

	def open(self)
		# close gripper
		rospy.sleep(2)
		grip_data = Float64()
		grip_data.data = 0.5
		self.pub_gripper.publish(grip_data)
		rospy.sleep(2)


	def home(self):

		############################ Method : Joint values (Go home!)############################

		# We can get the joint values from the group and adjust some of the values:

		# Go home!!!
		joint_goal = self.move_group.get_current_joint_values()
		joint_goal[0] = 0   		# arm_shoulder_pan_joint
		joint_goal[1] = -pi*5/13   	# arm_shoulder_lift_joint
		joint_goal[2] = pi*3/4   	# arm_elbow_flex_joint
		joint_goal[3] = pi/3   		# arm_wrist_flex_joint
		joint_goal[4] = 0   		# gripper_joint

		# The go command can be called with joint values, poses, or without any
		# parameters if you have already set the pose or joint target for the group
		self.move_group.go(joint_goal, wait=True)

		# Calling ``stop()`` ensures that there is no residual movement
		self.move_group.stop()
		
	def onShutdown(self):
		rospy.loginfo("Shutdown.")

	def callback(self, data)
		print "receive object pose"
		self.goal = Point
		self.goal.x = data.x
		self.goal.y = data.y
		self.goal.z = data.z
		print data.x, data.y, data.z

if __name__ == '__main__': 
	rospy.init_node('pick',anonymous=False)
	rospy.sleep(2)
	pick_object = pick_object()
	rospy.on_shutdown(pick_object.onShutdown)
	rospy.spin()