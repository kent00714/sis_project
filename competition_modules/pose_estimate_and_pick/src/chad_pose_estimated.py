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