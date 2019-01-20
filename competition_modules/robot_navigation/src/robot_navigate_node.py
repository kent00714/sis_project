#!/usr/bin/env python
import rospy
import tf
import actionlib
from robot_navigation.srv import robot_navigation
from apriltags2_ros.msg import AprilTagDetection
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import pi
import numpy as np
class RobotNavigate(object):
    def __init__(self):
        self.srv_navigate = rospy.Service("/robot_navigate", robot_navigation, self.cbNavigate)
        
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # stop at xx cm in front of tag
        self.stop_distance_list = [0.40, 0.62]
        self.park_x = {'0':2.47, '1':2.47, '2':2.47, '5':5.75}
        self.park_y = {'0':0.90, '1':0.90, '2':0.90, '5':5.91}
        self.park_th = {'0':0, '1':0, '2':0, '5':pi}
    def cbNavigate(self, req):
        print "cbNavigate"        
        self.client.wait_for_server()
        # declare goal as geometry_msgs::PoseStamped
        goal = MoveBaseGoal() 
        listener = tf.TransformListener()
        tag_id = req.id
        tag_tf = "tag_" + str(tag_id)

        #try:
        #    listener.waitForTransform("car_base", tag_tf, rospy.Time(), rospy.Duration(4.0))
        #except tf.LookupException as e:
        #    return "NoTrans"

        '''goal.target_pose.header.frame_id = 'car_base'
        goal.target_pose.header.stamp = rospy.Time.now()
        stop_distance = 0
        if req.id == 0 or req.id == 1 or req.id == 2:
            stop_distance = self.stop_distance_list[0]
        elif req.id == 5:
            stop.distance = self.stop_distance_list[1]
        else:
            _result = "Unsupported tag"
            return _result

        goal.target_pose.pose.position.x = trans[0]- stop_distance
        goal.target_pose.pose.position.y = trans[1]
        goal.target_pose.pose.position.z = 0

        # just set it to ten, because I think 5 is not long enough
        exec_time = 10

        # to eliminate the effect that there're some errors in the rot from lookupTransform
        transf_from_rot = tf.transformations.quaternion_matrix(rot)
        a_vec = transf_from_rot[0:2,2] # ax, ay & az
        z_pro = np.array([a_vec[0], a_vec[1], 0]) # projection of z vector on x-y plane
        z_pro_oppo = -z_pro # projection of z vector with opposite direction, which is the direction of robot's heading
        x_hat = z_pro_oppo / np.linalg.norm(z_pro_oppo) # the -z in tag frame is the x in car_base frame
        z_hat = np.array([0,0,1]) # the z direction should be as the same as always
        y_hat = np.cross(z_hat, x_hat) # simple algebra calculation :)

        # back to quaternion
        rot = np.array([[x_hat[0], y_hat[0], z_hat[0], 0],
            [x_hat[1], y_hat[1], z_hat[1], 0],
            [x_hat[2], y_hat[2], z_hat[2], 0],
            [0, 0, 0, 1]])
        quat = tf.transformations.quaternion_from_matrix(rot)
        
        # just want to check if it's the same as I thought
        print str(quat)

        # I think we're all set now!
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]'''
     
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.park_x[str(req.id)]
        goal.target_pose.pose.position.y = self.park_y[str(req.id)]
        goal.target_pose.pose.position.z = 0

        quat = tf.transformations.quaternion_from_euler(0, 0, self.park_th[str(req.id)])

        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]

        # Go! Pikachu!
        self.client.send_goal(goal)
        _result = self.client.wait_for_result(rospy.Duration(10)) 
        if _result == False:
            self.client.cancel_goal()
        return str(_result)

if __name__ == '__main__':
    rospy.init_node('robot_navigation',anonymous=False)
    node = RobotNavigate()
    rospy.spin()
