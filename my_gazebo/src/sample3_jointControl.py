#!/usr/bin/env python

import rospy
import time
import tf
from numpy import matrix
from numpy import linalg
import math

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState , FollowJointTrajectoryActionGoal
from actionlib_msgs.msg import GoalStatusArray
import actionlib.msg 

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


class color:
   PURPLE = '\033[95m'
   CYAN = '\033[96m'
   DARKCYAN = '\033[36m'
   BLUE = '\033[94m'
   GREEN = '\033[92m'
   YELLOW = '\033[93m'
   RED = '\033[91m'
   BOLD = '\033[1m'
   UNDERLINE = '\033[4m'
   WARNING = YELLOW + BOLD
   FAIL = RED + BOLD
   END = '\033[0m'

#--------------------------------------------------------------------
class Test1():
    def __init__(self):
        
        # print(self.GraspingMatrix(matrix([[1],[2]])))
        
        # Start listener:
        self.listener = tf.TransformListener()
        self.gripperState = FollowJointTrajectoryActionGoal()
        print self.gripperState

        

        # Subscribe to robot  (/calibrated_fts_wrench)
        rospy.Subscriber("/gripper/state", JointTrajectoryControllerState, self.cb1)
        
        # Publish to robot
        self.gripperTopic = rospy.Publisher("/gripper/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, queue_size=1)

        print color.WARNING + 'Hello World !' + color.END
        
        # Go into spin, with rateOption!
        self.rate = rospy.Rate(10)          # 10hz
        rospy.loginfo(rospy.get_caller_id() + "Test started...")
        self.spin()

#CBs ----------------------------------------------------------------
#--------------------------------------------------------------------
    def cb1(self, data):
        # Get update from the manipulator:
        self.gripperState.header = data.header
        self.gripperState.goal_id.stamp = data.header.stamp
        # self.gripperState.joint_names = data.joint_names
        # self.gripperState.points = data.actual.positions[0] + 0.01
        # self.gripperTopic.publish(self.gripperState)
        # print self.gripperState
        
        
# spin --------------------------------------------------------------
    def spin(self):
        while (not rospy.is_shutdown()):
            
            start_time = rospy.get_rostime() 
            # print time:
            # print("hello")
            end = time.time()
            
            # self.urScriptPub.publish(command)

            self.gripperState.goal.trajectory.header.frame_id = '/world'
            self.gripperState.goal.trajectory.joint_names = 'gripper_finger1_joint'
            
            # self.gripperState.goal.trajectory.points = {'positions':0.3,'velocities':[0.0],'accelerations':[0.0],'effort':[],'time_from_start':[],'secs':0,'nsecs':0}
            # self.gripperState.goal.trajectory.points.positions = 0.3
            self.gripperState.goal.trajectory.points = [{'positions':0.1,'velocities':[1.0],'accelerations':[1.0]}]

            print self.gripperState
            # 0.3
            # self.gripperState.goal.trajectory.points.velocities = 0.1
            # self.gripperState.goal.trajectory.points.accelerations = 1

            
            # Go into spin, with rateOption!
            self.rate.sleep()        

# --------------------------------------------------------------------
    def FTSframe(self):  
        try:			
            self.listener.waitForTransform('/base', '/optoforce_frame', rospy.Time(0),rospy.Duration(1))
            (trans,rot) = self.listener.lookupTransform('/base', '/optoforce_frame', rospy.Time(0))
            # print trans
            # # print rot
            # R = tf.transformations.euler_from_quaternion(rot)
            transrotM = self.listener.fromTranslationRotation(trans, rot)
            # print transrotM
            rotationMat = transrotM[0:3,0:3]
            print transrotM
            # print rotationMat
            # print self.RotationMatrix(R)
            # ttt1 = rotationMat * matrix([[0],[0],[1]])
            # ttt2 = rotationMat.T * matrix([[0],[1],[0]])
            # print rotationMat
            # rot_matrix_z = matrix([[ math.cos(3.14), -math.sin(3.14) ,0], [  math.sin(3.14), math.cos(3.14), 0 ] , [ 0, 0 ,1] ])
            # print rot_matrix_z
            # print rot_matrix_z * rotationMat
            # print self.RotationMatrix(R)
            print "-------------------------------------------------"
            # print R
            return transrotM
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "EXCEPTION"
#             pass
            
# #--------------------------------------------------------------------    
#     def RotationMatrix(self, rot):
#         rot_matrix_x = matrix([[1, 0, 0 ], [0, math.cos(rot[0]), -math.sin(rot[0])], [ 0, math.sin(rot[0]), math.cos(rot[0])] ])
#         rot_matrix_y = matrix([[ math.cos(rot[1]), 0, math.sin(rot[1])],[0, 1, 0 ], [ -math.sin(rot[1]), 0, math.cos(rot[1])] ])
#         rot_matrix_z = matrix([[ math.cos(rot[2]), -math.sin(rot[2]) ,0], [  math.sin(rot[2]), math.cos(rot[2]), 0 ] , [ 0, 0 ,1] ])
#         rot_matrix = rot_matrix_z * rot_matrix_y * rot_matrix_x
#         return rot_matrix

# #--------------------------------------------------------------------    
#     def GraspingMatrix(self, Pg):
#         G_matrix = matrix([[1,0,0],[0,1,0],[-Pg[1,0],Pg[0,0],1]])
#         return G_matrix


            

#--------------------------------------------------------------------    
# Here is the main entry point
if __name__ == '__main__':
    try:
        # Init the node:
        rospy.init_node('Tutorial_UR10')


        start = time.time()
        # Initializing and continue running the class Test1:
        Test1()

        # Just keep the node alive!
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
