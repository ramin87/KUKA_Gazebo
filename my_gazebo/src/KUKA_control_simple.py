#!/usr/bin/env python

import rospy
import time
import tf
from numpy import matrix
from numpy import linalg
import math
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ===================== Importing files and functions===========================
# find the current path:
# import os
# cwd = os.getcwd()
# print cwd
import sys
# sys.path.append('/home/ramin/catkin_ws_myself/src/library/src')
sys.path.append('./library/src')


#--------------------------------------------------------------------
'''
Comments:


'''
#--------------------------------------------------------------------
class KUKA_control():
    def __init__(self):
        
        # node functions:
        rospy.init_node('KUKA_control')

        # Publish to robot
        self.kuka_joints = rospy.Publisher("/iiwa/PositionJointInterface_J1_controller/command", String, queue_size=2)
        self.forceRvizTopic = rospy.Publisher("/forceRvizUR10_Endeffector",WrenchStamped,queue_size=2)
        self.output_data = rospy.Publisher("/dateOut",WrenchStamped,queue_size=2)
        self.output_data2 = rospy.Publisher("/dateOut2",WrenchStamped,queue_size=2)
        
        # Check the publishers:
        while (self.urScriptPub.get_num_connections() == 0):
            rospy.sleep(0.01)
        # sys.exit()

        # Go into spin, with rateOption!
        self.rate = rospy.Rate(self.simRate)          # 10hz
        rospy.loginfo(rospy.get_caller_id() + " started...")
        self.simStartTime = time.time()
        self.simTime = time.time()-self.simStartTime
        self.spin()

#CBs ----------------------------------------------------------------
#--------------------------------------------------------------------
#--------------------------------------------------------------------
    def cb1(self, data):
        

# spin --------------------------------------------------------------
#--------------------------------------------------------------------
    def spin(self):
        while (not rospy.is_shutdown()) and (self.simTime < 100):
            
            

#--------------------------------------------------------------------
#--------------------------------------------------------------------
    def FTSframe(self):  
        try:			
            # /fts_link , /tool0 , /optoforce_frame
            forceSensorFrame = 'tool0'
            self.listener.waitForTransform('/base', forceSensorFrame, rospy.Time(0),rospy.Duration(2))
            (trans,rot) = self.listener.lookupTransform('/base', forceSensorFrame, rospy.Time(0))
            
            R = tf.transformations.euler_from_quaternion(rot)
            transrotM = self.listener.fromTranslationRotation(trans, rot)
            
            rotationMat = transrotM[0:3,0:3]
            
            # ttt1 = rotationMat * matrix([[0],[0],[1]])
            # ttt2 = rotationMat.T * matrix([[0],[1],[0]])
            # print ttt1

            # rot_matrix_z = matrix([[ math.cos(3.14), -math.sin(3.14) ,0], [  math.sin(3.14), math.cos(3.14), 0 ] , [ 0, 0 ,1] ])
            
            # rotate world frame to match with speedl frame:
            # rot_matrix_x_pi = matrix([[1, 0, 0 ], [0, math.cos(3.14), -math.sin(3.14)], [ 0, math.sin(3.14), math.cos(3.14)] ])
            # R_final = rot_matrix_z * rotationMat
            R_final = rotationMat
            
            return [R_final, R, trans]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "EXCEPTION"
            pass
            
#--------------------------------------------------------------------
#--------------------------------------------------------------------    
    def RotationMatrix(self, rot):
        rot_matrix_x = matrix([[1, 0, 0 ], [0, math.cos(rot[0]), -math.sin(rot[0])], [ 0, math.sin(rot[0]), math.cos(rot[0])] ])
        rot_matrix_y = matrix([[ math.cos(rot[1]), 0, math.sin(rot[1])],[0, 1, 0 ], [ -math.sin(rot[1]), 0, math.cos(rot[1])] ])
        rot_matrix_z = matrix([[ math.cos(rot[2]), -math.sin(rot[2]) ,0], [  math.sin(rot[2]), math.cos(rot[2]), 0 ] , [ 0, 0 ,1] ])
        rot_matrix = rot_matrix_z * rot_matrix_y * rot_matrix_x
        return rot_matrix


#--------------------------------------------------------------------
#--------------------------------------------------------------------    
# Here is the main entry point
if __name__ == '__main__':
    try:
        KUKA_control()

        # Just keep the node alive!
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
