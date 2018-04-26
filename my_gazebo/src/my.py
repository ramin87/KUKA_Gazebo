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

from digilab_control.cModelGripper import CModelGripper
from robotiq_c_model_control.msg import _CModel_robot_input  as inputMsg

# ===================== Importing files and functions===========================
# find the current path:
# import os
# cwd = os.getcwd()
# print cwd
import sys
# sys.path.append('/home/ramin/catkin_ws_myself/src/library/src')
sys.path.append('./library/src')
# print('\n'.join(sys.path))
from filterForce import wrenchMovingAverage
from interfaceLib import color
from rlsLib import rls
from impedanceControllerLib import impedanceController

#--------------------------------------------------------------------
'''
Check the tcp of the UR10, it must be 0,0,0

'''
#--------------------------------------------------------------------
class CarryObject(CModelGripper):
    def __init__(self):
        
        
        self.lrc = matrix([[0.0],[0.0],[0.0]])
        
        # node functions:
        rospy.init_node('CarryObject')
        CModelGripper.__init__(self)
        
        # Start listener:
        self.listener = tf.TransformListener()

        # Subscribe to robot  (/calibrated_fts_wrench)
        rospy.Subscriber("/joint_states", JointState, self.cb1)
        
        # Publish to robot
        self.urScriptPub = rospy.Publisher("/ur_driver/URScript", String, queue_size=1)
        
        # wait to establish the connection completely:
        rospy.sleep(3.0)

        # Move the object to the first camera position:
        # self.startPosition = [-1.075233284627096, -1.8846333662616175, -2.0415623823748987, 0.7929238080978394, 1.066290020942688, -3.1720603148089808]
        self.startPosition = [-1.570688549672262, -1.2217925230609339, 1.919877052307129, -2.2689769903766077, 1.5707589387893677, -4.7985707418263246e-05]

        command = "movej(" +str(self.startPosition) + ",1,10,5,0)";
        # self.urSrciptToString(
        #         move="movej",
        #         jointPose=self.startPosition,
        #         a=1,
        #         v=10,
        #         t=5,
        #         r=0)
        
        print command    
        # publish command to robot
        self.urScriptPub.publish(command)


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
        self.joints = data.position
        self.jointsVel = data.velocity
        self.n1 += 1

# spin --------------------------------------------------------------
#--------------------------------------------------------------------
    def spin(self):
        while (not rospy.is_shutdown()) and (self.simTime < 100):
            
            self.simTime = time.time()-self.simStartTime
            
            [rotation_matrix,rot,trans]=self.FTSframe()

            self.alpha = rot[2]
            
            self.P_r = matrix([[trans[0]],[trans[1]],[trans[2]]])
            
            P_d = self.P_r + (rotation_matrix*matrix([[0],[0],[0.5]]))
            
            hForces = rotation_matrix * self.hForcesS
            hTorques = rotation_matrix * self.hTorquesS
            hWrench = matrix([ [hForces[0,0]],[hForces[1,0]],[hTorques[2,0]] ])

            # self.pMode.red(hForces)
            # self.pMode.green(self.P_r[0:2,0]) 
            # self.commandVel = self.controllerOut.update(self.P_r[0:2,0],self.alpha,self.tool_vel[0:2,0],self.tool_vel[2,0],hWrench[0:2,0] ,hWrench[2,0])
            
            self.thetaN , eN = self.rlsOut.updateWithReset(hWrench[0:2,0],hWrench[2,0])
            # print '-------------------------------------------'
            # print eN
            # self.pMode.yellow(self.thetaN)
            if np.absolute(self.thetaN[1,0]) < np.absolute(self.thetaN[0,0]):
                self.B = matrix([[0,0,0],[0,self.b2,0],[0,0,self.b3]])
            else:
                self.B = matrix([[self.b1,0,0],[0,0,0],[0,0,self.b3]])
                
            self.commandVel = self.controllerOut.update2(self.B,hWrench[0:2,0] ,hWrench[2,0],self.thetaN)
            # self.commandVel = self.controllerOut.update(self.P_r[0:2,0],self.alpha,matrix([[0],[0]]),0.0,matrix([[0],[0]]) ,0.0)
            # print self.commandVel
            # self.B[0,0] = min(0.1, max(0.002, abs(hWrench[0,0])*0.01))
            # self.B[1,1] = min(0.1, max(0.002, abs(hWrench[1,0])*0.01))

            # print self.B[1,1]

            # V_ref = self.B * hWrench
            
            ## Command -------------------------------------------
            ## limit veocity:
            V_ref = self.limit_vel(self.commandVel)
            # self.pMode.green(V_ref)
            # command = "speedl([" +str(V_ref[0,0]) +","+  str(V_ref[1,0]) +","+  str(V_ref_z) +",0,0,"+ str(V_ref[2,0]) + "],0.1, "+ str(1/self.simRate) + ")";
            command = "speedl([" +str(V_ref[0,0]) +","+  str(V_ref[1,0]) +",0,0,0,"+ str(V_ref[2,0]) + "],0.1, "+ str(1/self.simRate) + ")";
            # command = "speedl(["+  str(V_ref[0,0]) +","+  str(V_ref[1,0]) +",0,0,0,"+  str(V_ref[2,0]) +"],1, "+ str(2/self.simRate) + ")";
            # command = "speedl([0,0,0,0,0,"+  str(V_ref[2,0]) +"],3, "+ str(2/self.simRate) + ")";
            # command = "speedl(["+  str(V_ref[0,0]) +","+  str(V_ref[1,0]) +",0,0,0,0],3, "+ str(2/self.simRate) + ")";
            # command = "speedl([0,"+  str(V_ref[1,0]) +",0,0,0,0],1, "+ str(2/self.simRate) + ")";
            # command = "speedl([0,0.01,0,0,0,0],1, "+ str(2/self.simRate) + ")"
            
            # print "-----------------/Spin-----------------------"
            # print command
            # print self.B * self.GraspingMatrix(lhr) * hWrench
            # print "||fx: " + str(hForces[0]) + "||fy: " + str(hForces[1]) + "||tz: " + str(hTorques[2])
            # print "||vx: " + str(V_ref[0,0]) + "||vy: " + str(V_ref[1,0]) + "||wz: " + str(V_ref[2,0])
            # print "-----------------Spin/-----------------------"
            self.urScriptPub.publish(command)

            # if (hForces[0,0]**2+hForces[1,0]**2+hForces[2,0]**2)**0.5 > self.f_hys:
            #     self.urScriptPub.publish(command)
            #     # print command
            #     self.f_hys_en = 1
            # else:
            #     self.f_hys_en = 0

            # if self.f_hys_en == 0:
            #     self.f_hys = 0.5 *(1)
            # else:
            #     self.f_hys = 0.03 *(1)

            # self.output_data_value.wrench.force.y = self.hForcesS[1,0]
            # self.output_data_value.wrench.force.z = hForces[1,0]
            # print "** Vc: " + str(V_c)
            # print "** Vc2: " + str(V_c2)
            self.output_data_value.wrench.force.x = eN
            self.output_data_value.wrench.force.y = self.thetaN[0,0]
            self.output_data_value.wrench.force.z = self.thetaN[1,0]
            self.output_data_value.wrench.torque.x = hWrench[0,0]
            self.output_data_value.wrench.torque.y = hWrench[1,0]
            self.output_data_value.wrench.torque.z = hWrench[2,0]
            self.output_data_value2.wrench.force.x = trans[0]
            self.output_data_value2.wrench.force.y = trans[1]
            self.output_data_value2.wrench.force.z = rot[2]
            self.output_data_value2.wrench.torque.x = V_ref[0,0]
            self.output_data_value2.wrench.torque.y = V_ref[1,0]
            self.output_data_value2.wrench.torque.z = V_ref[2,0]
            
            # self.output_data_value.wrench.force.x = hWrench[0,0]
            # self.output_data_value.wrench.force.y = hWrench[1,0] #self.hForcesS[1,0] #hForces[1,0]
            # self.output_data_value.wrench.force.z = self.lcr2[2,0]
            # self.output_data_value.wrench.torque.x = V_ref[2,0]
            # self.output_data_value.wrench.torque.y = hWrench[2,0]
            # self.output_data_value.wrench.torque.z = hWrench[2,0]
            # self.output_data_value.wrench.force.x = self.hForcesS[0,0]
            # self.output_data_value.wrench.force.y = self.hForcesS[1,0]
            # self.output_data_value.wrench.force.z = self.hForcesS[2,0]
            # self.output_data_value.wrench.torque.x = hForces[0,0]
            # self.output_data_value.wrench.torque.y = hForces[1,0]
            # self.output_data_value.wrench.torque.z = hForces[2,0]
            
            self.output_data.publish(self.output_data_value)
            self.output_data2.publish(self.output_data_value2)
            
            # Go into spin, with rateOption!
            self.rate.sleep()        

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
        CarryObject()

        # Just keep the node alive!
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
