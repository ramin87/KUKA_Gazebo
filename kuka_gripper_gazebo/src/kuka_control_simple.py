#!/usr/bin/env python

import rospy
import time
import tf
from numpy import matrix
from numpy import linalg
import numpy.matlib
import math
import numpy as np

from std_msgs.msg import String, Float64
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
# Just for printing colorful
class color():
    def __init__(self):
        self.PURPLE = '\033[95m'
        self.CYAN = '\033[96m'
        self.DARKCYAN = '\033[36m'
        self.BLUE = '\033[94m'
        self.GREEN = '\033[92m'
        self.YELLOW = '\033[93m'
        self.RED = '\033[91m'
        self.BOLD = '\033[1m'
        self.UNDERLINE = '\033[4m'
        self.WARNING = self.YELLOW + self.BOLD
        self.FAIL = self.RED + self.BOLD
        self.END = '\033[0m'

    def warning(self,data):
        dataSTR = str(data)
        print self.WARNING + dataSTR + self.END

    def error(self,data):
        dataSTR = str(data)
        print self.FAIL + dataSTR + self.END
    
    def red(self,data):
        dataSTR = str(data)
        print self.RED + dataSTR + self.END

    def blue(self,data):
        dataSTR = str(data)
        print self.BLUE + dataSTR + self.END
    
    def green(self,data):
        dataSTR = str(data)
        print self.GREEN + dataSTR + self.END
    
    def white(self,data):
        dataSTR = str(data)
        print data

    def yellow(self,data):
        dataSTR = str(data)
        print self.YELLOW + dataSTR + self.END

class controller():
    def __init__(self, timeStep=None , Ep0= np.matlib.zeros((3, 1)) , Eo0=np.matlib.zeros((3, 1)) ):
        self.Md_p = 1.0 # translational inertia
        self.Kd_p = 10.0 # translational stiffness
        self.Dd_p = 8.0  # translational damping
        self.Md_o = 1.0 # rotational inertia
        self.Kd_o = 8.0 # rotational stiffness
        self.Dd_o = 5.5  # rotational damping

        self.Kp_click = 4.0;
        self.Ko_click = 0.5;

        # 6 DoF:
        self.ddEp = np.matlib.zeros((3, 1))
        self.dEp = np.matlib.zeros((3, 1))
        self.Ep =  Ep0
        self.ddEo =  np.matlib.zeros((3, 1))
        self.dEo =  np.matlib.zeros((3, 1))
        self.Eo =  Eo0

        self.Vd6 = np.matlib.zeros((6, 1))
        self.hWrench = np.matlib.zeros((6, 1))

    

    def Jacobian_KUKA7(self,q):
        q1 = q[0];
        q2 = q[1];
        q3 = q[2];
        q4 = q[3];
        q5 = q[4];
        q6 = q[5];
        t2 = math.sin(q1);
        t3 = math.cos(q1);
        t4 = math.cos(q3);
        t5 = math.cos(q2);
        t6 = math.sin(q3);
        t7 = math.sin(q2);
        t8 = math.sin(q4);
        t9 = t3*t6;
        t10 = t2*t4*t5;
        t11 = t9+t10;
        t12 = math.cos(q4);
        t13 = math.sin(q6);
        t14 = math.cos(q5);
        t15 = math.sin(q5);
        t16 = math.cos(q6);
        t17 = t3*t4*1.0;
        t36 = t2*t5*t6;
        t18 = t17-t36;
        t19 = t5*t8;
        t20 = t4*t7*t12*-1.0;
        t21 = t19+t20;
        t22 = t14*t21*1.0;
        t23 = t6*t7*t15*1.0;
        t24 = t22+t23;
        t25 = t13*t24*1.31e-1;
        t26 = t5*t12*4.0e-1;
        t27 = t5*t12;
        t28 = t4*t7*t8;
        t29 = t27+t28;
        t30 = t16*t29*1.31e-1;
        t31 = t4*t7*t8*4.0e-1;
        t32 = t11*t12;
        t33 = t2*t7*t8;
        t34 = t32+t33;
        t35 = t14*t34;
        t37 = t15*t18*1.0;
        t38 = t35+t37;
        t69 = t8*t11*1.0;
        t68 = -t69;
        t39 = -t68;
        t41 = t2*t7*t12;
        t40 = t39-t41;
        t42 = t2*t6;
        t43 = t3*t4*t5*-1.0;
        t44 = t42+t43;
        t45 = t5*4.2e-1;
        t46 = t25+t26+t30+t31+t45;
        t47 = t2*t4;
        t48 = t3*t5*t6;
        t49 = t47+t48;
        t50 = t25+t26+t30+t31;
        t51 = t8*t44;
        t52 = t3*t7*t12*1.0;
        t53 = t51+t52;
        t54 = t16*t53*1.31e-1;
        t55 = t8*t44*4.0e-1;
        t56 = t15*t49;
        t57 = t12*t44;
        t58 = t3*t7*t8*-1.0;
        t59 = t57+t58;
        t60 = t14*t59;
        t61 = t56+t60;
        t62 = t13*t61*-1.31e-1;
        t63 = t3*t7*t12*4.0e-1;
        t70 = t15*t21*-1.0;
        t64 = -t70;
        t71 = t6*t7*t14*1.0;
        t65 = -t71;
        t66 = t64+t65;
        t67 = t25+t30;

        # J = matrix([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]);
        # J[0,0] = t2*t7*-4.2e-1+t8*t11*4.0e-1-t13*t38*1.31e-1+t16*t40*1.31e-1-t2*t7*t12*4.0e-1
        # J[0,1] = t54+t55+t62+t63+t3*t7*4.2e-1
        # J[0,2] = 0.0
        # J[0,3] = 0.0
        # J[0,4] = 0.0
        # J[0,5] = 1.0
        # J[0,6] = t3*t46
        # J[0,7] = t2*t46
        # J[0,8] = t7*-4.2e-1-t7*t12*4.0e-1+t4*t5*t8*4.0e-1-t7*t12*t16*1.31e-1+t4*t5*t8*t16*1.31e-1+t5*t6*t13*t15*1.31e-1-t7*t8*t13*t14*1.31e-1-t4*t5*t12*t13*t14*1.31e-1
        # J[0,9] = t2*-1.0
        # J[0,10] = t3
        # J[0,11] = 0.0
        # J[0,12] = t2*t4*t8*4.0e-1+t3*t5*t6*t8*4.0e-1+t2*t4*t8*t16*1.31e-1+t2*t6*t13*t15*1.31e-1+t3*t5*t6*t8*t16*1.31e-1-t3*t4*t5*t13*t15*1.31e-1-t2*t4*t12*t13*t14*1.31e-1-t3*t5*t6*t12*t13*t14*1.31e-1
        # J[0,13] = t3*t4*t8*-4.0e-1+t2*t5*t6*t8*4.0e-1-t3*t4*t8*t16*1.31e-1-t3*t6*t13*t15*1.31e-1+t2*t5*t6*t8*t16*1.31e-1-t2*t4*t5*t13*t15*1.31e-1+t3*t4*t12*t13*t14*1.31e-1-t2*t5*t6*t12*t13*t14*1.31e-1
        # J[0,14] = t7*(t6*t8*4.0e2+t6*t8*t16*1.31e2-t4*t13*t15*1.31e2-t6*t12*t13*t14*1.31e2)*-1.0e-3
        # J[0,15] = t3*t7
        # J[0,0] = t2*t7
        # J[0,0] = t5
        # J[0,0] = -t18*t50-t6*t7*(t8*t11*4.0e-1-t13*t38*1.31e-1+t16*t40*1.31e-1-t2*t7*t12*4.0e-1)*1.0
        # J[0,0] = t49*t50*-1.0-t6*t7*(t54+t55+t62+t63)*1.0
        # J[0,0] = t5*t8*-4.0e-1+t4*t7*t12*4.0e-1-t5*t8*t16*1.31e-1+t4*t7*t12*t16*1.31e-1+t5*t12*t13*t14*1.31e-1+t4*t7*t8*t13*t14*1.31e-1
        # J[0,0] = t49
        # J[0,0] = t36-t3*t4
        # J[0,0] = t6*t7*-1.0
        # J[0,0] = t13*(t2*t4*t14+t3*t5*t6*t14+t3*t7*t8*t15-t2*t6*t12*t15*1.0+t3*t4*t5*t12*t15)*(-1.31e2/1.0e3)
        # J[0,0] = t13*(-t3*t4*t14+t2*t5*t6*t14+t2*t7*t8*t15+t3*t6*t12*t15+t2*t4*t5*t12*t15)*-1.31e-1
        # J[0,0] = t13*(t6*t7*t14-t5*t8*t15+t4*t7*t12*t15)*1.31e-1
        # J[0,0] = t53
        # J[4,] = t41+t68
        # J[4,] = t29
        # J[4,] = t66*(t13*t38*1.31e-1-t16*t40*1.31e-1)*1.0+t67*(t14*t18-t15*t34)*1.0
        # J[4,] = t67*(t14*t49*1.0-t15*t59)*1.0-t66*(t54+t62)
        # J[4,] = t5*t12*t13*(-1.31e2/1.0e3)-t4*t7*t8*t13*(1.31e2/1.0e3)+t5*t8*t14*t16*1.31e-1+t6*t7*t15*t16*1.31e-1-t4*t7*t12*t14*t16*(1.31e2/1.0e3)
        # J[4,] = t14*t49*-1.0+t15*t59
        # J[4,] = t14*t18*1.0-t15*t34*1.0
        # J[5,] = t70+t71
        # J[5,] = 0.0
        # J[5,] = 0.0
        # J[5,] = 0.0
        # J[5,] = t16*t53-t13*t61*1.0
        # J[5,] = t13*t38+t16*(t41-t69)
        # J[5,] = t13*t24*1.0+t16*t29]
        # J[] = [6,7]);
        # J = reshape([t2*t7*-4.2e-1+t8*t11*4.0e-1-t13*t38*1.31e-1+t16*t40*1.31e-1-t2*t7*t12*4.0e-1,t54+t55+t62+t63+t3*t7*4.2e-1,0.0,0.0,0.0,1.0,t3*t46,t2*t46,t7*-4.2e-1-t7*t12*4.0e-1+t4*t5*t8*4.0e-1-t7*t12*t16*1.31e-1+t4*t5*t8*t16*1.31e-1+t5*t6*t13*t15*1.31e-1-t7*t8*t13*t14*1.31e-1-t4*t5*t12*t13*t14*1.31e-1,t2*-1.0,t3,0.0,t2*t4*t8*4.0e-1+t3*t5*t6*t8*4.0e-1+t2*t4*t8*t16*1.31e-1+t2*t6*t13*t15*1.31e-1+t3*t5*t6*t8*t16*1.31e-1-t3*t4*t5*t13*t15*1.31e-1-t2*t4*t12*t13*t14*1.31e-1-t3*t5*t6*t12*t13*t14*1.31e-1,t3*t4*t8*-4.0e-1+t2*t5*t6*t8*4.0e-1-t3*t4*t8*t16*1.31e-1-t3*t6*t13*t15*1.31e-1+t2*t5*t6*t8*t16*1.31e-1-t2*t4*t5*t13*t15*1.31e-1+t3*t4*t12*t13*t14*1.31e-1-t2*t5*t6*t12*t13*t14*1.31e-1,t7*(t6*t8*4.0e2+t6*t8*t16*1.31e2-t4*t13*t15*1.31e2-t6*t12*t13*t14*1.31e2)*-1.0e-3,t3*t7,t2*t7,t5,-t18*t50-t6*t7*(t8*t11*4.0e-1-t13*t38*1.31e-1+t16*t40*1.31e-1-t2*t7*t12*4.0e-1)*1.0,t49*t50*-1.0-t6*t7*(t54+t55+t62+t63)*1.0,t5*t8*-4.0e-1+t4*t7*t12*4.0e-1-t5*t8*t16*1.31e-1+t4*t7*t12*t16*1.31e-1+t5*t12*t13*t14*1.31e-1+t4*t7*t8*t13*t14*1.31e-1,t49,t36-t3*t4,t6*t7*-1.0,t13*(t2*t4*t14+t3*t5*t6*t14+t3*t7*t8*t15-t2*t6*t12*t15*1.0+t3*t4*t5*t12*t15)*(-1.31e2/1.0e3),t13*(-t3*t4*t14+t2*t5*t6*t14+t2*t7*t8*t15+t3*t6*t12*t15+t2*t4*t5*t12*t15)*-1.31e-1,t13*(t6*t7*t14-t5*t8*t15+t4*t7*t12*t15)*1.31e-1,t53,t41+t68,t29,t66*(t13*t38*1.31e-1-t16*t40*1.31e-1)*1.0+t67*(t14*t18-t15*t34)*1.0,t67*(t14*t49*1.0-t15*t59)*1.0-t66*(t54+t62),t5*t12*t13*(-1.31e2/1.0e3)-t4*t7*t8*t13*(1.31e2/1.0e3)+t5*t8*t14*t16*1.31e-1+t6*t7*t15*t16*1.31e-1-t4*t7*t12*t14*t16*(1.31e2/1.0e3),t14*t49*-1.0+t15*t59,t14*t18*1.0-t15*t34*1.0,t70+t71,0.0,0.0,0.0,t16*t53-t13*t61*1.0,t13*t38+t16*(t41-t69),t13*t24*1.0+t16*t29],[6,7]);
        J = matrix([t2*t7*-4.2e-1+t8*t11*4.0e-1-t13*t38*1.31e-1+t16*t40*1.31e-1-t2*t7*t12*4.0e-1,t54+t55+t62+t63+t3*t7*4.2e-1,0.0,0.0,0.0,1.0,t3*t46,t2*t46,t7*-4.2e-1-t7*t12*4.0e-1+t4*t5*t8*4.0e-1-t7*t12*t16*1.31e-1+t4*t5*t8*t16*1.31e-1+t5*t6*t13*t15*1.31e-1-t7*t8*t13*t14*1.31e-1-t4*t5*t12*t13*t14*1.31e-1,t2*-1.0,t3,0.0,t2*t4*t8*4.0e-1+t3*t5*t6*t8*4.0e-1+t2*t4*t8*t16*1.31e-1+t2*t6*t13*t15*1.31e-1+t3*t5*t6*t8*t16*1.31e-1-t3*t4*t5*t13*t15*1.31e-1-t2*t4*t12*t13*t14*1.31e-1-t3*t5*t6*t12*t13*t14*1.31e-1,t3*t4*t8*-4.0e-1+t2*t5*t6*t8*4.0e-1-t3*t4*t8*t16*1.31e-1-t3*t6*t13*t15*1.31e-1+t2*t5*t6*t8*t16*1.31e-1-t2*t4*t5*t13*t15*1.31e-1+t3*t4*t12*t13*t14*1.31e-1-t2*t5*t6*t12*t13*t14*1.31e-1,t7*(t6*t8*4.0e2+t6*t8*t16*1.31e2-t4*t13*t15*1.31e2-t6*t12*t13*t14*1.31e2)*-1.0e-3,t3*t7,t2*t7,t5,-t18*t50-t6*t7*(t8*t11*4.0e-1-t13*t38*1.31e-1+t16*t40*1.31e-1-t2*t7*t12*4.0e-1)*1.0,t49*t50*-1.0-t6*t7*(t54+t55+t62+t63)*1.0,t5*t8*-4.0e-1+t4*t7*t12*4.0e-1-t5*t8*t16*1.31e-1+t4*t7*t12*t16*1.31e-1+t5*t12*t13*t14*1.31e-1+t4*t7*t8*t13*t14*1.31e-1,t49,t36-t3*t4,t6*t7*-1.0,t13*(t2*t4*t14+t3*t5*t6*t14+t3*t7*t8*t15-t2*t6*t12*t15*1.0+t3*t4*t5*t12*t15)*(-1.31e2/1.0e3),t13*(-t3*t4*t14+t2*t5*t6*t14+t2*t7*t8*t15+t3*t6*t12*t15+t2*t4*t5*t12*t15)*-1.31e-1,t13*(t6*t7*t14-t5*t8*t15+t4*t7*t12*t15)*1.31e-1,t53,t41+t68,t29,t66*(t13*t38*1.31e-1-t16*t40*1.31e-1)*1.0+t67*(t14*t18-t15*t34)*1.0,t67*(t14*t49*1.0-t15*t59)*1.0-t66*(t54+t62),t5*t12*t13*(-1.31e2/1.0e3)-t4*t7*t8*t13*(1.31e2/1.0e3)+t5*t8*t14*t16*1.31e-1+t6*t7*t15*t16*1.31e-1-t4*t7*t12*t14*t16*(1.31e2/1.0e3),t14*t49*-1.0+t15*t59,t14*t18*1.0-t15*t34*1.0,t70+t71,0.0,0.0,0.0,t16*t53-t13*t61*1.0,t13*t38+t16*(t41-t69),t13*t24*1.0+t16*t29]);
        
        j2 = J.reshape(7,6).transpose()
        # print j2
        return j2

    def command_position(self,q_old,command_velocity,dt):
        # The reason of the problem must be related to the conversion between float and double

        dq = np.linalg.pinv(self.Jacobian_KUKA7(q_old)) * command_velocity
        # print command_velocity
        # print dq
        for i in [0,1,2,3,4,5]:
            if abs(dq[i,0]) < 0.1:
                dq[i,0] = 0

        # print dq*dt
        # print q_old
        q_new = dq*dt + q_old
        # print dq*dt
        # print q_new
        return q_new

    def command_position_fromforce(self,q_old,command_force,dt,robotP3,robotPhi3,robotDp3,robotDphi3):
        fExt3 = command_force[0:3]
        tExt3 = command_force[3:6]
        tExt3 = matrix([[0.0],[0.0],[0.0]])
        self.Ts = dt
        # fExt3 = np.multiply( np.sign(fExt3) , np.maximum( np.matlib.zeros((3, 1)) , np.absolute(fExt3) - self.forceDeadZone * np.matlib.ones((3, 1)) ) )  
        # tExt3 = np.multiply( np.sign(tExt3) , np.maximum( np.matlib.zeros((3, 1)) , np.absolute(tExt3) - self.torqueDeadZone * np.matlib.ones((3, 1)) ) )  
            
        # for(int i=0;i<6;i++)
        #     {
        #         Fee(i) = (Fee_sign(i)) * fmax(0.0,fabs(Fee(i)) - cmd_args.Fee_dead_zone(i));
        #     }
        # print fExt3
        self.ddEp = (1.0 / self.Md_p) * (- self.Dd_p * self.dEp - self.Kd_p * (self.Ep - robotP3) + fExt3);
        self.ddEo = (1.0 / self.Md_o) * (- self.Dd_o * self.dEo - self.Kd_o * (self.Eo - robotPhi3) + tExt3);   
        # self.ddEp = (1.0 / self.Md_p) * (- self.Dd_p * self.dEp - self.Kd_p * self.Ep + fExt);
        # self.ddEo = (1.0 / self.Md_o) * (- self.Dd_o * self.dEo - self.Kd_o * self.Eo + tExt);   
        # self.ddEp = (1.0 / self.Md_p) * (- self.Dd_p * self.dEp + fExt3);
        # self.ddEo = (1.0 / self.Md_o) * (- self.Dd_o * self.dEo + tExt3);   
        
        self.dEp = self.dEp + self.ddEp * self.Ts;
        self.Ep  = self.Ep + self.dEp * self.Ts;
        
        self.dEo = self.dEo + self.ddEo * self.Ts;
        # print self.Eo
        self.Eo  = self.Eo + self.dEo * self.Ts;
        # print self.Eo

        # print self.dEp
        # print self.Kp_click
        # print robotP3
        # print self.Ep
        self.Vd6[0:3,0] = ( self.dEp - self.Kp_click * (robotP3 - self.Ep) )
        self.Vd6[3:6,0] = ( self.dEo - self.Ko_click * (robotPhi3 - self.Eo) ) 

        final_q = self.command_position(q_old,self.Vd6,dt)
        # print self.Ep
        return final_q


#--------------------------------------------------------------------
class kuka_gripper_controlNode():
    def __init__(self):
        
        # for printing colorful
        self.pMode = color()

        # Node parameters:
        self.simRate = 125.0
        self.simTimeCheck = time.time()

        # Start listener:
        self.listener = tf.TransformListener()

        # kuka controller
        self.currentKukaJoints = matrix([[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]])
        self.v_command = matrix([[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]])
        self.com_x = 0.0
        self.com_y = 0.0
        self.com_z = 0.0
        self.com_z_r = 0.0
        self.gripper = 0.0
        self.robot3     = matrix([[0.0],[0.0],[0.0]])
        self.robot3Phi  = matrix([[0.0],[0.0],[0.0]])

        # node functions:
        rospy.init_node('kuka_gripper_controlNode')
        
        # Publish to robot
        self.kuka_joint1 = rospy.Publisher("/iiwa/PositionJointInterface_J1_controller/command", Float64, queue_size=1)
        self.kuka_joint2 = rospy.Publisher("/iiwa/PositionJointInterface_J2_controller/command", Float64, queue_size=1)
        self.kuka_joint3 = rospy.Publisher("/iiwa/PositionJointInterface_J3_controller/command", Float64, queue_size=1)
        self.kuka_joint4 = rospy.Publisher("/iiwa/PositionJointInterface_J4_controller/command", Float64, queue_size=1)
        self.kuka_joint5 = rospy.Publisher("/iiwa/PositionJointInterface_J5_controller/command", Float64, queue_size=1)
        self.kuka_joint6 = rospy.Publisher("/iiwa/PositionJointInterface_J6_controller/command", Float64, queue_size=1)
        self.kuka_joint7 = rospy.Publisher("/iiwa/PositionJointInterface_J7_controller/command", Float64, queue_size=1)
        self.kuka_gripper = rospy.Publisher("/iiwa/PositionJointInterface_gripper/command",      Float64, queue_size=1)
        self.simplecommand_x   = rospy.Publisher("/iiwa/go_x",    Float64 , queue_size=1)
        self.simplecommand_y  = rospy.Publisher("/iiwa/go_y",   Float64, queue_size=1)
        self.simplecommand_z     = rospy.Publisher("/iiwa/go_z",      Float64, queue_size=1)
        self.simplecommand_z_r = rospy.Publisher("/iiwa/go_z_r",  Float64, queue_size=1)
        self.simplecommand_gripper = rospy.Publisher("/iiwa/gripper"  ,Float64, queue_size=1)

        # Check the publishers:
        rospy.sleep(0.5)
        # sys.exit()
        
        #  Subscribe to robot  (/calibrated_fts_wrench):
        rospy.Subscriber("/iiwa/go_x",  Float64, self.cb1)
        rospy.Subscriber("/iiwa/go_y", Float64, self.cb2)
        rospy.Subscriber("/iiwa/go_z",    Float64, self.cb3)
        rospy.Subscriber("/iiwa/go_z_r",Float64, self.cb4)
        rospy.Subscriber("/iiwa/gripper"  ,Float64, self.cb5)
        rospy.Subscriber("/joint_states",JointState,self.cb_joints)

        #  wait to establish the connection completely:
        # Checking the topics:
        rospy.sleep(3)

        # Bring the robot to the home position:
        q_home = math.pi/180.0*matrix([[0.0] , [30.0] , [0] , [-60], [0], [90.0], [0.0]])
        self.send_command(q_home)
        self.kuka_gripper.publish(0.0)
        self.pMode.warning("Command KUKA to go to home position ...")
        [rotation_matrix,rot,trans]=self.FTSframe()

        self.robot3[0,0] = trans[0]
        self.robot3[1,0] = trans[1]
        self.robot3[2,0] = trans[2]
        self.robot3Phi[0,0] = rot[0]
        self.robot3Phi[1,0] = rot[1]
        self.robot3Phi[2,0] = rot[2]
        # print rot
        # print self.robot3Phi
        # print 180/math.pi*self.robot3Phi
        self.kuka_controller = controller(1/self.simRate,self.robot3,self.robot3Phi)
        

        rospy.sleep(1)
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
        # pass
        self.com_x = max(-1,min(1,data.data))

    def cb2(self, data):
        # pass
        self.com_y = max(-1,min(1,data.data))

    def cb3(self, data):
        # pass
        self.com_z = max(-1,min(1,data.data))

    def cb4(self, data):
        # pass
        self.com_z_r = max(-1,min(1,data.data))

    def cb5(self, data):
        self.gripper = max(0,min(0.8,data.data))
        # print self.gripper
        self.kuka_gripper.publish(self.gripper)

    def cb_joints(self, data):
        # Be sure to have column vector for all!!!!
        # print data.position[1]
        # print type(data.position[1])
        self.currentKukaJoints[0] = round(data.position[1], 3)
        self.currentKukaJoints[1] = round(data.position[2], 3)
        self.currentKukaJoints[2] = round(data.position[3], 3)
        self.currentKukaJoints[3] = round(data.position[4], 3)
        self.currentKukaJoints[4] = round(data.position[5], 3)
        self.currentKukaJoints[5] = round(data.position[6], 3)
        self.currentKukaJoints[6] = round(data.position[7], 3)
        self.currentGripper = data.position[0]

# spin --------------------------------------------------------------
#--------------------------------------------------------------------
    def spin(self):
        while (not rospy.is_shutdown()) and (self.simTime < 100):
            
            # # to check the running cycle time
            # if (time.time()-self.simTimeCheck) > (1/self.simRate + 0.001):
            #     self.pMode.error('Cycle time pass ' + str(self.simRate) + ' Hz! -> '+ str(time.time()-self.simTimeCheck))
            # self.simTimeCheck = time.time()
            # self.pMode.error("--------------------------------------")
            
            self.readcommands()

            # self.v_command = matrix([[0.0],[0.0],[-0.2],[0.0],[0.0],[0.0]])
            q_new = self.kuka_controller.command_position(self.currentKukaJoints,self.v_command,1/self.simRate)
            [rotation_matrix,rot,trans]=self.FTSframe()
            robotDp3 = matrix([[0],[0],[0]])
            robotDphi3 = matrix([[0],[0],[0]])
            self.robot3[0,0] = trans[0]
            self.robot3[1,0] = trans[1]
            self.robot3[2,0] = trans[2]
            self.robot3Phi[0,0] = rot[0]
            self.robot3Phi[1,0] = rot[1]
            self.robot3Phi[2,0] = rot[2]
            # print 180/math.pi*self.robot3Phi
            # print rot
            # q_new = self.kuka_controller.command_position_fromforce(self.currentKukaJoints,self.v_command,1/self.simRate,self.robot3,self.robot3Phi,robotDp3,robotDphi3)
            # q_new = matrix([[0.0 , 0.7 , 0.3 , 0.1, 0.1, 0.1, 0.0]])
            
            # self.pMode.warning(q_new)
            self.send_command(q_new)

            # To run with the specified rate
            self.rate.sleep()
            
            



#--------------------------------------------------------------------
#--------------------------------------------------------------------
    def send_command(self,q_new):  
        # self.pMode.warning(q_new[0,0])
        # print q_new
        # print q_new[1,0]
        self.kuka_joint1.publish(q_new[0,0])
        self.kuka_joint2.publish(q_new[1,0])
        self.kuka_joint3.publish(q_new[2,0])
        self.kuka_joint4.publish(q_new[3,0])
        self.kuka_joint5.publish(q_new[4,0])
        self.kuka_joint6.publish(q_new[5,0])
        self.kuka_joint7.publish(q_new[6,0])

    def gripper_close(self):
        self.kuka_gripper.publish(0.8)

    def gripper_open(self):
        self.kuka_gripper.publish(0.0)
        
    def gripper_move(self,width):
        self.kuka_gripper.publish(width)

    def readcommands(self):
        transMove = 0.5
        transRot = 0.1

        self.v_command[0,0] = self.readcommands_sub1(self.com_x,transMove)
        self.v_command[1,0] = self.readcommands_sub1(self.com_y,transMove)
        self.v_command[2,0] = self.readcommands_sub1(self.com_z,transMove)
        self.v_command[5,0] = self.readcommands_sub1(self.com_z_r,transRot)
        
        # print self.v_command
        self.com_x = 0.0
        self.com_y = 0.0
        self.com_z = 0.0
        self.com_z_r = 0.0


    def readcommands_sub1(self,direction,value):
        v=0
        if direction > 0.5:
            v=value
        else:
            if direction < -0.5:
                v=-value
            else:
                v=0
        return v
        
        
#--------------------------------------------------------------------
#--------------------------------------------------------------------
    def FTSframe(self):  
        try:			
            # /fts_link , /tool0 , /optoforce_frame
            forceSensorFrame = 'ee_link'
            self.listener.waitForTransform('/iiwa_link_0', forceSensorFrame, rospy.Time(0),rospy.Duration(2))
            (trans,rot) = self.listener.lookupTransform('/iiwa_link_0', forceSensorFrame, rospy.Time(0))
            
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
            
            return [R_final, rot, trans]
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
        kuka_gripper_controlNode()

        # Just keep the node alive!
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
