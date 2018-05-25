#!/usr/bin/env python
import rospy
import roslib
import time
from numpy import matrix
from numpy import linalg
import numpy.matlib
import math
import numpy as np



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
        return j2

    def command_position(self,q_old,command_velocity,dt):
        dq = np.linalg.pinv(self.Jacobian_KUKA7(q_old)) * command_velocity
        q_new = dq*dt + q_old
        return q_new

    def command_position_fromforce(self,q_old,command_force,dt,robotP3,robotPhi3,robotDp3,robotDphi3):
        fExt3 = command_force[0:3]
        tExt3 = command_force[3:6]
        self.Ts = dt
        # fExt3 = np.multiply( np.sign(fExt3) , np.maximum( np.matlib.zeros((3, 1)) , np.absolute(fExt3) - self.forceDeadZone * np.matlib.ones((3, 1)) ) )  
        # tExt3 = np.multiply( np.sign(tExt3) , np.maximum( np.matlib.zeros((3, 1)) , np.absolute(tExt3) - self.torqueDeadZone * np.matlib.ones((3, 1)) ) )  
            
        # for(int i=0;i<6;i++)
        #     {
        #         Fee(i) = (Fee_sign(i)) * fmax(0.0,fabs(Fee(i)) - cmd_args.Fee_dead_zone(i));
        #     }
        print fExt3
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

        self.Vd6[0:3,0] = ( self.dEp - self.Kp_click * (robotP3 - self.Ep) )
        self.Vd6[3:6,0] = ( self.dEo - self.Ko_click * (robotPhi3 - self.Eo) ) 

        final_q = self.command_position(q_old,self.Vd6,dt)
        # print self.Ep
        return final_q

# Add keyboard shortkeys!!!!! to control the robot

# Here is the main entry point
if __name__ == '__main__':
    # Init
    # rospy.init_node('filterForce')
    con1 = controller()
    q = matrix([[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]])
    print q
    t1 = time.time()
    jacobian = con1.Jacobian_KUKA7(q)
    print jacobian
    vv = matrix([[0.0],[0.0],[0.0],[0.0],[0.0],[1.0]])
    temp1 = con1.command_position(q,vv,0.1)
    print temp1

    temp2 = con1.command_position_fromforce(q,vv,0.1,[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]],[[0.0],[0.0],[0.0]])
    print temp2 

    t2 = time.time()
    print (t2-t1)
    
