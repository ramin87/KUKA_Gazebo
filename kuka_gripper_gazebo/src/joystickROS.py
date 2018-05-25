#!/usr/bin/env python
 
import rospy
import os
import pyxhook
import time
from std_msgs.msg import String, Float64

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

class joystick():
    def __init__(self):
        # create a hook manager object
        self.new_hook = pyxhook.HookManager()
        self.new_hook.KeyDown = self.OnKeyPress
        # set the hook
        self.new_hook.HookKeyboard()
        print 'test'

        self.pMode = color()
        # Node parameters:
        self.simRate = 125.0
        self.simTimeCheck = time.time()

        # node functions:
        rospy.init_node('joystick_node')

        self.simplecommand_x   = rospy.Publisher("/iiwa/go_x",    Float64 , queue_size=1)
        self.simplecommand_y  = rospy.Publisher("/iiwa/go_y",   Float64, queue_size=1)
        self.simplecommand_z     = rospy.Publisher("/iiwa/go_z",      Float64, queue_size=1)
        self.simplecommand_z_r = rospy.Publisher("/iiwa/go_z_r",  Float64, queue_size=1)
        self.simplecommand_gripper = rospy.Publisher("/iiwa/gripper"  ,Float64, queue_size=1)

        # Check the publishers:
        rospy.sleep(0.2)
        # sys.exit()
        
        try:
            print "started!"
            self.pMode.error("------------------------------------------------------------------")
            self.pMode.error("Up Down Left Right q a -> for moving along x y z\nz x for rotating around z\n 1 to close the gripper 2 to open the gripper\n")
            self.pMode.green("press \'Escape\' to exit and ctrl+C")
            self.pMode.error("------------------------------------------------------------------")
            self.new_hook.start()         # start the hook
        except KeyboardInterrupt:
            # User cancelled from command line.
            pass


    #creating key pressing event and saving it into log file
    def OnKeyPress(self,event):
        
        # print '\n key is pressed! \n'
        print ''
        # print self.simTimeCheck-time.time()
        self.simTimeCheck = time.time()
        # print event.Key
        self.lastkey = event.Key
        if event.Key=='Up':
            self.simplecommand_x.publish(1.0)
        if event.Key=='Down':
            self.simplecommand_x.publish(-1.0)
        if event.Key=='Left':
            self.simplecommand_y.publish(1.0)
        if event.Key=='Right':
            self.simplecommand_y.publish(-1.0)
        if event.Key=='q':
            self.simplecommand_z.publish(1.0)
        if event.Key=='a':
            self.simplecommand_z.publish(-1.0)
        if event.Key=='z':
            self.simplecommand_z_r.publish(1.0)
        if event.Key=='x':
            self.simplecommand_z_r.publish(-1.0)
        if event.Key=='1':
            self.simplecommand_gripper.publish(0.4)
        if event.Key=='2':
            self.simplecommand_gripper.publish(0)
        

        if event.Key=='Escape':
            self.new_hook.cancel()
        pass
        # print event.Key


# Here is the main entry point
if __name__ == '__main__':
    joystick1 = joystick()



