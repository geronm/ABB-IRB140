'''
Author: alexc89@mit.edu
IRB140LCMWrapper
This is a script connecting OPEN ABB Driver with LCM.  It publishes IRB140's joint position in IRB140Pos LCM Channel and listens IRB140Input to control IRB140's joint.
Please note that the unit of the joint state and command is dependent on the setting on the IRB140, this script DOES NOT translate the command into a specific unit like radian.

'''

import lcm
import time
import abb
from ctypes import *
import threading 
#Import LCM Messages
from lcmtypes import *

#Message Conversion
def convertABBstate(joint_pos,joint_vel,cartesian):
    msg = abb_irb140state()
    msg.utime=  time.time()*1000000
   
    msg.joints = abb_irb140joints()
    msg.cartesian = abb_irb140cartesian()
    msg.joints.utime = msg.utime
    msg.cartesian.utime= msg.utime
    msg.joints.pos = joint_pos
    msg.joints.vel = joint_vel
    msg.cartesian.pos = cartesian[0]
    msg.cartesian.quat = cartesian[1]
    return msg

def convertACH_Command(msg):
    return msg.Joints

class abbIRB140LCMWrapper:
    
    def __init__(self):
        self.robot = abb.Robot(); #Robot Connection to openABB, input Robot's IP if needed.
        self.joint_cmd_lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
        self.joint_cmd_subscription = self.joint_cmd_lc.subscribe("IRB140Input",self.command_handler)
        self.joint_plan_lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
        self.joint_plan_subscription = self.joint_plan_lc.subscribe("IRB140JOINTPLAN",self.plan_handler)
        

    def plan_handler(self,channel,data):
        msg = abb_irb140joint_plan.decode(data)
        for i in range(msg.n_cmd_times):
            self.robot.addJointPosBuffer(msg.joint_cmd[i].joints)
        self.robot.executeJointPosBuffer()
        self.robot.clearJointPosBuffer()
        
    def command_handler(self,channel,data):
        msg = abb_irb140joints.decode(data)
	jointCommand = msg.pos
        self.robot.setJoints(jointCommand)

    def broadcast_state(self):
        jointPos = self.robot.getJoints()
	cartesian = self.robot.getCartesian()
        #ABB drive to LCM conversion
	msg = convertABBstate(jointPos,[0,0,0,0,0,0],cartesian)
        self.joint_cmd_lc.publish("IRB140STATE", msg.encode())

    def mainLoop(self,freq):
        pauseDelay = 1.0/freq #In Seconds.
        t = 1
        def broadcastLoop():
            while True:
                self.broadcast_state()
                time.sleep(pauseDelay)
        try:
            t = threading.Thread(target=broadcastLoop)
            t.daemon = True
            t.start()
            while True:
                time.sleep(pauseDelay)
                self.joint_cmd_lc.handle()
        except KeyboardInterrupt:
            pass

if __name__ == "__main__":
    wrapper = abbIRB140LCMWrapper()
    print "IRB140LCMWrapper finish initialization, Begin transmission to LCM"
    wrapper.mainLoop(10) #Hertz
    print "IRB140LCMWrapper terminated successfully."
