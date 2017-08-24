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
from abblcm import *
from scipy import interpolate
import numpy as np

#Message Conversion
def convertSensordata(rawdata):
    msg = abb_irb140ftsensor()
    msg.utime = time.time()*1000000
    if rawdata is not None:
        msg.hand_force = rawdata[0:3]
        msg.hand_torque = rawdata[3:6]
    else:
        msg.hand_force = [float('nan'), float('nan'), float('nan')]
        msg.hand_torque = [float('nan'), float('nan'), float('nan')]
    return msg

def convertABBstate(joint_pos,joint_vel,cartesian,force_torque):
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

    msg.force_torque = convertSensordata(force_torque)
    return msg


#Interpolate and resample
def resampleJointPlanCubicSpline(joint_cmd, resample_utime_step):
    '''
      Inputs:
        joint_cmd - list of abb_irb140joints instances, utime-ordered, for traj
        resample_utime_step - time resolution at which resampling should occur. Units
            are units for abb_irb140joints.utime
      Outputs:
        returns a list of the resampled pos configurations, which are represented as
            lists of joint positions Eg: [ [1, 1, 1, 1, 1, 1] , [1.2, 1, 1, 1, 1, 1] ],
            or None or [] if joint_cmd is None or [], respectively.
    '''
    if joint_cmd is None or resample_utime_step is None or (resample_utime_step == 0):
        return None # No good answer can be given.
    if len(joint_cmd) == 0:
        return [], []
    times_old = np.array([cmd.utime for cmd in joint_cmd])
    start_time = joint_cmd[0].utime
    end_time = joint_cmd[-1].utime
    if end_time <= start_time:
        print "Invalid plan times coming in:", times_old
        return None, None

    times_new = np.arange(start_time, end_time, resample_utime_step)
    times_new = np.append(times_new, [end_time])
    print "times: ", start_time, end_time, times_new
    joint_pos_resampled = []
    joint_times_resampled = []
    try:
        joints_pos_new = []
        for joint in range(len(joint_cmd[0].pos)): #For each joint, construct spline-interpolation
            joint_pos_old = np.array([cmd.pos[joint] for cmd in joint_cmd])
            cubespline = interpolate.splrep(times_old, joint_pos_old, s=0)    
            joints_pos_new.append(interpolate.splev(times_new, cubespline, der=0))
        joint_pos_resampled = [[joints_pos_new[joint][t] \
                                  for joint in range(len(joint_cmd[0].pos))] \
                                  for t in range(len(times_new))] #Rearrange into a list of joint pos's
    except TypeError as e: # Gives an error if the data can't be spline-interpolated; try linear instead
        print "Caught TypeError in resampleJointPlanCubicSpline: ", e
        joints_pos_new = []
        for joint in range(len(joint_cmd[0].pos)): #For each joint, construct spline-interpolation
            joint_pos_old = np.array([cmd.pos[joint] for cmd in joint_cmd])
            interpf = interpolate.interp1d(times_old, joint_pos_old)
            joints_pos_new.append(interpf(times_new))
        joint_pos_resampled = [[joints_pos_new[joint][t] \
                                  for joint in range(len(joint_cmd[0].pos))] \
                                  for t in range(len(times_new))] #Rearrange into a list of joint pos's
    #Joint move times (all same, because resampled)
    joint_times_resampled = [resample_utime_step/1000000 for i in range(0,len(joint_pos_resampled) - 1)]
    return joint_pos_resampled, joint_times_resampled

def convertACH_Command(msg):
    return msg.Joints

class abbIRB140LCMWrapper:
    
    def __init__(self):
        self.robot = abb.Robot(verbose=True, speed=[40,20,20,20])# speed=[100, 50, 50, 50]) #Robot Connection to openABB, input Robot's IP if needed.
        self.robot.setJointPosBufferFTSetpoint()
        self.logger = abb.Logger(verbose=False)
        self.lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
        self.lc.subscribe("IRB140Input",self.command_handler)
        self.lc.subscribe("IRB140JOINTPLAN",self.plan_handler)
        self.lc.subscribe("IRB140JOINTCMD",self.command_handler)
        self.resample_utime_step = .05*1000*1000 # left number (ie. not 1000000) gives seconds per step

    def plan_handler(self,channel,data):
        print "receive plan"
        msg = abb_irb140joint_plan.decode(data)

        plan_pos, plan_times = resampleJointPlanCubicSpline(msg.joint_cmd, self.resample_utime_step)

        if plan_pos and plan_times:
            # Add pos to buffer
            self.robot.clearJointPosBuffer()
            self.robot.addJointPosBuffer(plan_pos[0])
            for i in range(1, len(plan_pos)):
                # Set move time before calling addJointPosBuffer
                #self.robot.setMoveTime(plan_times[i-1])
                # Add pos to buffer
                self.robot.addJointPosBuffer(plan_pos[i])
            self.robot.executeJointPosBufferWithFT()
#            self.robot.executeJointPosBuffer()
            self.robot.clearJointPosBuffer()

    def command_handler(self,channel,data):
        print "receive command"
        msg = abb_irb140joints.decode(data)
        jointCommand = msg.pos
        self.robot.setJoints(jointCommand)

    def broadcast_state(self):
        jointPos = self.logger.getJoints()
        cartesian = self.logger.getCartesian()
        forceTorque = self.logger.getForceSensors()
        #ABB drive to LCM conversion
        if (jointPos and cartesian):
            msg = convertABBstate(jointPos,[0,0,0,0,0,0],cartesian, forceTorque)
            self.lc.publish("IRB140STATE", msg.encode())

    def mainLoop(self,freq):
        pauseDelay = 1.0/freq #In Seconds.
        t = 1
        stop = False
        def broadcastLoop():
            while not stop:
                self.broadcast_state()
                time.sleep(pauseDelay)
        try:
            t = threading.Thread(target=broadcastLoop)
            t.daemon = True
            t.start()
            while True:
                time.sleep(pauseDelay)
                self.lc.handle()
        except KeyboardInterrupt:
            stop = True
            t.join()
            self.logger.stop()
            self.robot.close()

if __name__ == "__main__":
    wrapper = abbIRB140LCMWrapper()
    print "IRB140LCMWrapper finish initialization, Begin transmission to LCM"
    wrapper.mainLoop(50) #Hertz
    print "IRB140LCMWrapper terminated successfully."
