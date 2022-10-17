#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
range_max = 1
# --------------------------

kp = .8
ki = .03
kd = 0

kpbl = 2.9 
kibl = .25

sumoferror = 0
sumoferrorbl = 0
# --------------------------
velocity = .95
turningvelocity = .3

regions = {
        'bright':   0,
        'fright':   0,
        'front':    0,
        'fleft':   0,
        'bleft':   0,
    }
def toint(reg):
    val_list = list(reg.values())
    A = [int(val_list) for val_list in val_list]
    print("Int List : ", A)
    return(A)
def PID(presentvalue):
    global sumoferror
    sumoferror += presentvalue 
    x = (kp*presentvalue)+(ki*sumoferror)
    print("Pos 1 PID : ",x)
    return x
def laser_callback(msg):
    global regions,range_max
    regions = {
        'bright':   min(min(msg.ranges[0:144]), range_max)   ,
        'fright':   min(min(msg.ranges[144:288]), range_max) ,
        'front':    min(min(msg.ranges[288:432]), range_max) ,
        'fleft':    min(min(msg.ranges[432:576]), range_max) ,
        'bleft':    min(min(msg.ranges[576:720]), range_max) ,
    }

def pidbl(present,kpbl,kibl,s):
    global sumoferrorbl
    setpoint = s
    err = present - setpoint
    sumoferrorbl += err 
    x = kpbl*(err)+kibl*(sumoferrorbl)
    print("PID Back left ",x)
    return x    

def pos1error(reg):
    e = 5
    if(reg == [1,0,0,0,0]):e=-4 +1
    if(reg == [0,1,0,0,0]):e=-2 +1
    if(reg == [0,0,1,0,0]):e=0
    if(reg == [0,0,0,1,0]):e=2 -1
    if(reg == [0,0,0,0,1]):e=4 -1
    if(reg == [1,1,0,0,0]):e=-3 +1
    if(reg == [0,1,1,0,0]):e=-1 +.5
    if(reg == [0,0,1,1,0]):e=1 -.5
    if(reg == [0,0,0,1,1]):e=3 -1
    if(reg == [1,1,1,0,0]):e=-2.5 +1
    if(reg == [1,0,1,0,1]):e=-0.01
    if(reg == [0,1,1,1,0]):e=-0.01
    if(reg == [0,0,1,1,1]):e=2.5 -1
    if(reg == [1,1,1,1,0]):e=-0.08
    return e

def pos3error(reg):
    e = 5
    if(reg == [1,1,1,1,0]):e=-4 +1
    if(reg == [0,1,0,0,0]):e=-2 +1
    if(reg == [1,1,1,1,1]):e=0
    if(reg == [0,0,0,1,0]):e=2 -1
    if(reg == [0,0,0,0,1]):e=4 -1
    if(reg == [1,1,0,0,0]):e=-3 +1
    if(reg == [0,1,1,0,0]):e=-1 +.5
    if(reg == [0,0,1,1,0]):e=1 -.5
    if(reg == [0,0,0,1,1]):e=3 -1
    if(reg == [1,1,1,0,0]):e=-2.5 +1
    if(reg == [1,0,1,0,1]):e=-0.01
    if(reg == [0,1,1,1,0]):e=-0.01
    if(reg == [0,0,1,1,1]):e=2.5 -1
    if(reg == [1,1,1,1,0]):e=-0.08
    return e    
def pos4pid(present,s):
    global sumoferror,kibl,kpbl
    setpoint = s
    err = present - setpoint
    sumoferror += err 
    x = kpbl*(err)+kibl*(sumoferror)
    print("Pos 2 PID ",x)
    return x

def control_loop():
    rospy.init_node('ebot_controller')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    
    rate = rospy.Rate(10) 

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)
    intlist = 0
    distanceList = 0
    state = 0
    pos = 0
    global sumoferror,sumoferrorbl
    while not rospy.is_shutdown():
        
        intlist = toint(regions)   
        distanceList = list(regions.values())
        print("Bright :",regions['bright'])
        print("Fright :",regions['fright'])
        print("Forward :",regions['front'])
        print("Fleft :",regions['fleft'])
        print("Bleft :",regions['bleft'])

        if(intlist == [1,1,1,1,1] and state == 0):  # Starting phase where all the sensor data = 1    
            pos += 1       # Keeping a pos value to know the position of the rover
            state = 1
            sumoferror = 0
            sumoferrorbl = 0


        if(pos == 1 ):
            velocity_msg.linear.x = velocity        # At state 1 the rover starts moving forward 
        
        elif(pos == 2 and intlist == [1,1,1,1,1]):
            velocity_msg.linear.x = turningvelocity         # At state 2 the rover slows at the end and wait for the turning     
            sumoferror = 0
            velocity_msg.angular.z = pos4pid(distanceList[3],.6)

        elif(pos == 2 and distanceList[2]>.5):
            if(distanceList[4]>.5):
                if(intlist == [1,1,1,0,0] ):
                    velocity_msg.linear.x = velocity
                    velocity_msg.angular.z = 0
                    state=0
                else:
                    velocity_msg.linear.x = turningvelocity         # At state 2 the rover slows at the end and wait for the turning     
                    sumoferror = 0
                    velocity_msg.angular.z = pos4pid(distanceList[3],.6)
                    print("Pos 2 00000",velocity_msg.angular.z)
            elif(distanceList[4]<.5):
                velocity_msg.angular.z = -turningvelocity

        # else:
        #     velocity_msg.linear.x = velocity  

        if(intlist != [1,1,1,1,1] and pos == 1):
            # This case the rover have to move forward till all the sensor value reach 1
            errorv = pos1error(intlist)
            if(errorv != 5): 
                w = PID(errorv) 
                velocity_msg.linear.x = velocity
                velocity_msg.angular.z = w
            else:
                velocity_msg.linear.x =0
            state = 0
        if(pos ==3):
            if(intlist == [1,1,1,1,1]):
                velocity_msg.linear.x = velocity 
                velocity_msg.angular.z = 0 
                

            else:
                
                if(distanceList[4]>.4):
                    if(intlist == [0,0,1,0,0] ):
                        velocity_msg.linear.x = velocity
                        velocity_msg.angular.z = 0
                        state=0
                    else:
                        velocity_msg.linear.x = turningvelocity         # At state 2 the rover slows at the end and wait for the turning     
                        sumoferror = 0
                        velocity_msg.angular.z = pos4pid(distanceList[3],.6)
                        print("Pos 2 00000",velocity_msg.angular.z)
                elif(distanceList[4]<.4):
                    velocity_msg.angular.z = -turningvelocity
        if(pos == 4):
            velocity_msg.linear.x = 0
            velocity_msg.angular.z = 0

        print("Pos : ",pos)
        pub.publish(velocity_msg)
        print("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep()




if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass 







