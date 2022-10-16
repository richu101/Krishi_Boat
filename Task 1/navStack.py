#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

range_max = 1
# --------------------------

kp = .1
ki = 0
kd = 0

sumoferror = 0
# --------------------------
velocity = .5

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
def PID(error):
    global sumoferror
    sumoferror += error 
    x = kp*error+ki*sumoferror
    return x

def pos1error(reg):
    e = 5
    if(reg == [1,0,0,0,0]):e=-4
    if(reg == [0,1,0,0,0]):e=-2
    if(reg == [0,0,1,0,0]):e=0
    if(reg == [0,0,0,1,0]):e=2
    if(reg == [0,0,0,0,1]):e=4
    if(reg == [1,1,0,0,0]):e=-3
    if(reg == [0,1,1,0,0]):e=-1
    if(reg == [0,0,1,1,0]):e=1
    if(reg == [0,0,0,1,1]):e=3
    if(reg == [1,1,1,0,0]):e=-2.5
    if(reg == [0,0,1,1,1]):e=2.5
    return e
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
    state = 0
    pos = 0
    while not rospy.is_shutdown():
        
        intlist = toint(regions)        
        print("Bright :",regions['bright'])
        print("Fright :",regions['fright'])
        print("Forward :",regions['front'])
        print("Fleft :",regions['fleft'])
        print("Bleft :",regions['bleft'])

        if(intlist == [1,1,1,1,1] and state == 0):  # Starting phase where all the sensor data = 1    
            pos += 1       # Keeping a pos value to know the position of the rover
            state = 1

        if(pos == 1):
            velocity_msg.linear.x = velocity        # At state 1 the rover starts moving forward 
        
        elif(pos == 2):
            velocity_msg.linear.x = 0        # At state 2 the rover stops at the ens and wait for the turning     
        
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
        print("Pos : ",pos)
        pub.publish(velocity_msg)
        print("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep()

def laser_callback(msg):
    global regions,range_max
    regions = {
        'bright':   min(min(msg.ranges[0:144]), range_max)   ,
        'fright':   min(min(msg.ranges[144:288]), range_max) ,
        'front':    min(min(msg.ranges[288:432]), range_max) ,
        'fleft':    min(min(msg.ranges[432:576]), range_max) ,
        'bleft':    min(min(msg.ranges[576:720]), range_max) ,
    }


if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass    