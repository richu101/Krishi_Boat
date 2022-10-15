#!/usr/bin/env python3

from turtle import position
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
range_max = 1
# PID Constants
# ----------------------
kp = .3 
ki = 0.01
kd = 0
# ----------------------
errorsum = 0
regions = {
        'bright':   0,
        'fright':   0,
        'front':    0,
        'fleft':   0,
        'bleft':   0,
    }
e = 0    

def laser_callback(msg):
    global regions
    regions = {            
        'bright':   min(min(msg.ranges[0:144]), range_max) ,
        'fright':   min(min(msg.ranges[144:288]), range_max) ,
        'front':    min(min(msg.ranges[288:432]), range_max) ,
        'fleft':    min(min(msg.ranges[432:576]), range_max) ,
        'bleft':    min(min(msg.ranges[576:720]), range_max)         
    }

def boolList(reg):
    global e
    val_list = list(reg.values())
    A = [int(val_list) for val_list in val_list]
    # val_list=[2,3,5,6,4]        example value
    if A == [1,1,1,1,1] : e = 5
    
    if sum(A)==1:
        pos = A.index(max(A))
        bool_list = [0,0,0,0,0] 
        bool_list[pos] = 1 
        if A == [1,1,1,1,1] : e = 5
        elif bool_list[0] == 1 : e = 4
        elif bool_list[1] == 1 : e = 2
        elif bool_list[2] == 1 : e = 0 
        elif bool_list[3] == 1 : e = -2
        elif bool_list[4] == 1 : e = -4
        print(bool_list)
    elif sum(A) == 2:
        print(A)
        if A == [0,0,0,1,1]: e = 4
        if A == [1,1,0,0,0]: e = -4
        if A == [0,1,1,0,0]: e = -1
        if A == [0,0,1,1,0]: e = 0
        if A == [0,1,0,1,0]: print("Stop")
    elif sum(A)==3:
        print(A)
        if A == [1,1,1,0,0]: e = 4
        if A == [0,0,1,1,1]: e = -4
        if A == [0,1,1,1,0]: e = 1
    elif sum(A)==4:
        print(A)
        if A == [1,1,1,1,0]: e = -2
        if A == [0,1,1,1,1]: e = 2
    print("Est Eroor Value : ",e)
    return e

def pid(eval):
    global kp,ki,kd,errorsum
    errorsum = errorsum+eval
    x =  (kp*eval)+(ki*errorsum)
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
    state = 0
    while not rospy.is_shutdown():
        global regions
        #
        # Your algorithm to navigate
        #
        bl = boolList(regions) 
        global e
        print("Bright :",regions['bright'])
        print("Fright :",regions['fright'])
        print("Forward :",regions['front'])
        print("Fleft :",regions['fleft'])
        print("Bleft :",regions['bleft'])
        velocity_msg.linear.x = .2
        if(bl==5):state+=1
        if bl != 5 and state == 1:
            er = pid(bl)
            velocity_msg.angular.z = er
            print(er)

        pub.publish(velocity_msg)
        print("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep()



if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
