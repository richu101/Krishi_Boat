#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

range_max = 1

regions = {
        'bright':   0,
        'fright':   0,
        'front':    0,
        'fleft':   0,
        'bleft':   0,
    }

def control_loop():
    rospy.init_node('ebot_controller')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    
    rate = rospy.Rate(10) 

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

    while not rospy.is_shutdown():
        
                
        print("Bright :",regions['bright'])
        print("Fright :",regions['fright'])
        print("Forward :",regions['front'])
        print("Fleft :",regions['fleft'])
        print("Bleft :",regions['bleft'])

        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 0
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
        'bleft':    min(min(msg.ranges[576:720]), range_max) 
    }


if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass    