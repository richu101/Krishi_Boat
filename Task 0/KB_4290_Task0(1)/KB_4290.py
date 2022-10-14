#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Krishi Bot (KB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of Krishi Bot (KB) Theme (eYRC 2022-23).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:			task_0.py
# Functions:		
# 					[ Comma separated list of functions in this file ]
# Nodes:		    Add your publishing and subscribing node


####################### IMPORT MODULES #######################
import sys
import traceback
import rospy
from geometry_msgs.msg import Twist
##############################################################

################# ADD GLOBAL VARIABLES HERE #################



##############################################################

################# ADD UTILITY FUNCTIONS HERE #################



##############################################################

def callback(line_val,ang_val):
    count=0
    rospy.init_node('turtlemove',anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
    rate = rospy.Rate(10)
    vel = Twist()
    while count < 17.5:
        print("TurtleBot is: Moving in circle")
        count+=0.5
        vel.linear.x = line_val
        vel.linear.y = 0
        vel.linear.z = 0

        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = ang_val
        pub.publish(vel)
        rate.sleep()

    #Straight line
    print("TurtleBot is: Moving in straight line")
    vel.linear.x = 0
    vel.linear.y = 4
    vel.linear.z = 0

    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = 0
    pub.publish(vel)
    


    """
	Purpose:
	---
	This function should be used as a callback. Refer Example #1: Pub-Sub with Custom Message in the Learning Resources Section of the Learning Resources.
    You can write your logic here.
    NOTE: Radius value should be 1. Refer expected output in document and make sure that the tu rtle traces "same" path.

	Input Arguments:
	---
        `data`  : []
            data received by the call back function

	Returns:
	---
        May vary depending on your logic.
	
	Example call:
	---
        Depends on the usage of the function.
	"""
    

def main():
    callback(2,1)
    """
    Purpose:
	---
	This function will be called by the default main function given below.
    You can write your logic here.

	Input Arguments:
	---
        None

	Returns:
	---
        None
	
	Example call:
	---
        main()
	"""    

######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS PART #########    
if __name__ == "__main__":
    try:
        print("------------------------------------------")
        print("         Python Script Started!!          ")
        print("------------------------------------------")
        main()
        
    except:
        print("------------------------------------------")
        traceback.print_exc(file=sys.stdout)
        print("------------------------------------------")
        sys.exit()

    finally:
        print("------------------------------------------")
        print("    Python Script Executed Successfully   ")
        print("------------------------------------------")
