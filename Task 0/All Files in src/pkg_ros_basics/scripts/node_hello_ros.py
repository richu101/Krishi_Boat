#!/usr/bin/env python3

import rospy


def main():    
    
    # 1. Make the script a ROS Node.
    rospy.init_node('node_hello_ros', anonymous=True)
    param_config_my = rospy.get_param('details')
    print(param_config_my['name']['first'])
    # 2. Print info on the console.
    rospy.loginfo("Hello World!")
    
    # 3. Keep the node alive till it is killed by the user.
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
