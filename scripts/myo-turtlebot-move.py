#!/usr/bin/env python

## Simple myo demo that listens to std_msgs/UInt8 poses published 
## to the 'myo_gest' topic

import rospy
from std_msgs.msg import UInt8, String
from geometry_msgs.msg import Twist, Vector3
from ros_myo.msg import MyoArm

if __name__ == '__main__':

    global armState
    global xDirState
    armState = 0
    rospy.init_node('turtlebot_myo_move', anonymous=True)

    turtlebotPub = rospy.Publisher("directs", String, queue_size=10)
    tbPub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)

    # set the global arm states
    def setArm(data):
        global armState
        global xDirState

        armState = data.arm
        xDirState = data.xdir
        rospy.sleep(0.1)

    # Use the calibrated Myo gestures to drive the turtle
    def drive(gest):
        rospy.set_param('~guard_action', gest.data)

        if gest.data == 1:  # FIST
            turtlebotPub.publish("go back")
            tbPub.publish(Twist(Vector3(-0.2, 0, 0), Vector3(0, 0, 0)))
        elif gest.data == 2:  # WAVE_IN, RIGHT arm
            turtlebotPub.publish("go left")
            tbPub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.8)))
        elif gest.data == 2 and armState == 2:  # WAVE_IN, LEFT arm
            turtlebotPub.publish("go right")
            tbPub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, -1.0)))
        elif gest.data == 3:  # WAVE_OUT, RIGHT arm
            turtlebotPub.publish("go right")
            tbPub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.8)))
        elif gest.data == 3 and armState == 2:  # WAVE_OUT, LEFT arm
            turtlebotPub.publish("go left")
            tbPub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 1.0)))
        elif gest.data == 4:  # FINGERS_SPREAD
            turtlebotPub.publish("go forward")
            tbPub.publish(Twist(Vector3(0.2, 0, 0), Vector3(0, 0, 0)))

    rospy.Subscriber("myo_arm", MyoArm, setArm)
    rospy.Subscriber("myo_gest", UInt8, drive)
    rospy.loginfo('Please sync the Myo')

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
