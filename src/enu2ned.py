#!/usr/bin/env python3

import rospy
import math
import numpy as np

from geometry_msgs.msg import TransformStamped, PoseStamped

class ENU2NED():

    def __init__(self):
        # Set up subscriber and publisher
        self.enu_sub = rospy.Subscriber('vicon/CDC_drone/CDC_drone', TransformStamped, self.enuCallback, queue_size=10)
        self.ned_pub = rospy.Publisher('vicon_ned', PoseStamped, queue_size=10)

        # # For testing
        # self.ned_euler_pub = rospy.Publisher('euler/ned', PoseStamped, queue_size=10)
        # self.enu_euler_pub = rospy.Publisher('euler/enu', PoseStamped, queue_size=10)

        # Initialize ned_msg
        self.ned_msg = PoseStamped()

        # Set rotation (w, x, y, z)
        self.rotation = [0, 0.707, -0.707, 0]

        while not rospy.is_shutdown():
        # wait for new messages and call the callback when they arrive
            rospy.spin()

    def quaternion_multiply(self, quaternion0, quaternion1): # w, x, y, z; Looks Good
        a0, b0, c0, d0 = quaternion0
        a1, b1, c1, d1 = quaternion1
        return np.array([a0*a1 - b0*b1 - c0*c1 - d0*d1,
                            b0*a1 + a0*b1 + c0*d1 - d0*c1,
                            a0*c1 - b0*d1 + c0*a1 + d0*b1,
                            a0*d1 + b0*c1 - c0*b1 + d0*a1])

    def enuCallback(self, enu_msg):
        q_x = enu_msg.transform.rotation.x
        q_y = enu_msg.transform.rotation.y
        q_z = enu_msg.transform.rotation.z
        q_w = enu_msg.transform.rotation.w

        [w, x, y, z] = self.quaternion_multiply(self.rotation, [q_w, q_x, q_y, q_z])

        # Create ned_msg
        self.ned_msg.header = enu_msg.header
        # Change ENU positions to NED
        self.ned_msg.pose.position.x = enu_msg.transform.translation.y
        self.ned_msg.pose.position.y = enu_msg.transform.translation.x
        self.ned_msg.pose.position.z = -enu_msg.transform.translation.z
        # Use rotated quaternion
        self.ned_msg.pose.orientation.x = x
        self.ned_msg.pose.orientation.y = y
        self.ned_msg.pose.orientation.z = z
        self.ned_msg.pose.orientation.w = w


        # # Convert to Euler
        # [enu_x, enu_y, enu_z] = self.Quaternion2Euler(q_w, q_x, q_y, q_z)
        
        # # Change from ENU to NED
        # ned_x = enu_y
        # ned_y = enu_x
        # ned_z = -enu_z

        # # ### For testing: print euler angles
        # # euler_msg = PoseStamped()
        # # euler_msg.pose.orientation.x = ned_x
        # # euler_msg.pose.orientation.y = ned_y
        # # euler_msg.pose.orientation.z = ned_z
        # # self.ned_euler_pub.publish(euler_msg)

        # # Convert to Quaternion
        # [w, x, y, z] = self.Euler2Quaternion(ned_x, ned_y, ned_z)
        # q_ned = [x, y, z, w]

        # ### Just switch axes? Test
        # # q_ned = [q_y, q_x, -q_z, q_w]
        
        # # Create ned_msg
        # self.ned_msg.header = enu_msg.header
        # # Change ENU positions to NED
        # self.ned_msg.pose.position.x = enu_msg.transform.translation.y
        # self.ned_msg.pose.position.y = enu_msg.transform.translation.x
        # self.ned_msg.pose.position.z = -enu_msg.transform.translation.z
        # # Use rotated quaternion
        # self.ned_msg.pose.orientation.x = q_ned[0]
        # self.ned_msg.pose.orientation.y = q_ned[1]
        # self.ned_msg.pose.orientation.z = q_ned[2]
        # self.ned_msg.pose.orientation.w = q_ned[3]

        # Publish message
        self.ned_pub.publish(self.ned_msg)

    # def printEuler(self, quat_array, topic):
    #     # For Testing
    #     x = quat_array[0]
    #     y = quat_array[1]
    #     z = quat_array[2]
    #     w = quat_array[3]

    #     euler = self.Quaternion2Euler(w, x, y, z)

    #     euler_msg = PoseStamped()
    #     euler_msg.pose.orientation.x = euler[0]
    #     euler_msg.pose.orientation.y = euler[1]
    #     euler_msg.pose.orientation.z = euler[2]

    #     if topic == 'ned':
    #         self.ned_euler_pub.publish(euler_msg)
    #     elif topic == 'enu':
    #         self.enu_euler_pub.publish(euler_msg)

    def Quaternion2Euler(self, e0, e1, e2, e3):

        phi = np.arctan2(2*(e0*e1+e2*e3), (e0**2+e3**2-e1**2-e2**2))
        theta = np.arcsin(2*(e0*e2-e1*e3))
        psi = np.arctan2(2*(e0*e3+e1*e2), (e0**2+e1**2-e2**2-e3**2))

        return [phi, theta, psi]

    def Euler2Quaternion(self, phi, theta, psi):
        e0 = math.cos(psi/2)*math.cos(theta/2)*math.cos(phi/2) + math.sin(psi/2)*math.sin(theta/2)*math.sin(phi/2)
        e1 = math.cos(psi/2)*math.cos(theta/2)*math.sin(phi/2) - math.sin(psi/2)*math.sin(theta/2)*math.cos(phi/2)
        e2 = math.cos(psi/2)*math.sin(theta/2)*math.cos(phi/2) + math.sin(psi/2)*math.cos(theta/2)*math.sin(phi/2)
        e3 = math.sin(psi/2)*math.cos(theta/2)*math.cos(phi/2) - math.cos(psi/2)*math.sin(theta/2)*math.sin(phi/2)

        return [e0, e1, e2, e3]


if __name__ == '__main__':
    rospy.init_node('enu2ned', anonymous=True)
    try:
        enu2ned = ENU2NED()
    except:
        rospy.ROSInterruptException
    pass