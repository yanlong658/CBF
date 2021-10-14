#! /usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from mavros_msgs.msg import State
from geometry_msgs.msg import Quaternion
import math
import cv2
"""
implement velocity controller

TODO:
    0. Set "offboard" mode and arm the vehicle. Refer to example code on hackmd
    1. Transform body frame velocity command into world frame velocity command
    2. Publish world frame velocity command to topic[/mavros/setpoint_velocity/cmd_vel]
    3. Instead of publishing command randomly, implement a keyboard commander (optional)

def body_to_world(vel_cmd):
    return wf_vel_cmd[0], wf_vel_cmd[1], wf_vel_cmd[2], wf_vel_cmd[3]
"""
def keyboard_control():
    vel=np.array([0,0,0,0])
    key = cv2.waitKey(1)
    if key == 87 or key == 119:
        vel[0]=1	
    elif key == 83 or key == 115:
        vel[0]=-1
    elif key == 65 or key == 97:
        vel[1]=1
    elif key == 68 or key == 100:
        vel[1]=-1
    cv2.imshow("keyboard", np.zeros((32, 32)))
    return vel

def quaternion_to_euler_angle(x, y, z,w):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.atan2(t3, t4)

    return X, Y, Z


class Controller:
    def __init__(self):
        self.current_odom=Odometry()
        self.current_state = State()
        self.odom_sub = rospy.Subscriber("/mavros/local_position/odom", Odometry, self.odom_cb)
        self.local_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)

        self.rate=rospy.Rate(20)
        self.TwistStamped=TwistStamped()




    def body_to_world(self,vel_cmd):
        q_x=self.current_odom.pose.pose.orientation.x
        q_y=self.current_odom.pose.pose.orientation.y
        q_z=self.current_odom.pose.pose.orientation.z
        q_w=self.current_odom.pose.pose.orientation.w
        row,pitch,yaw=quaternion_to_euler_angle(q_x, q_y, q_z,q_w)
        Rz=np.array([[math.cos(yaw),    -math.sin(yaw), 0,   0],

                     [math.sin(yaw),    math.cos(yaw),  0,   0],

                     [0,                     0,         1,   0],
                     [0,                     0,         0,   1]

                      ])
        wf_vel_cmd=np.dot(Rz,vel_cmd)



        return wf_vel_cmd[0], wf_vel_cmd[1], wf_vel_cmd[2], wf_vel_cmd[3]





    def commander(self, vel_cmd):
        vx, vy, vz, w = self.body_to_world(vel_cmd)
        cmd = TwistStamped()
        cmd.twist.linear.x = vx
        cmd.twist.linear.y = vy
        cmd.twist.linear.z = vz
        cmd.twist.angular.z = w

        self.local_vel_pub.publish(cmd)
        self.rate.sleep()

    def odom_cb(self, data):
        self.current_odom=data

    def takeoff(self, x, y, z):
        Kp=1.5
        w=0
        while True:
            ex=x-self.current_odom.pose.pose.position.x
            ey=y-self.current_odom.pose.pose.position.y
            ez=z-self.current_odom.pose.pose.position.z


            vel=TwistStamped()
            vel.twist.linear.x = Kp*ex
            vel.twist.linear.y = Kp*ey
            vel.twist.linear.z = Kp*ez
            self.local_vel_pub.publish(vel)
            self.rate.sleep()
            if abs(ex)<=0.05 and abs(ey)<=0.05 and abs(ez)<=0.05 :
                print("Go!!")
                break

if __name__ == "__main__":


    rospy.init_node("controller")
    rate = rospy.Rate(10)
    controller = Controller()

    controller.takeoff(0, 0, 3)

    while not rospy.is_shutdown():
        vel_cmd=keyboard_control()
        controller.commander(vel_cmd)

