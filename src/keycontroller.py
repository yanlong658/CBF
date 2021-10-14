# This Python file uses the following encoding: utf-8
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from mavros_msgs.msg import State
from cvxopt import matrix, solvers
import readchar
import threading
import math
import cv2

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
        ## controller
        self.local_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        ## get desired velocity from the motive.cpp
        self.desired_cmd = rospy.Subscriber("/desired_cmd_vel", TwistStamped, self.vel_cb)
        ## command
        self.odom_sub = rospy.Subscriber("/vrpn_client_node/RigidBody7/pose", PoseStamped, self.odom_cb)
        self.original_pub = rospy.Publisher("/desired_velocity", TwistStamped, queue_size=10)
        self.cbf_pub = rospy.Publisher("/cbf_velocity", TwistStamped, queue_size=10)

        self.rate = rospy.Rate(50.0)

        self.current_odom = PoseStamped()

        solvers.options['show_progress'] = False
        self.P = matrix(np.identity(3))
        self.Q = matrix(np.zeros(3))
        self.G = None
        self.H = None

        self.point1_x =  2.0
        self.point1_y =  0.0
        self.point1_z =  1.0

        self.point2_x =  -2.0
        self.point2_y =  0.0
        self.point2_z =  1.0

        self.safe_dis = 1.0

        self.u = np.array([0.0,0.0,0.0,0.0])

    def vel_cb(self,data):
        self.u[0] = data.twist.linear.x
        self.u[1] = data.twist.linear.y
        self.u[2] = data.twist.linear.z
        self.u[3] = data.twist.angular.z

    def odom_cb(self, data):
        self.current_odom=data

    def contraint_solver(self):
        h = [(self.current_odom.pose.position.x-self.point1_x)**2+(self.current_odom.pose.position.y-self.point1_y)**2 - self.safe_dis, \
        (self.current_odom.pose.position.x-self.point2_x)**2+(self.current_odom.pose.position.y-self.point2_y)**2 - self.safe_dis]

        diffx1 = self.current_odom.pose.position.x-self.point1_x
        diffx2 = self.current_odom.pose.position.x-self.point2_x

        diffy1 = self.current_odom.pose.position.y-self.point1_y
        diffy2 = self.current_odom.pose.position.y-self.point2_y

        self.H = matrix(h,tc='d')
        self.G = matrix([[-2*diffx1, -2*diffx2], [-2*diffy1, -2*diffy2],[0.0, 0.0]],tc='d')
        self.Q = matrix(-1*self.u[0:3],tc='d')

        #solvers.options['feastol']=1e-5
        sol=solvers.coneqp(self.P, self.Q, self.G, self.H)
        u_star = sol['x']
        #print(u_star)
        #print(sol['s'])

        origin_cmd = TwistStamped()
        cbf_cmd = TwistStamped()

        origin_cmd.twist.linear.x = self.u[0]
        origin_cmd.twist.linear.y = self.u[1]
        origin_cmd.twist.linear.z = self.u[2]

        cbf_cmd.twist.linear.x = u_star[0]
        cbf_cmd.twist.linear.y = u_star[1]
        cbf_cmd.twist.linear.z = u_star[2]

        self.original_pub.publish(origin_cmd)
        self.cbf_pub.publish(cbf_cmd)

        return np.array([u_star[0], u_star[1],u_star[2],self.u[3]])

#### velocity#####
    def commander(self, vel_cmd):
        vx, vy, vz, w = self.body_to_world(vel_cmd)
        cmd = TwistStamped()
        cmd.twist.linear.x = vx
        cmd.twist.linear.y = vy
        cmd.twist.linear.z = vz
        cmd.twist.angular.z = w

        self.local_vel_pub.publish(cmd)
        self.rate.sleep()

    def body_to_world(self,vel_cmd):
            q_x=self.current_odom.pose.orientation.x
            q_y=self.current_odom.pose.orientation.y
            q_z=self.current_odom.pose.orientation.z
            q_w=self.current_odom.pose.orientation.w
            row,pitch,yaw=quaternion_to_euler_angle(q_x, q_y, q_z,q_w)
            Rz=np.array([[math.cos(yaw),    -math.sin(yaw), 0,   0],

                         [math.sin(yaw),    math.cos(yaw),  0,   0],

                         [0,                     0,         1,   0],
                         [0,                     0,         0,   1]

                          ])
            wf_vel_cmd=np.dot(Rz,vel_cmd)



            return wf_vel_cmd[0], wf_vel_cmd[1], wf_vel_cmd[2], wf_vel_cmd[3]


if __name__ == "__main__":
    rospy.init_node("offb_node")
    controller = Controller()
    while not rospy.is_shutdown():
        constrained_vel_cmd = controller.contraint_solver()
        controller.commander(constrained_vel_cmd)

