import rospy
import ros_numpy
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import laser_geometry.laser_geometry as lg
from controller import Controller
from cvxopt import matrix, solvers
import time
import cv2

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
    elif key == 75 or key == 107:
        vel[2]=1
    elif key == 76 or key == 108:
        vel[2]=-1
    cv2.imshow("keyboard", np.zeros((32, 32)))
    return vel

class ConstraintGenerator:
    def __init__(self):
#        self.object1 = rospy.Subscriber("/vrpn_client_node/RigidBody1/pose", PoseStamped, self.object1_cb)
#        self.object2 = rospy.Subscriber("/vrpn_client_node/RigidBody2/pose", PoseStamped, self.object2_cb)
#        self.mav = rospy.Subscriber("/vrpn_client_node/RigidBody3/pose", PoseStamped, self.mav_cb)
        self.mav = rospy.Subscriber("/mavros/local_position/odom", Odometry, self.mav_cb)
        self.rate = rospy.Rate(50)
        solvers.options['show_progress'] = False
        self.P = matrix(np.identity(3))
        self.Q = matrix(np.zeros(3))
        self.G = None
        self.H = None

        self._initialize()

        self.point1_x =  3.0
        self.point1_y =  3.0
        self.point1_z =  1.5

        self.point2_x =  -3.0
        self.point2_y =  3.0
        self.point2_z =  1.5

        self.safe_dis = 2.0

#    def object1_cb(self,data):
#        self.point1_x =  data.pose.position.x
#        self.point1_y =  data.pose.position.y
#        self.point1_z =  data.pose.position.z

#    def object1_cb(self,data):
#        self.point2_x =  data.pose.position.x
#        self.point2_y =  data.pose.position.y
#        self.point2_z =  data.pose.position.z


    def mav_cb(self, data):
        self.mav_x = data.pose.pose.position.x
        self.mav_y = data.pose.pose.position.y
        self.mav_z = data.pose.pose.position.z

    def _initialize(self):
        while self.mav is None:
            self.rate.sleep()

    def contraint_solver(self, u):
        h = [(self.mav_x-self.point1_x)**2+(self.mav_y-self.point1_y)**2 - self.safe_dis, \
        (self.mav_x-self.point2_x)**2+(self.mav_y-self.point2_y)**2 - self.safe_dis]

        diffx1 = self.mav_x-self.point1_x
        diffx2 = self.mav_x-self.point2_x

        diffy1 = self.mav_y-self.point1_y
        diffy2 = self.mav_y-self.point2_y

        self.H = matrix(h,tc='d')
        self.G = matrix([[-2*diffx1, -2*diffx2], [-2*diffy1, -2*diffy2],[0.0, 0.0]],tc='d')
        self.Q = matrix(-1*u[0:3],tc='d')

        #solvers.options['feastol']=1e-5
        sol=solvers.coneqp(self.P, self.Q, self.G, self.H)
        u_star = sol['x']
        #print(u_star)
        #print(sol['s'])

        return np.array([u_star[0], u_star[1],u_star[2],u[3]])


if __name__ == "__main__":
    rospy.init_node("uav_controller")
    controller = Controller()
    cg = ConstraintGenerator()

    controller.takeoff(0, 0, 1.5)
    while not rospy.is_shutdown():
        desired_vel_cmd = keyboard_control()
        constrained_vel_cmd = cg.contraint_solver(desired_vel_cmd)
        controller.commander(constrained_vel_cmd)
