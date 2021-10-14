
import rospy
import ros_numpy
import numpy as np
import open3d as o3d
from sensor_msgs.msg import LaserScan
import laser_geometry.laser_geometry as lg
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from controller import Controller
from cvxopt import matrix, solvers
import time
import cv2


"""
implement constraint generator

TODO:
    0. Knowing how to use CBF (paper reading)
    1. (Maybe) need coordinate transform
    2. Design P, Q, G, H. Refer to cvxopt tutorial
    3. Instead of publishing command randomly, implement a keyboard commander (optional)
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

class ConstraintGenerator:
    def __init__(self):
        self.pcl_sub = rospy.Subscriber("/laser/scan", LaserScan, self.scan_cb)
        self.original_pub = rospy.Publisher("/desired_velocity", TwistStamped, queue_size=10)
        self.cbf_pub = rospy.Publisher("/cbf_velocity", TwistStamped, queue_size=10)

        self.rate = rospy.Rate(20)

        self.pcl = None
        self.lp = lg.LaserProjection()
        self.safe_dis = 1.0

        solvers.options['show_progress'] = False
        self.P = matrix(np.identity(3))
        self.Q = matrix(np.zeros(3))
        self.G = None
        self.H = None

        self._initialize()

    def scan_cb(self, data):
        pc2_data = self.lp.projectLaser(data)
        xyz = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_data)
        #print(np.linalg.norm(xyz,axis=1,keepdims=True))



        self.pcl = xyz

    def contraint_solver(self, u):
        pcl = o3d.geometry.PointCloud()
        pcl.points = o3d.utility.Vector3dVector(self.pcl)
        downpcl = pcl.voxel_down_sample(voxel_size=0.1)
        pcl = np.asarray(downpcl.points)
        pcl = pcl[np.abs(pcl[:, 0]) < 10.0]
        pcl = pcl[np.abs(pcl[:, 1]) < 10.0]
        pcl = pcl[np.abs(pcl[:, 2]) < 0.5]
        print(np.linalg.norm(pcl,axis=1,keepdims=True))


        row_sum_square=np.square(pcl).sum(axis=1)
        h=row_sum_square-self.safe_dis*self.safe_dis



        self.Q = matrix(-1*u[0:3],tc='d')
        self.G = matrix(pcl,tc='d')
        self.H = matrix(h,tc='d')
        #solvers.options['feastol']=1e-5
        sol=solvers.coneqp(self.P, self.Q, self.G, self.H)
        u_star = sol['x']
        #print(u_star)
        #print(sol['s'])
        origin_cmd = TwistStamped()
        cbf_cmd = TwistStamped()

        origin_cmd.header.stamp = rospy.get_rostime()
        origin_cmd.header.frame_id = 'control'
        origin_cmd.twist.linear.x = u[0]
        origin_cmd.twist.linear.y = u[1]
        origin_cmd.twist.linear.z = u[2]

        cbf_cmd.header.stamp = rospy.get_rostime()
        cbf_cmd.header.frame_id = 'control'
        cbf_cmd.twist.linear.x = u_star[0]
        cbf_cmd.twist.linear.y = u_star[1]
        cbf_cmd.twist.linear.z = u_star[2]

        self.original_pub.publish(origin_cmd)
        self.cbf_pub.publish(cbf_cmd)

        return np.array([u_star[0], u_star[1], u_star[2],u[3]])




    def _initialize(self):
        while self.pcl is None:
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("constraint_generator_node")
    controller = Controller()
    cg = ConstraintGenerator()


    controller.takeoff(0, 0, 1.5)

    while not rospy.is_shutdown():
        desired_vel_cmd = keyboard_control()
        constrained_vel_cmd = cg.contraint_solver(desired_vel_cmd)
        controller.commander(constrained_vel_cmd)
