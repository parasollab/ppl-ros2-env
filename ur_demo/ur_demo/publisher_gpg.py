import rclpy
from rclpy.node import Node

# from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import struct
import ctypes
import pcl
import ros2_numpy

import numpy as np
import pygpg

import matplotlib.pyplot as plt 


class PublisherGPG(Node):

    def __init__(self):
        super().__init__('publisher_gpg')
        self.get_logger().info(f'Start!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1')
        # self.declare_parameter("gripper_cfg_file_path", "home/lee/ur5e_ws/src/Universal_Robots_ROS2_Gazebo_Simulation/ur_simulation_gazebo/config/gripper_params.cfg")
        # self.gripper_cfg_file_path = self.get_parameter("gripper_cfg_file_path").value

        subscribe_topic = "/ur/camera/points"
        self.subscriber_point_cloud_ = self.create_subscription(
                    PointCloud2, subscribe_topic, self.gpg_callback, 1)
        self.subscriber_point_cloud_

        publish_topic = "/grasp_pose"
        self.publisher_grasp_pose_ = self.create_publisher(Float32MultiArray, publish_topic, 1)

        wait_sec_between_publish = 1.
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)


    def gpg_callback(self, msg):
        # point_cloud_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        # point_cloud_np = np.array(list(point_cloud_data))

        # xyz = np.array([[0,0,0]])
        # rgb = np.array([[0,0,0]])
        # #self.lock.acquire()
        # gen = pc2.read_points(msg, skip_nans=True)
        # int_data = list(gen)
        # for x in int_data:
        #     test = x[3] 
        #     # cast float32 to int so that bitwise operations are possible
        #     s = struct.pack('>f' ,test)
        #     i = struct.unpack('>l',s)[0]
        #     # you can get back the float value by the inverse operations
        #     pack = ctypes.c_uint32(i).value
        #     r = (pack & 0x00FF0000)>> 16
        #     g = (pack & 0x0000FF00)>> 8
        #     b = (pack & 0x000000FF)
        #     # prints r,g,b values in the 0-255 range
        #                 # x,y,z can be retrieved from the x[0],x[1],x[2]
        #     xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
        #     rgb = np.append(rgb,[[r,g,b]], axis = 0)

        pc = ros2_numpy.numpify(msg)

        rgb = pc['rgb']

        plt.imshow(rgb)






        # points=np.zeros((pc['x'].shape[0],3))
        # points[:,0]=pc['x']
        # points[:,1]=pc['y']
        # points[:,2]=pc['z']
        # p = pcl.PointCloud(np.array(points, dtype=np.float32))
        
        # # self.get_logger().info(f'xyz{np.shape(xyz)}')

        # # points = msg.data
        # # points = np.random.rand(3000, 3)  # put your point cloud here, should be a nX3 numpy array, here is an example random array
        # self.get_logger().info(f'read points')
        # num_samples = 10000
        # show_grasp = False
        # gripper_config_file = "home/lee/ur5e_ws/src/Universal_Robots_ROS2_Gazebo_Simulation/ur_simulation_gazebo/config/gripper_params.cfg"
        # self.get_logger().info(f'gripper config file:{gripper_config_file}')
        # self.get_logger().info(f'call function')
        # grasps = pygpg.generate_grasps(points, num_samples, show_grasp, gripper_config_file)
        # self.get_logger().info(f'done')
        # # grasps is a list of grasp objet, to construct a Transformation matrix from each grasp object, use:
        # pose_list = []
        # for grasp in grasps:
        #     pose = np.eye(4)
        #     pose[:3, 0] = grasp.get_grasp_approach()
        #     pose[:3, 1] = grasp.get_grasp_binormal()
        #     pose[:3, 2] = grasp.get_grasp_axis()
        #     pose[:3, 3] = grasp.get_grasp_bottom()

        #     for i in range(4):
        #         for j in range(4):
        #             pose_list.append(pose[i,j])

        # msg = Float32MultiArray()
        # msg.data = pose_list
        # self.publisher_grasp_pose_(msg)
            


    def timer_callback(self):
        # msg = String()
        # msg.data = 'Hello World: %d' % self.i
        msg = Float32MultiArray()
        # msg.data = [-2.87172984, -1.85668126, -2.12308832, -0.73293357,  1.57079633, -2.8717298, 0.025, 0.025]
        # self.publisher_.publish(msg)
        # self.get_logger().info(f'Publishing joints: "{msg.data}"')
        


def main(args=None):
    rclpy.init(args=args)

    publisher_gpg = PublisherGPG()

    plt.show()

    rclpy.spin(publisher_gpg)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher_gpg.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
