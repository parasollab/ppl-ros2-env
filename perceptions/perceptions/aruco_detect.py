from math import frexp
from traceback import print_tb
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose, Detection2D
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import yaml
import os
import numpy as np
from cv2 import aruco

import pyrealsense2 as rs2


# Get the ROS distribution version and set the shared directory for arucoV5 configuration files.
ros_distribution = os.environ.get("ROS_DISTRO")
package_share_directory = get_package_share_directory('perceptions')

# Create a ROS 2 Node class arucoV5Ros2.
class arucoV5Ros2(Node):
    def __init__(self):
        super().__init__('aruco_ros2')
        # self.get_logger().info(f"Current ROS 2 distribution: {ros_distribution}")

        # Declare ROS parameters.
        self.declare_parameter("computing_device", "cpu", ParameterDescriptor(
            name="computing_device", description="Compute device selection, default: cpu, options: cuda:0"))

        self.declare_parameter("aruco_image_topic", "dddd", ParameterDescriptor(
            name="aruco_image_topic", description="Image topic, default: /image_raw"))

        self.declare_parameter("aruco_depth_topic", "", ParameterDescriptor(
            name="aruco_depth_topic", description="Image topic, default: /image_raw"))
        
        self.declare_parameter("aruco_camera_info_topic", "", ParameterDescriptor(
            name="aruco_camera_info_topic", description="Camera information topic, default: /camera/camera_info"))

        # Read parameters from the camera_info topic if available, otherwise, use the file-defined parameters.
        self.declare_parameter("aruco_camera_info_file", f"{package_share_directory}/config/camera_info.yaml", ParameterDescriptor(
            name="aruco_camera_info", description=f"Camera information file path, default: {package_share_directory}/config/camera_info.yaml"))

        # Default to displaying detection results.
        self.declare_parameter("aruco_show_result", False, ParameterDescriptor(
            name="aruco_show_result", description="Whether to display detection results, default: False"))

        # Default to publishing detection result images.
        self.declare_parameter("aruco_pub_result_img", True, ParameterDescriptor(
            name="aruco_pub_result_img", description="Whether to publish detection result images, default: False"))

        # 2. Create publishers.
        self.aruco_result_pub = self.create_publisher(
            Detection2DArray, "tag_detection_result", 1)
        self.result_msg = Detection2DArray()

        self.result_img_pub = self.create_publisher(Image, "tag_result_img", 1)

        # 3. Create an image subscriber (subscribe to depth information for 3D cameras, load camera info for 2D cameras).
        image_topic = self.get_parameter('aruco_image_topic').value
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 1)

        depth_topic = self.get_parameter('aruco_depth_topic').value
        self.depth_sub = self.create_subscription(
            Image, depth_topic, self.depth_callback, 1)


        camera_info_topic = self.get_parameter('aruco_camera_info_topic').value
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 1)


        detection_request_topic = '/tag_detection_request'
        self.detection_request_sub = self.create_subscription(
            Bool, detection_request_topic, self.detection_request_callback, 1)

        # Get camera information.
        # with open(self.get_parameter('aruco_camera_info_file').value) as f:
        #     self.camera_info = yaml.full_load(f.read())
        #     # self.get_logger().info(f"default_camera_info: {self.camera_info['k']} \n {self.camera_info['d']}")

        # 4. Image format conversion (using cv_bridge).
        self.bridge = CvBridge()

        self.show_result = self.get_parameter('aruco_show_result').value
        self.pub_result_img = self.get_parameter('aruco_pub_result_img').value

        self.depth_image = []

        self.detect_request = False

        # self.sub = self.create_subscription(topic, msg_Image, self.imageDepthCallback)
        self.intrinsics = None
        self.extrinsics = [7, -72]

        self.get_logger().info(f"{image_topic}, {depth_topic}, {camera_info_topic}")


    def camera_info_callback(self, msg: CameraInfo):
        """
        Get camera parameters through a callback function.
        """
        # self.camera_info['k'] = msg.k
        # self.camera_info['p'] = msg.p
        # self.camera_info['d'] = msg.d
        # self.camera_info['r'] = msg.r
        # self.camera_info['roi'] = msg.roi

        self.intrinsics = rs2.intrinsics()
        self.intrinsics.width = msg.width
        self.intrinsics.height = msg.height
        self.intrinsics.ppx = msg.k[2]
        self.intrinsics.ppy = msg.k[5]
        self.intrinsics.fx = msg.k[0]
        self.intrinsics.fy = msg.k[4]
        if msg.distortion_model == 'plumb_bob':
            self.intrinsics.model = rs2.distortion.brown_conrady
        elif msg.distortion_model == 'equidistant':
            self.intrinsics.model = rs2.distortion.kannala_brandt4
        self.intrinsics.coeffs = [i for i in msg.d]


    def depth_callback(self, msg: Image):
        image = self.bridge.imgmsg_to_cv2(msg)
        self.depth_image = image

    def detection_request_callback(self, msg):
        self.detect_request = msg.data


    def image_callback(self, msg: Image):
        # 5. Detect and publish results.
        if self.detect_request:
            image = self.bridge.imgmsg_to_cv2(msg)
            # self.get_logger().info(f"color image size: {np.shape(image)}")
            # mask = np.zeros(image.shape[:2], dtype=np.uint8)
            # mask[300::, 300:920] = 255
            # image = cv2.bitwise_and(image, image, mask=mask)
            self.get_logger().info("detection requested")

            self.result_msg.detections.clear()
            self.result_msg.header.frame_id = "camera"
            self.result_msg.header.stamp = self.get_clock().now().to_msg()

            # Parse the results.
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
            parameters =  cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(dictionary, parameters)
            corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
            image = cv2.aruco.drawDetectedMarkers(image.copy(), corners, ids)

            if ids is None:
                self.get_logger().info("not detected")
                ids = []

                
            for index in range(len(ids)):
                if index > 0:
                    break
                name = str(ids[index])
                c = corners[index][0]
                center_x = float(c[:, 0].mean())
                center_y = float(c[:, 1].mean())

                if self.intrinsics:
                    depth = self.depth_image[int(center_y), int(center_x)]
                    result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [int(center_x),int(center_y)], depth)
                    world_x = result[0] + self.extrinsics[0]
                    world_y = result[1] + self.extrinsics[1]
                    world_z = result[2]
                else:
                    break

                detection2d = Detection2D()
                detection2d.id = name

                obj_pose = ObjectHypothesisWithPose()
                obj_pose.hypothesis.class_id = name
                obj_pose.pose.pose.position.x = world_x
                obj_pose.pose.pose.position.y = world_y
                obj_pose.pose.pose.position.z = world_z


                # world_x, world_y = px2xy(
                #     [center_x, center_y], self.camera_info["k"], self.camera_info["d"], 1)
                # obj_pose.pose.pose.position.x = world_x
                # obj_pose.pose.pose.position.y = world_y

                detection2d.results.append(obj_pose)
                self.result_msg.detections.append(detection2d)

                self.get_logger().info(f"found tag: {world_x}, {world_y}, {world_z}")
                if self.show_result or self.pub_result_img:
                    cv2.circle(image, (360,590), radius=10, color=(0, 0, 255), thickness=4)
                    cv2.circle(image, (810,580), radius=10, color=(0, 0, 255), thickness=4)
                    cv2.circle(image, (585,585), radius=5, color=(0, 0, 255), thickness=4)
                    cv2.line(image, (360, 590), (810, 580), (255, 0, 255), thickness=2)
                    cv2.putText(image, f"{name} | ({world_x:.1f}  |  {world_y:.1f}  |  {world_z:.1f}", (int(center_x), int(center_y)),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
                    cv2.waitKey(1)

            # Display results if needed.
            if self.show_result:
                cv2.imshow('result', image)
                cv2.waitKey(1)

            # Publish result images if needed.
            if self.pub_result_img:
                result_img_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
                result_img_msg.header = msg.header
                self.result_img_pub.publish(result_img_msg)

            if len(ids) > 0:
                self.aruco_result_pub.publish(self.result_msg)
                self.result_msg = Detection2DArray()

        else:
            image = self.bridge.imgmsg_to_cv2(msg)
            if self.pub_result_img:
                cv2.circle(image, (360,590), radius=10, color=(0, 0, 255), thickness=4)
                cv2.circle(image, (810,580), radius=10, color=(0, 0, 255), thickness=4)
                cv2.circle(image, (585,585), radius=5, color=(0, 0, 255), thickness=4)
                cv2.line(image, (360, 590), (810, 580), (255, 0, 255), thickness=2)
                cv2.waitKey(1)

                result_img_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
                result_img_msg.header = msg.header
                self.result_img_pub.publish(result_img_msg)

def main(args=None):
    rclpy.init(args=args)
    arucoV5 = arucoV5Ros2()

    rclpy.spin(arucoV5)
    publisher_joint_trajectory.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
