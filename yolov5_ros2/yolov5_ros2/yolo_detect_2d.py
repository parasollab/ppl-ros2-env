from math import frexp
from traceback import print_tb
from torch import imag
# from yolo import YOLOv5
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
from yolov5_ros2.cv_tool import px2xy
import os
import numpy as np
import pyrealsense2 as rs2

from ultralytics import YOLO
# model = YOLO('yolov8n.pt')  # load a pretrained YOLOv8n detection model

# Get the ROS distribution version and set the shared directory for YoloV5 configuration files.
ros_distribution = os.environ.get("ROS_DISTRO")
package_share_directory = get_package_share_directory('yolov5_ros2')

# Create a ROS 2 Node class YoloV5Ros2.
class YoloV5Ros2(Node):
    def __init__(self):
        super().__init__('yolov5_ros2')
        # self.get_logger().info(f"Current ROS 2 distribution: {ros_distribution}")

        # Declare ROS parameters.
        self.declare_parameter("computing_device", "cpu", ParameterDescriptor(
            name="computing_device", description="Compute device selection, default: cpu, options: cuda:0"))

        self.declare_parameter("yolo_model", "yolov5s", ParameterDescriptor(
            name="yolo_model", description="Default model selection: yolov5s"))

        self.declare_parameter("yolo_image_topic", "dddd", ParameterDescriptor(
            name="yolo_image_topic", description="Image topic, default: /image_raw"))

        self.declare_parameter("yolo_depth_topic", "", ParameterDescriptor(
            name="yolo_depth_topic", description="Image topic, default: /image_raw"))
        
        self.declare_parameter("yolo_camera_info_topic", "", ParameterDescriptor(
            name="yolo_camera_info_topic", description="Camera information topic, default: /camera/camera_info"))

        # Read parameters from the camera_info topic if available, otherwise, use the file-defined parameters.
        self.declare_parameter("yolo_camera_info_file", f"{package_share_directory}/config/camera_info.yaml", ParameterDescriptor(
            name="yolo_camera_info", description=f"Camera information file path, default: {package_share_directory}/config/camera_info.yaml"))

        # Default to displaying detection results.
        self.declare_parameter("yolo_show_result", False, ParameterDescriptor(
            name="yolo_show_result", description="Whether to display detection results, default: False"))

        # Default to publishing detection result images.
        self.declare_parameter("yolo_pub_result_img", True, ParameterDescriptor(
            name="yolo_pub_result_img", description="Whether to publish detection result images, default: False"))

        # 1. Load the model.
        model_path = package_share_directory + "/config/" + self.get_parameter('yolo_model').value + ".pt"
        device = self.get_parameter('computing_device').value
        # self.yolo = YOLOv5(model_path=model_path, device=device)
        self.yolo = YOLO('yolov8m.pt')

        # 2. Create publishers.
        self.yolo_result_pub = self.create_publisher(
            Detection2DArray, "/detection_result", 10)
        self.result_msg = Detection2DArray()

        self.result_img_pub = self.create_publisher(Image, "result_img", 10)

        # 3. Create an image subscriber (subscribe to depth information for 3D cameras, load camera info for 2D cameras).
        image_topic = self.get_parameter('yolo_image_topic').value
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)

        depth_topic = self.get_parameter('yolo_depth_topic').value
        self.depth_sub = self.create_subscription(
            Image, depth_topic, self.depth_callback, 10)

        self.get_logger().info(f"{device}, {image_topic}, {depth_topic}")

        camera_info_topic = self.get_parameter('yolo_camera_info_topic').value
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 1)

        detection_request_topic = '/detection_request'
        self.detection_request_sub = self.create_subscription(
            Bool, detection_request_topic, self.detection_request_callback, 10)

        # Get camera information.
        with open(self.get_parameter('yolo_camera_info_file').value) as f:
            self.camera_info = yaml.full_load(f.read())
            # self.get_logger().info(f"default_camera_info: {self.camera_info['k']} \n {self.camera_info['d']}")

        # 4. Image format conversion (using cv_bridge).
        self.bridge = CvBridge()

        self.show_result = self.get_parameter('yolo_show_result').value
        self.pub_result_img = self.get_parameter('yolo_pub_result_img').value

        self.depth_image = []

        self.detect_request = False

        self.intrinsics = None
        self.extrinsics = [7, -72]

        self.get_logger().info(f"{image_topic}, {depth_topic}, {camera_info_topic}")


    def camera_info_callback(self, msg: CameraInfo):
        # """
        # Get camera parameters through a callback function.
        # """
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

    def detection_request_callback(self, msg: Bool):
        self.detect_request = msg.data


    def image_callback(self, msg: Image):
        # 5. Detect and publish results.
        if self.detect_request:
            image = self.bridge.imgmsg_to_cv2(msg)
            # mask = np.zeros(image.shape[:2], dtype=np.uint8)
            # mask[100:600, 170:1000] = 255
            # image = cv2.bitwise_and(image, image, mask=mask)


            # results = self.yolo.track(image, persist=True,conf=0.5)
            results = self.yolo(image, verbose=False)
            names = self.yolo.names

            # self.get_logger().info(f"{results}")

            self.result_msg.detections.clear()
            self.result_msg.header.frame_id = "camera"
            self.result_msg.header.stamp = self.get_clock().now().to_msg()

            # # Parse the results.
            # predictions = results.pred[0]
            # boxes = predictions[:, :4]  # x1, y1, x2, y2
            # scores = predictions[:, 4]
            # categories = predictions[:, 5]

            # a = []
            # for index in range(len(categories)):
            #     name = results.names[int(categories[index])]
            #     detection2d = Detection2D()
            #     detection2d.id = name
            #     x1, y1, x2, y2 = boxes[index]
            #     x1 = int(x1)
            #     y1 = int(y1)
            #     x2 = int(x2)
            #     y2 = int(y2)
            #     center_x = (x1+x2)/2.0
            #     center_y = (y1+y2)/2.0
                # a.append(abs(x1-x2)*abs(y1-y2))

            
            # if len(categories) > 0:
            #     categories = [categories[np.argmax(a)]]
            for result in results:
                # boxes = result.boxes  # Boxes object for bbox outputs
                # masks = result.masks  # Masks object for segmentation masks outputs
                # keypoints = result.keypoints  # Keypoints object for pose outputs
                # probs = result.probs
                boxes = result.boxes.cpu().numpy()

                for box in boxes:
                    xyxy = box.xyxy[0].astype(int)
                    id = int(box.cls[0])
                    name = names[id]
    
                    x1 = int(xyxy[0])
                    y1 = int(xyxy[1])
                    x2 = int(xyxy[2])
                    y2 = int(xyxy[3])
                    center_x = (x1+x2)/2.0
                    center_y = (y1+y2)/2.0
                    if center_x < 170 or center_x > 1000:
                        continue
                    if center_y < 100 or center_y > 600:
                        continue
                    if abs(x2-x1) * abs(y2-y1) > 300*300:
                        continue
                    # Check if the id is unique
                    # if  int_id  not  in  unique_id:
                    #     unique_id.add(int_id)

                    detection2d = Detection2D()
                    detection2d.id = str(id)
        
                    detection2d.bbox.center.position.x = center_x
                    detection2d.bbox.center.position.y = center_y
                    
                    detection2d.bbox.size_x = float(x2-x1)
                    detection2d.bbox.size_y = float(y2-y1)

                    obj_pose = ObjectHypothesisWithPose()
                    obj_pose.hypothesis.class_id = name
                    # obj_pose.hypothesis.score = float(scores[index])

                    # # px2xy
                    # world_x, world_y = px2xy(
                    #     [center_x, center_y], self.camera_info["k"], self.camera_info["d"], 1)
                    if self.intrinsics:
                        depth = self.depth_image[int(center_y), int(center_x)]
                        result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [int(center_x),int(center_y)], depth)
                        world_x = result[0] + self.extrinsics[0]
                        world_y = result[1] + self.extrinsics[1]
                        world_z = result[2]
                    else:
                        break
            
                    obj_pose.pose.pose.position.x = world_x
                    obj_pose.pose.pose.position.y = world_y
                    obj_pose.pose.pose.position.z = world_z

                    detection2d.results.append(obj_pose)
                    self.result_msg.detections.append(detection2d)

                    # self.get_logger().info(f"{id} detected: {center_x}, {center_y}, {world_z}")
                    # Draw results.
                    if self.show_result or self.pub_result_img:
                        # Draw the bounding xyxy and id on the frame
                        cv2.rectangle(image, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (85, 45, 255), 2, lineType=cv2.LINE_AA)
                        cv2.putText(
                            image,
                            f"Id {name}, {world_x/1000:.2f}, {world_y/1000:.2f}, {world_z/1000:.2f}",
                            (xyxy[0], xyxy[1]),
                            0,
                            0.9,
                            [85, 45, 255],
                            2,
                            lineType=cv2.LINE_AA
                        )

                        cv2.circle(image, (360,590), radius=10, color=(0, 0, 255), thickness=4)
                        cv2.circle(image, (810,580), radius=10, color=(0, 0, 255), thickness=4)
                        cv2.circle(image, (585,585), radius=5, color=(0, 0, 255), thickness=4)
                        cv2.line(image, (360, 590), (810, 580), (255, 0, 255), thickness=2)
                        cv2.waitKey(1)


            # Display results if needed.
            if self.show_result:
                cv2.imshow('result', image)
                cv2.waitKey(1)

            # Publish result images if needed.
            if self.pub_result_img:
                result_img_msg = self.bridge.cv2_to_imgmsg(image, encoding="rgb8")
                result_img_msg.header = msg.header
                self.result_img_pub.publish(result_img_msg)

            if len(results) > 0:
                self.yolo_result_pub.publish(self.result_msg)
        
        else:
            image = self.bridge.imgmsg_to_cv2(msg)
            if self.pub_result_img:
                cv2.circle(image, (360,590), radius=10, color=(0, 0, 255), thickness=4)
                cv2.circle(image, (810,580), radius=10, color=(0, 0, 255), thickness=4)
                cv2.circle(image, (585,585), radius=5, color=(0, 0, 255), thickness=4)
                cv2.line(image, (360, 590), (810, 580), (255, 0, 255), thickness=2)
                cv2.waitKey(1)

                result_img_msg = self.bridge.cv2_to_imgmsg(image, encoding="rgb8")
                result_img_msg.header = msg.header
                self.result_img_pub.publish(result_img_msg)



def main(args=None):
    rclpy.init(args=args)
    yoloV5 = YoloV5Ros2()

    rclpy.spin(yoloV5)
    publisher_joint_trajectory.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
