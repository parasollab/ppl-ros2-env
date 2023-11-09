#!/usr/bin/env python3

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
import time
import numpy as np
import random
import yaml
import sys
from math import pi

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool

import rclpy
from rclpy.node import Node

"""Module to control Robotiq's grippers - tested with HAND-E"""

import socket
import threading
import time
from enum import Enum
from typing import Union, Tuple, OrderedDict

class RobotiqGripper:
    """
    Communicates with the gripper directly, via socket with string commands, leveraging string names for variables.
    """
    # WRITE VARIABLES (CAN ALSO READ)
    ACT = 'ACT'  # act : activate (1 while activated, can be reset to clear fault status)
    GTO = 'GTO'  # gto : go to (will perform go to with the actions set in pos, for, spe)
    ATR = 'ATR'  # atr : auto-release (emergency slow move)
    ADR = 'ADR'  # adr : auto-release direction (open(1) or close(0) during auto-release)
    FOR = 'FOR'  # for : force (0-255)
    SPE = 'SPE'  # spe : speed (0-255)
    POS = 'POS'  # pos : position (0-255), 0 = open
    # READ VARIABLES
    STA = 'STA'  # status (0 = is reset, 1 = activating, 3 = active)
    PRE = 'PRE'  # position request (echo of last commanded position)
    OBJ = 'OBJ'  # object detection (0 = moving, 1 = outer grip, 2 = inner grip, 3 = no object at rest)
    FLT = 'FLT'  # fault (0=ok, see manual for errors if not zero)

    ENCODING = 'UTF-8'  # ASCII and UTF-8 both seem to work

    class GripperStatus(Enum):
        """Gripper status reported by the gripper. The integer values have to match what the gripper sends."""
        RESET = 0
        ACTIVATING = 1
        # UNUSED = 2  # This value is currently not used by the gripper firmware
        ACTIVE = 3

    class ObjectStatus(Enum):
        """Object status reported by the gripper. The integer values have to match what the gripper sends."""
        MOVING = 0
        STOPPED_OUTER_OBJECT = 1
        STOPPED_INNER_OBJECT = 2
        AT_DEST = 3

    def __init__(self):
        """Constructor."""
        self.socket = None
        self.command_lock = threading.Lock()
        self._min_position = 0
        self._max_position = 255
        self._min_speed = 0
        self._max_speed = 255
        self._min_force = 0
        self._max_force = 255

    def connect(self, hostname: str, port: int, socket_timeout: float = 2.0) -> None:
        """Connects to a gripper at the given address.
        :param hostname: Hostname or ip.
        :param port: Port.
        :param socket_timeout: Timeout for blocking socket operations.
        """
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((hostname, port))
        self.socket.settimeout(socket_timeout)

    def disconnect(self) -> None:
        """Closes the connection with the gripper."""
        self.socket.close()

    def _set_vars(self, var_dict: OrderedDict[str, Union[int, float]]):
        """Sends the appropriate command via socket to set the value of n variables, and waits for its 'ack' response.
        :param var_dict: Dictionary of variables to set (variable_name, value).
        :return: True on successful reception of ack, false if no ack was received, indicating the set may not
        have been effective.
        """
        # construct unique command
        cmd = "SET"
        for variable, value in var_dict.items():
            cmd += f" {variable} {str(value)}"
        cmd += '\n'  # new line is required for the command to finish
        # atomic commands send/rcv
        with self.command_lock:
            self.socket.sendall(cmd.encode(self.ENCODING))
            data = self.socket.recv(1024)
        return self._is_ack(data)

    def _set_var(self, variable: str, value: Union[int, float]):
        """Sends the appropriate command via socket to set the value of a variable, and waits for its 'ack' response.
        :param variable: Variable to set.
        :param value: Value to set for the variable.
        :return: True on successful reception of ack, false if no ack was received, indicating the set may not
        have been effective.
        """
        return self._set_vars(OrderedDict([(variable, value)]))

    def _get_var(self, variable: str):
        """Sends the appropriate command to retrieve the value of a variable from the gripper, blocking until the
        response is received or the socket times out.
        :param variable: Name of the variable to retrieve.
        :return: Value of the variable as integer.
        """
        # atomic commands send/rcv
        with self.command_lock:
            cmd = f"GET {variable}\n"
            self.socket.sendall(cmd.encode(self.ENCODING))
            data = self.socket.recv(1024)

        # expect data of the form 'VAR x', where VAR is an echo of the variable name, and X the value
        # note some special variables (like FLT) may send 2 bytes, instead of an integer. We assume integer here
        var_name, value_str = data.decode(self.ENCODING).split()
        if var_name != variable:
            raise ValueError(f"Unexpected response {data} ({data.decode(self.ENCODING)}): does not match '{variable}'")
        value = int(value_str)
        return value

    @staticmethod
    def _is_ack(data: str):
        return data == b'ack'

    def _reset(self):
        """
        Reset the gripper.
        The following code is executed in the corresponding script function
        def rq_reset(gripper_socket="1"):
            rq_set_var("ACT", 0, gripper_socket)
            rq_set_var("ATR", 0, gripper_socket)

            while(not rq_get_var("ACT", 1, gripper_socket) == 0 or not rq_get_var("STA", 1, gripper_socket) == 0):
                rq_set_var("ACT", 0, gripper_socket)
                rq_set_var("ATR", 0, gripper_socket)
                sync()
            end

            sleep(0.5)
        end
        """
        self._set_var(self.ACT, 0)
        self._set_var(self.ATR, 0)
        while (not self._get_var(self.ACT) == 0 or not self._get_var(self.STA) == 0):
            self._set_var(self.ACT, 0)
            self._set_var(self.ATR, 0)
        time.sleep(0.5)


    def activate(self, auto_calibrate: bool = True):
        """Resets the activation flag in the gripper, and sets it back to one, clearing previous fault flags.
        :param auto_calibrate: Whether to calibrate the minimum and maximum positions based on actual motion.
        The following code is executed in the corresponding script function
        def rq_activate(gripper_socket="1"):
            if (not rq_is_gripper_activated(gripper_socket)):
                rq_reset(gripper_socket)

                while(not rq_get_var("ACT", 1, gripper_socket) == 0 or not rq_get_var("STA", 1, gripper_socket) == 0):
                    rq_reset(gripper_socket)
                    sync()
                end

                rq_set_var("ACT",1, gripper_socket)
            end
        end
        def rq_activate_and_wait(gripper_socket="1"):
            if (not rq_is_gripper_activated(gripper_socket)):
                rq_activate(gripper_socket)
                sleep(1.0)

                while(not rq_get_var("ACT", 1, gripper_socket) == 1 or not rq_get_var("STA", 1, gripper_socket) == 3):
                    sleep(0.1)
                end

                sleep(0.5)
            end
        end
        """
        if not self.is_active():
            self._reset()
            while (not self._get_var(self.ACT) == 0 or not self._get_var(self.STA) == 0):
                time.sleep(0.01)

            self._set_var(self.ACT, 1)
            time.sleep(1.0)
            while (not self._get_var(self.ACT) == 1 or not self._get_var(self.STA) == 3):
                time.sleep(0.01)

        # auto-calibrate position range if desired
        if auto_calibrate:
            self.auto_calibrate()

    def is_active(self):
        """Returns whether the gripper is active."""
        status = self._get_var(self.STA)
        return RobotiqGripper.GripperStatus(status) == RobotiqGripper.GripperStatus.ACTIVE

    def get_min_position(self) -> int:
        """Returns the minimum position the gripper can reach (open position)."""
        return self._min_position

    def get_max_position(self) -> int:
        """Returns the maximum position the gripper can reach (closed position)."""
        return self._max_position

    def get_open_position(self) -> int:
        """Returns what is considered the open position for gripper (minimum position value)."""
        return self.get_min_position()

    def get_closed_position(self) -> int:
        """Returns what is considered the closed position for gripper (maximum position value)."""
        return self.get_max_position()

    def is_open(self):
        """Returns whether the current position is considered as being fully open."""
        return self.get_current_position() <= self.get_open_position()

    def is_closed(self):
        """Returns whether the current position is considered as being fully closed."""
        return self.get_current_position() >= self.get_closed_position()

    def get_current_position(self) -> int:
        """Returns the current position as returned by the physical hardware."""
        return self._get_var(self.POS)

    def auto_calibrate(self, log: bool = True) -> None:
        """Attempts to calibrate the open and closed positions, by slowly closing and opening the gripper.
        :param log: Whether to print the results to log.
        """
        # first try to open in case we are holding an object
        (position, status) = self.move_and_wait_for_pos(self.get_open_position(), 64, 1)
        if RobotiqGripper.ObjectStatus(status) != RobotiqGripper.ObjectStatus.AT_DEST:
            raise RuntimeError(f"Calibration failed opening to start: {str(status)}")

        # try to close as far as possible, and record the number
        (position, status) = self.move_and_wait_for_pos(self.get_closed_position(), 64, 1)
        if RobotiqGripper.ObjectStatus(status) != RobotiqGripper.ObjectStatus.AT_DEST:
            raise RuntimeError(f"Calibration failed because of an object: {str(status)}")
        assert position <= self._max_position
        self._max_position = position

        # try to open as far as possible, and record the number
        (position, status) = self.move_and_wait_for_pos(self.get_open_position(), 64, 1)
        if RobotiqGripper.ObjectStatus(status) != RobotiqGripper.ObjectStatus.AT_DEST:
            raise RuntimeError(f"Calibration failed because of an object: {str(status)}")
        assert position >= self._min_position
        self._min_position = position

        if log:
            print(f"Gripper auto-calibrated to [{self.get_min_position()}, {self.get_max_position()}]")

    def move(self, position: int, speed: int, force: int) -> Tuple[bool, int]:
        """Sends commands to start moving towards the given position, with the specified speed and force.
        :param position: Position to move to [min_position, max_position]
        :param speed: Speed to move at [min_speed, max_speed]
        :param force: Force to use [min_force, max_force]
        :return: A tuple with a bool indicating whether the action it was successfully sent, and an integer with
        the actual position that was requested, after being adjusted to the min/max calibrated range.
        """

        def clip_val(min_val, val, max_val):
            return max(min_val, min(val, max_val))

        clip_pos = clip_val(self._min_position, position, self._max_position)
        clip_spe = clip_val(self._min_speed, speed, self._max_speed)
        clip_for = clip_val(self._min_force, force, self._max_force)

        # moves to the given position with the given speed and force
        var_dict = OrderedDict([(self.POS, clip_pos), (self.SPE, clip_spe), (self.FOR, clip_for), (self.GTO, 1)])
        return self._set_vars(var_dict), clip_pos

    def move_and_wait_for_pos(self, position: int, speed: int, force: int) -> Tuple[int, ObjectStatus]:  # noqa
        """Sends commands to start moving towards the given position, with the specified speed and force, and
        then waits for the move to complete.
        :param position: Position to move to [min_position, max_position]
        :param speed: Speed to move at [min_speed, max_speed]
        :param force: Force to use [min_force, max_force]
        :return: A tuple with an integer representing the last position returned by the gripper after it notified
        that the move had completed, a status indicating how the move ended (see ObjectStatus enum for details). Note
        that it is possible that the position was not reached, if an object was detected during motion.
        """
        set_ok, cmd_pos = self.move(position, speed, force)
        if not set_ok:
            raise RuntimeError("Failed to set variables for move.")

        # wait until the gripper acknowledges that it will try to go to the requested position
        while self._get_var(self.PRE) != cmd_pos:
            time.sleep(0.001)

        # wait until not moving
        cur_obj = self._get_var(self.OBJ)
        while RobotiqGripper.ObjectStatus(cur_obj) == RobotiqGripper.ObjectStatus.MOVING:
            cur_obj = self._get_var(self.OBJ)

        # report the actual position and the object status
        final_pos = self._get_var(self.POS)
        final_obj = cur_obj
        return final_pos, RobotiqGripper.ObjectStatus(final_obj)



class PickAndPlace(Node):
    
    def __init__(self):
        super().__init__('pick_and_place')

        self.publisher_ur_command = self.create_publisher(
                Float32MultiArray, "/ur_joint_command", 1)
        self.publisher_ee_position = self.create_publisher(
                    Float32MultiArray, "/ee_position", 1)

        self.subscriber_joint_values = self.create_subscription(
                    Float32MultiArray, "/ik_joint_values", self.ik_joint_callback, 1)
        self.subscriber_joint_state = self.create_subscription(
                    JointState, "/joint_states", self.joint_state_callback, 1)

        self.publisher_detection_request = self.create_publisher(
                    Bool, "/detection_request", 1)
        self.subscriber_object_pixel_position = self.create_subscription(
                    Detection2DArray, "/detection_result", self.detection_result_callback, 1)
        self.subscriber_camera_info = self.create_subscription(
                    CameraInfo, "/camera/camera_info", self.camera_info_callback, 1)

        self.subscriber_camera_info
        self.subscriber_joint_values
        self.subscriber_joint_state
        self.subscriber_object_pixel_position

        self.dXYZ = {}
        self.objCnt = {}
        
        self.grasp_offset = 0.21
        self.backoff_offset = 0.35
        self.hover_offset = 0.5

        self.ee_goal = [0., 0., -1., -0.3,       0., -1., 0., 0.,       -1., 0., 0., self.hover_offset]
        self.prev_ee_goal = []
        self.ee_state = []

        self.gripper_open = []
        self.gripper_close = []
        
        self.joint_goal = []
        self.prev_joint_goal = []

        self.ik_joint = []

        self.joint_goal_received = False
        self.joint_goal_reached = False
        self.joint_states = []

        self.image_height = 0
        self.image_width = 0

        self.object_detected = False

        


    def camera_info_callback(self, msg):
        if msg.width:
            self.image_height = msg.height
            self.image_width = msg.width

    def ik_joint_callback(self, msg):
        if msg.data:
            self.ik_joint = msg.data.tolist()
        

    def joint_state_callback(self, msg):
        if msg.position:
            if len(self.joint_states) == 0:
                self.joint_states = [msg.position[i] for i in range(len(msg.position))]
            self.joint_states[0] = msg.position[0]
            self.joint_states[2] = msg.position[1]
            self.joint_states[3] = msg.position[2]
            self.joint_states[4] = msg.position[3]
            self.joint_states[5] = msg.position[4]
            self.joint_states[1] = msg.position[5]

            # self.get_logger().info(f"{self.joint_states}, {self.joint_goal}")

            if len(self.joint_goal) > 0:
                joint_goal = np.array(self.joint_goal)
                joint_states = np.array(self.joint_states)
                self.joint_goal_reached = np.linalg.norm(joint_states-joint_goal) < 0.001
                
                # Initilize variables
                if self.joint_goal_reached:
                    self.get_logger().info('goal reached')
                    self.ik_joint = []
                    self.joint_goal = []
                else:
                    pass


    def detection_result_callback(self, msg):
        if len(msg.detections)>0 and len(self.objCnt.keys())==0:
            detections = msg.detections

            self.objCnt = {}
            for i in range(len(detections)):
                obj = detections[i].results[0].hypothesis.class_id
                if obj in self.objCnt.keys():
                    self.objCnt[str(obj)] += 1
                else:
                    self.objCnt[str(obj)] = 1

            self.dXYZ = {}
            for i in range(len(detections)):
                obj = detections[i].results[0].hypothesis.class_id

                dx = (detections[i].results[0].pose.pose.position.x)/1000.
                dy = (detections[i].results[0].pose.pose.position.y)/1000.
                dz = (detections[i].results[0].pose.pose.position.z)/1000.

                self.dXYZ[str(obj)+str(self.objCnt[str(obj)])]=[0., 0., 0., -dy, 0., 0., 0., -dx, 0., 0., 0., 0.]
                self.objCnt[str(obj)] -= 1

            # self.get_logger().info(f"found {obj}: {self.deltaPixel} | {self.dXYZ}")
            self.object_detected = True
            self.get_logger().info(f"{self.dXYZ.keys()}")


    def pulbish_ee_goal(self, ee_position):
        msg = Float32MultiArray()
        msg.data = ee_position
        self.ee_goal = ee_position
        self.publisher_ee_position.publish(msg)
            
    def publish_ur_command(self, joint_goal):
        # self.get_logger().info(f"publishing {joint_goal}")
        msg = Float32MultiArray()
        msg.data = joint_goal
        self.publisher_ur_command.publish(msg)

    def publish_detection_request(self, detection):
        msg = Bool()
        msg.data = detection
        self.publisher_detection_request.publish(msg)
        
        

def main():
    rclpy.init()
    
    node = PickAndPlace()

    node.get_logger().info(f"Warming up the system")
    node.get_logger().info("Creating gripper...")
    gripper = RobotiqGripper()
    node.get_logger().info("Connecting to gripper...")
    ip = "192.168.1.9"
    gripper.connect(ip, 63352)
    node.get_logger().info("Activating gripper...")
    gripper.activate()
    

    ####################################
    # Move the arm to the inital position
    init_ee = node.ee_goal
    node.ik_joint = []
    while len(node.ik_joint) == 0:
        node.pulbish_ee_goal(init_ee)
        rclpy.spin_once(node)
    node.get_logger().info("Found IK")
    node.joint_goal = node.ik_joint
    # node.joint_goal[0] -= np.pi # Base Rotation. Need to generalize this.

    node.publish_ur_command(node.joint_goal)
    while not node.joint_goal_reached:
        rclpy.spin_once(node)
    node.joint_goal_reached = False
    node.get_logger().info("Initialized")
    time.sleep(1)


    ####################################
    # Detect the aruco markers
    while not node.object_detected:
        node.publish_detection_request(True)
        rclpy.spin_once(node)
        # For debug purpose
        # node.object_detected = False
    node.get_logger().info(f"object_detected")
    node.publish_detection_request(False)
    node.object_detected = False
    rclpy.spin_once(node)
    time.sleep(1)



    for key in node.dXYZ.keys():
        name = key
        node.get_logger().info(f"Sorting {name}")
        dXYZ = node.dXYZ[key]
        ####################################
        # Move the arm to the inital position
        node.get_logger().info("home position")
        node.ik_joint = []
        while len(node.ik_joint) == 0:
            node.pulbish_ee_goal(init_ee)
            rclpy.spin_once(node)
        node.joint_goal = node.ik_joint
        # node.joint_goal[0] -= np.pi # Base Rotation. Need to generalize this.

        node.publish_ur_command(node.joint_goal)
        while not node.joint_goal_reached:
            rclpy.spin_once(node)
        node.joint_goal_reached = False
        time.sleep(0.1)


        ####################################
        # Move to the hover position
        node.get_logger().info("hover")
        node.ee_goal = [x + y for x, y in zip(node.ee_goal, dXYZ)]
        node.get_logger().info(f"{node.ee_goal}")
        node.ee_goal[-1] = node.hover_offset
        node.ik_joint = []
        while len(node.ik_joint) == 0:
            node.pulbish_ee_goal(node.ee_goal)
            rclpy.spin_once(node)
        node.joint_goal = node.ik_joint
        # node.joint_goal[0] -= np.pi 

        node.publish_ur_command(node.joint_goal)
        while not node.joint_goal_reached:
            rclpy.spin_once(node)
        node.joint_goal_reached = False
        time.sleep(0.1)



        ####################################
        # Move to the backoff position
        node.get_logger().info("backoff")
        node.ee_goal[-1] = node.backoff_offset
        node.get_logger().info(f"{node.ee_goal}")
        node.ik_joint = []
        while len(node.ik_joint) == 0:
            node.pulbish_ee_goal(node.ee_goal)
            rclpy.spin_once(node)
        node.joint_goal = node.ik_joint
        # node.joint_goal[0] -= np.pi 

        node.publish_ur_command(node.joint_goal)
        while not node.joint_goal_reached:
            rclpy.spin_once(node)
        node.joint_goal_reached = False
        time.sleep(0.1)

        

        ####################################
        # Move to the grasp position
        node.get_logger().info("grasp")
        node.ee_goal[-1] = node.grasp_offset
        node.get_logger().info(f"{node.ee_goal}")
        node.ik_joint = []
        while len(node.ik_joint) == 0:
            node.pulbish_ee_goal(node.ee_goal)
            rclpy.spin_once(node)
        node.joint_goal = node.ik_joint
        # node.joint_goal[0] -= np.pi 

        node.publish_ur_command(node.joint_goal)
        while not node.joint_goal_reached:
            rclpy.spin_once(node)
        node.joint_goal_reached = False
        time.sleep(0.1)

        # Close the gripper
        node.get_logger().info("close gripper")
        gripper.move_and_wait_for_pos(255, 255, 255)
        time.sleep(0.1)

        ####################################
        # Move to the backoff position
        node.ee_goal[-1] = node.backoff_offset
        node.ik_joint = []
        while len(node.ik_joint) == 0:
            node.pulbish_ee_goal(node.ee_goal)
            rclpy.spin_once(node)
        node.joint_goal = node.ik_joint
        # node.joint_goal[0] -= np.pi 

        node.publish_ur_command(node.joint_goal)
        while not node.joint_goal_reached:
            rclpy.spin_once(node)
        node.joint_goal_reached = False

        ####################################
        # Move to the target backoff position
        if "bottle" in name:
            node.ee_goal = [0., 0., -1., -0.3,       0., -1., 0., -0.4,       -1., 0., 0., 0.2]
        else:
            node.ee_goal = [0., 0., 1., -0.3,       0., 1., 0., 0.,       -1., 0., 0., 0.2]

        node.ee_goal[-1] = 0.3
        node.ik_joint = []
        while len(node.ik_joint) == 0:
            node.pulbish_ee_goal(node.ee_goal)
            rclpy.spin_once(node)
        node.joint_goal = node.ik_joint
        # node.joint_goal[0] -= np.pi 

        node.publish_ur_command(node.joint_goal)
        while not node.joint_goal_reached:
            rclpy.spin_once(node)
        node.joint_goal_reached = False

        ####################################
        # Move to the target grasp position
        node.ee_goal[-1] = 0.3
        node.ik_joint = []
        while len(node.ik_joint) == 0:
            node.pulbish_ee_goal(node.ee_goal)
            rclpy.spin_once(node)
        node.joint_goal = node.ik_joint
        # node.joint_goal[0] -= np.pi 

        node.publish_ur_command(node.joint_goal)
        while not node.joint_goal_reached:
            rclpy.spin_once(node)
        node.joint_goal_reached = False

        # Open the gripper
        gripper.move_and_wait_for_pos(0, 255, 255)
        time.sleep(0.1)


        ####################################
        # Move to the targe backoff position
        node.ee_goal[-1] = node.backoff_offset
        node.ik_joint = []
        while len(node.ik_joint) == 0:
            node.pulbish_ee_goal(node.ee_goal)
            rclpy.spin_once(node)
        node.joint_goal = node.ik_joint
        # node.joint_goal[0] -= np.pi 

        node.publish_ur_command(node.joint_goal)
        while not node.joint_goal_reached:
            rclpy.spin_once(node)
        node.joint_goal_reached = False


    # Terminate the nodes
    rclpy.shutdown()

if __name__ == '__main__':
    main()




