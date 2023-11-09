# Copyright 2022 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Denis Štogl, Lovro Ivanov
#

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from rcl_interfaces.msg import ParameterDescriptor

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray


class PublisherJointTrajectory(Node):
    def __init__(self):
        super().__init__("publisher_position_trajectory_controller")
        # Declare all parameters
        self.declare_parameter("controller_name", "position_trajectory_controller")
        self.declare_parameter("wait_sec_between_publish", 1)
        self.declare_parameter("goal_names", ["pos1", "pos2"])
        self.declare_parameter("joints", [""])
        self.declare_parameter("check_starting_point", False)

        # Read parameters
        controller_name = self.get_parameter("controller_name").value
        wait_sec_between_publish = self.get_parameter("wait_sec_between_publish").value
        goal_names = self.get_parameter("goal_names").value
        self.joints = self.get_parameter("joints").value

        # self.get_logger().info(f"Joint INFO: {self.joints}")

        self.check_starting_point = self.get_parameter("check_starting_point").value

        # self.get_logger().info(f"starting point: {self.check_starting_point}")

        self.starting_point = {}

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is not set!')


        # initialize starting point status
        self.starting_point_ok = not self.check_starting_point
        self.joint_state_msg_received = False

        # Read all positions from parameters
        self.goals = []  # List of JointTrajectoryPoint

        subscribe_topic = "/ur_joint_command"
        self.subscriber_joint_values_ = self.create_subscription(Float32MultiArray, subscribe_topic, self.joint_values_callback, 1)
        self.subscriber_joint_values_

        publish_topic = "/" + controller_name + "/" + "joint_trajectory"

        self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 1)
        # self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)
        self.i = 0

        self.goal = []
        self.goal_exist = False

    def joint_values_callback(self, msg):
        if msg.data:
            self.goal = [msg.data[i] for i in range(len(msg.data))]
            point = JointTrajectoryPoint()
            point.positions = self.goal
            point.time_from_start = Duration(sec=4)

            traj = JointTrajectory()
            traj.joint_names = self.joints
            traj.points.append(point)
            self.publisher_.publish(traj)
        
def main(args=None):
    rclpy.init(args=args)

    publisher_joint_trajectory = PublisherJointTrajectory()

    rclpy.spin(publisher_joint_trajectory)
    publisher_joint_trajectory.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()