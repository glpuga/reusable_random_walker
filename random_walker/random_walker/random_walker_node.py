#!/usr/bin/env python3

# Copyright 2024 Gerardo Puga
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

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

import random
import yaml


class RandomWalkerNode(Node):
    def __init__(self):
        """Create a RandomWalkerNode."""
        super().__init__("random_walker")

        self.declare_parameter("goal_timeout", 50.0)
        self.declare_parameter("timer_period", 1.0)
        self.declare_parameter("goal_tolerance", 2.0)
        self.declare_parameter("goals_file_path", "/path_to/file/file.yaml")

        self.goal_timeout_ = (
            self.get_parameter("goal_timeout").get_parameter_value().double_value
        )

        self.timer_period_ = (
            self.get_parameter("timer_period").get_parameter_value().double_value
        )

        self.goal_tolerance_ = (
            self.get_parameter("goal_tolerance").get_parameter_value().double_value
        )

        self.goals_file_ = (
            self.get_parameter("goals_file_path").get_parameter_value().string_value
        )

        self.current_pose_ = None
        self.current_goal_ = None

        self.goal_publisher_ = self.create_publisher(PoseStamped, "goal_pose", 1)

        self.current_pose_subscriber_ = self.create_subscription(
            PoseWithCovarianceStamped, "pose", self.current_pose_callback, 1
        )

        self.goals_ = self.load_goals_file(self.goals_file_)

        self.uptime_ = 0.0
        self.timer_ = self.create_timer(self.timer_period_, self.timer_callback)

    def timer_callback(self):
        """Publish a new goal every goal_timeout seconds."""
        if self.goals_ is None:
            self.get_logger().error("No navigation goals loaded, exiting")
            return
        pick_next = False
        if self.current_pose_ is None:
            self.get_logger().info("No pose received yet, forcing new goal")
            pick_next = True
        else:
            if self.current_goal_ is None:
                pick_next = True
            else:
                self.uptime_ += self.timer_period_
                x_diff = self.current_goal_[0] - self.current_pose_.pose.pose.position.x
                y_diff = self.current_goal_[1] - self.current_pose_.pose.pose.position.y
                distance = (x_diff**2 + y_diff**2) ** 0.5
                self.get_logger().info("Distance to goal: %f" % distance)
                if distance < self.goal_tolerance_:
                    self.get_logger().info("Goal reached, sending new goal")
                    pick_next = True
                if self.uptime_ > self.goal_timeout_:
                    self.get_logger().info("Timeout to goal, sending new goal")
                    pick_next = True
        if pick_next:
            self.uptime_ = 0.0
            x, y, z = self.get_next_goal()
            self.current_goal_ = (x, y, z)
            self.go_to_goal(x, y, z)
            self.uptime_ = self.uptime_ - self.goal_timeout_
        self.get_logger().info(
            "Time until goal timeout: %f" % (self.goal_timeout_ - self.uptime_)
        )

    def load_goals_file(self, filename):
        """Load the goals from a file.

        The file should be a YAML file with a list of dictionaries with the keys x, y, and yaw.
        """
        goals = None
        try:
            with open(filename, "r") as file:
                goals = yaml.safe_load(file)
            goals = [(item["x"], item["y"], item["yaw"]) for item in goals]
        except FileNotFoundError:
            self.get_logger().error("File not found: %s" % filename)
            goals = None
        except yaml.YAMLError as e:
            self.get_logger().error("Error parsing YAML file: %s" % e)
            goals = None
        except Exception as e:
            self.get_logger().error("Error loading file: %s" % e)
            goals = None
        return goals

    def current_pose_callback(self, msg):
        """Store the current pose of the robot."""
        self.current_pose_ = msg

    def get_next_goal(self):
        """Pick a goal from the list of goal randomly with replacement."""
        return random.choice(self.goals_)

    def go_to_goal(self, x, y, yaw):
        """Publish a goal to the goal_pose topic."""
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = 0.0
        quaternion = (0.0, 0.0, 0.0, 1.0)  # quaternion_from_euler(0, 0, yaw)
        goal.pose.orientation.x = quaternion[0]
        goal.pose.orientation.y = quaternion[1]
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]
        self.goal_publisher_.publish(goal)
        self.get_logger().info("Publishing goal: (%f, %f, %f)" % (x, y, yaw))


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = RandomWalkerNode()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
