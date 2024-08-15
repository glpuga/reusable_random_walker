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

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import rclpy
import yaml


class RandomWalkRecorderNode(Node):
    def __init__(self):
        """Create a RandomWalkRecorderNode."""
        super().__init__("random_walk_recorder")

        self.declare_parameter("goals_file_path", "goals.yaml")

        self.goals_file_ = (
            self.get_parameter("goals_file_path").get_parameter_value().string_value
        )

        self.goals_data_ = []

        self.current_pose_subscriber_ = self.create_subscription(
            PoseStamped, "goal_pose", self.current_pose_callback, 1
        )

    def current_pose_callback(self, msg):
        """Store the current pose of the robot."""
        next_goal = msg
        self.goals_data_.append(
            {
                "x": next_goal.pose.position.x,
                "y": next_goal.pose.position.y,
                "yaw": 0.0,
            }
        )
        self.get_logger().info(f"Recorded goal: {self.goals_data_[-1]}")
        self.save_goals_data()

    def save_goals_data(self):
        """Save the goals data to a yaml file."""
        with open(self.goals_file_, "w") as file:
            yaml.dump(self.goals_data_, file)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = RandomWalkRecorderNode()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
