# Copyright 2021 OROCA
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

import time
from msg_srv_action_interface_example.msg import ArithmeticArgument
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

class Calculator(Node):

  def __init__(self):
    super().__init__('calculator')
    self.argument_a = 0.0
    self.argument_b = 0.0

    self.callback_group = ReentrantCallbackGroup()

    self.declare_parameter('qos_depth', 10)
    qos_depth = self.get_parameter('qos_depth').value

    QOS_RKL10V = QoSProfile(
      reliability = QoSReliabilityPolicy.RELIABLE,
      history = QoSHistoryPolicy.KEEP_LAST,
      depth=qos_depth,
      durability=QoSDurabilityPolicy.VOLATILE)

    self.arithmetic_argument_subscriber = self.create_subscription(
      ArithmeticArgument,
      'arithmetic_argument',
      self.get_arithmetic_argument,
      QOS_RKL10V,
      callback_group=self.callback_group)

  def get_arithmetic_argument(self, msg):
    self.argument_a = msg.argument_a
    self.argument_b = msg.argument_b
    self.get_logger().info('Timestamp of the message: {0}'.format(msg.stamp))
    self.get_logger().info('Subscribed argument a: {0}'.format(self.argument_a))
    self.get_logger().info('Subscribed argument b: {0}'.format(self.argument_b))

