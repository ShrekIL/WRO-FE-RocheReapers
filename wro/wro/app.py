#!/usr/bin/env python3
# encoding: utf-8
# ????(color tracking)
import os
import os.path
import cv2
import math
import queue
import rclpy
import time
import threading
import numpy as np
import sdk.pid as pid
import sdk.common as common
from rclpy.node import Node
from app.common import Heart
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from app.common import ColorPicker
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, Trigger
from interfaces.srv import SetPoint, SetFloat64
from ros_robot_controller_msgs.msg import MotorsState, SetPWMServoState, PWMServoState

class OjbectTrackingNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        self.set_callback = False
        self.color_picker = None
        self.tracker = None
        self.is_running = False
        self.threshold = 0.1
        self.dist_threshold = 0.3
        self.lock = threading.RLock()
        self.image_sub = None
        self.result_image = None
        self.image_height = None
        self.image_width = None
        self.bridge = CvBridge()

        self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.servo_state_pub = self.create_publisher(SetPWMServoState, 'ros_robot_controller/pwm_servo/set_state', 1)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

        self.image_sub = self.create_subscription(Image, '/ascamera/camera_publisher/rgb0/image', self.image_callback, 1)  # ?????(subscribe to the camera)


        self.skip_img = 50

    def get_node_state(self, request, response):
        response.success = True
        return response        
    
    def publish_servo_state(self, positions):
        servo_state = PWMServoState()
        servo_state.id = [3]
        servo_state.position = positions
        data = SetPWMServoState()
        data.state = [servo_state]
        data.duration = 0.02
        self.get_logger().info(f'Publishing servo state: {data}')
        self.servo_state_pub.publish(data)

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        self.image_height, self.image_width = rgb_image.shape[:2]
        self.get_logger().info('\033[1;32m%s\033[0m' % 'image callback')

        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        result_image = np.copy(rgb_image)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'image copied')


        if self.skip_img != 0:
            self.skip_img -= 1
            return
        if not os.path.isfile("/home/ubuntu/result.jpg"):
            self.publish_servo_state([2100])
            t = Twist()
            t.linear.x = 1.0
            self.mecanum_pub.publish(t)
            cv2.imwrite('/home/ubuntu/result.jpg', result_image)
            time.sleep(30)
            self.publish_servo_state([1500])
            t = Twist()
            self.mecanum_pub.publish(t)




def main():
    node = OjbectTrackingNode('object_tracking')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
