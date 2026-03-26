#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2

def frame_to_imgmsg(frame):
    msg = Image()
    msg.height, msg.width = frame.shape[:2]
    msg.encoding = 'bgr8'
    msg.step = frame.shape[1] * 3
    msg.data = frame.tobytes()
    return msg

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.pub = self.create_publisher(Image, 'camera/image_raw', 10)

        self.cap = cv2.VideoCapture(0)  # webcam
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            self.pub.publish(frame_to_imgmsg(frame))

def main():
    rclpy.init()
    node = CameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

