#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import array

class ImageCompressor(Node):
    def __init__(self):
        super().__init__("compress_camera_node")

        self.declare_parameter('topic_namespace', "camera_left")
        self.topic_namespace = self.get_parameter('topic_namespace').get_parameter_value().string_value
        self.get_logger().info(f"topic_namespace: {self.topic_namespace}")

        self.img_bridge = CvBridge()
        self.depth_bridge = CvBridge()
        color_topic = f"/{self.topic_namespace}/color/image_raw"
        depth_topic = f"/{self.topic_namespace}/aligned_depth_to_color/image_raw"
        self.image_compressed_pub = self.create_publisher(CompressedImage, f"{color_topic}/compressed", 30)
        self.depth_compressed_pub = self.create_publisher(CompressedImage, f"{depth_topic}/compressed", 30)
        self.image_sub = self.create_subscription(Image, color_topic, self.image_callback, 30)
        self.depth_sub = self.create_subscription(Image, depth_topic, self.depth_callback, 30)

    def image_callback(self, msg=Image()):
        cv_image = self.img_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        ret, jpeg_image = cv2.imencode('.jpg', cv_image, encode_param)

        compressed_image_msg = CompressedImage()
        compressed_image_msg.header = msg.header
        compressed_image_msg.format = "jpeg"
        compressed_image_msg.data = array.array('B', jpeg_image.tobytes())
        self.image_compressed_pub.publish(compressed_image_msg)

    def depth_callback(self, msg=Image()):
        cv_depth = self.depth_bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
        success, encoded_depth = cv2.imencode(".png", cv_depth, [int(cv2.IMWRITE_PNG_COMPRESSION), 1])

        compressed_msg = CompressedImage()
        compressed_msg.header = msg.header
        compressed_msg.format = "png"
        compressed_msg.data = array.array('B', encoded_depth.tobytes())
        self.depth_compressed_pub.publish(compressed_msg)

def main(args=None):
    print(args)
    rclpy.init(args=args)
    compress_camera_node = ImageCompressor()
    try:
        rclpy.spin(compress_camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        compress_camera_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()