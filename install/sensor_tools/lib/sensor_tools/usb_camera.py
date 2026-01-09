#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import threading 
import signal
import sys
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class RosOperator(Node):
    def __init__(self): 
        super().__init__('camera_fisheye')
        self.cap = None
        self.camera_port = None
        self.camera_hz = None
        self.camera_height = None
        self.camera_width = None
        self.bridge = None
        self.camera_color_publisher = None
        self.camera_config_publisher = None
        self.camera_frame_id = None
        self.tf_broadcaster = None
        self.running = False  # 添加运行状态标志
        self.camera_thread = None  # 添加线程引用
        self.init_ros()

    def init_ros(self):
        self.declare_parameter('camera_port', 22)
        self.declare_parameter('camera_fps', 30)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_frame_id', "camera_rgb")
        self.camera_port = self.get_parameter('camera_port').get_parameter_value().integer_value
        self.camera_hz = int(self.get_parameter('camera_fps').get_parameter_value().integer_value)
        self.camera_height = int(self.get_parameter('camera_height').get_parameter_value().integer_value)
        self.camera_width = int(self.get_parameter('camera_width').get_parameter_value().integer_value)
        self.camera_frame_id = self.get_parameter('camera_frame_id').get_parameter_value().string_value
        if self.camera_frame_id.startswith('/'):
            self.camera_frame_id = self.camera_frame_id[1:]
        self.bridge = CvBridge()
        self.camera_color_publisher = self.create_publisher(Image, '/camera_rgb/color/image_raw', 10)
        self.camera_config_publisher = self.create_publisher(CameraInfo, '/camera_rgb/color/camera_info', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def init_camera(self):
        symlink_path = '/dev/video' + str(self.camera_port)
        if os.path.islink(symlink_path):
            target_path = os.readlink(symlink_path)
            target_path = int(target_path[5:])
            target_paths = []
            if target_path % 2 == 1:
                target_paths.append(target_path - 1)
                target_paths.append(target_path)
            else:
                target_paths.append(target_path)
                target_paths.append(target_path - 1)
        else:
            target_paths = [int(self.camera_port)]
        for i in target_paths:
            self.cap = cv2.VideoCapture(int(i))
            self.fourcc = cv2.VideoWriter_fourcc(*'MJPG')
            self.cap.set(cv2.CAP_PROP_FOURCC, self.fourcc)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.camera_hz)
            if self.cap.isOpened():
                return True
            else:
                if self.cap:
                    self.cap.release()  # 释放失败的摄像头
                continue
        return False
    
    def run(self):
        rate = self.create_rate(self.camera_hz)
        self.running = True
        try:
            while self.cap.isOpened() and rclpy.ok() and self.running:
                ret, frame = self.cap.read()
                if ret:
                    self.publish_camera_color(frame)
                rate.sleep()
        except Exception as e:
            self.get_logger().error(f"Camera error: {e}")
        finally:
            self.cleanup_camera()

    def cleanup_camera(self):
        """清理摄像头资源"""
        if self.cap and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info("Camera released")
    
    def stop(self):
        """停止摄像头操作"""
        self.running = False
        if self.camera_thread and self.camera_thread.is_alive():
            self.camera_thread.join(timeout=2.0)  # 等待线程结束

    def publish_camera_color(self, color):
        img = self.bridge.cv2_to_imgmsg(color, "bgr8")
        img.header.stamp = self.get_clock().now().to_msg()
        img.header.frame_id = self.camera_frame_id + "_color"
        self.camera_color_publisher.publish(img)
        camera_info = CameraInfo()
        camera_info.header.frame_id = self.camera_frame_id + "_color"
        camera_info.header.stamp = self.get_clock().now().to_msg()
        self.camera_config_publisher.publish(camera_info)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.camera_frame_id
        t.child_frame_id = self.camera_frame_id + "_color"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)


# 全局变量用于信号处理
ros_operator_instance = None

def signal_handler(signum, frame):
    """信号处理函数"""
    print(f"\nReceived signal {signum}, shutting down gracefully...")
    if ros_operator_instance:
        ros_operator_instance.stop()
    rclpy.shutdown()
    sys.exit(0)

def main():
    global ros_operator_instance
    
    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)   # Ctrl+C
    signal.signal(signal.SIGTERM, signal_handler)  # kill命令
    
    rclpy.init()
    ros_operator_instance = RosOperator()
    
    try:
        if ros_operator_instance.init_camera():
            print("camera opened")
            ros_operator_instance.camera_thread = threading.Thread(target=ros_operator_instance.run)
            ros_operator_instance.camera_thread.daemon = True  # 设置为守护线程
            ros_operator_instance.camera_thread.start()
            rclpy.spin(ros_operator_instance)
        else:
            print("camera error")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # 确保资源清理
        if ros_operator_instance:
            ros_operator_instance.stop()
        rclpy.shutdown()
        print("Program terminated, resources cleaned up")


if __name__ == '__main__':
    main()



# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, CameraInfo
# from cv_bridge import CvBridge, CvBridgeError
# import cv2
# import os
# import threading 
# from tf2_ros import TransformBroadcaster
# from geometry_msgs.msg import TransformStamped


# class RosOperator(Node):
#     def __init__(self): 
#         super().__init__('camera_fisheye')
#         self.cap = None
#         self.camera_port = None
#         self.camera_hz = None
#         self.camera_height = None
#         self.camera_width = None
#         self.bridge = None
#         self.camera_color_publisher = None
#         self.camera_config_publisher = None
#         self.camera_frame_id = None
#         self.tf_broadcaster = None
#         self.init_ros()

#     def init_ros(self):
#         self.declare_parameter('camera_port', 22)
#         self.declare_parameter('camera_fps', 30)
#         self.declare_parameter('camera_height', 480)
#         self.declare_parameter('camera_width', 640)
#         self.declare_parameter('camera_frame_id', "camera_rgb")
#         self.camera_port = self.get_parameter('camera_port').get_parameter_value().integer_value
#         self.camera_hz = int(self.get_parameter('camera_fps').get_parameter_value().integer_value)
#         self.camera_height = int(self.get_parameter('camera_height').get_parameter_value().integer_value)
#         self.camera_width = int(self.get_parameter('camera_width').get_parameter_value().integer_value)
#         self.camera_frame_id = self.get_parameter('camera_frame_id').get_parameter_value().string_value
#         self.bridge = CvBridge()
#         self.camera_color_publisher = self.create_publisher(Image, '/camera_rgb/color/image_raw', 10)
#         self.camera_config_publisher = self.create_publisher(CameraInfo, '/camera_rgb/color/camera_info', 10)
#         self.tf_broadcaster = TransformBroadcaster(self)

#     def init_camera(self):
#         symlink_path = '/dev/video' + str(self.camera_port)
#         if os.path.islink(symlink_path):
#             target_path = os.readlink(symlink_path)
#             target_path = int(target_path[5:])
#             target_paths = []
#             if target_path % 2 == 1:
#                 target_paths.append(target_path - 1)
#                 target_paths.append(target_path)
#             else:
#                 target_paths.append(target_path)
#                 target_paths.append(target_path - 1)
#         else:
#             target_paths = [int(self.camera_port)]
#         for i in target_paths:
#             self.cap = cv2.VideoCapture(int(i))
#             self.fourcc = cv2.VideoWriter_fourcc(*'MJPG')
#             self.cap.set(cv2.CAP_PROP_FOURCC, self.fourcc)
#             self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
#             self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
#             self.cap.set(cv2.CAP_PROP_FPS, self.camera_hz)
#             if self.cap.isOpened():
#                 return True
#             else:
#                 continue
#         return False
    
#     def run(self):
#         rate = self.create_rate(self.camera_hz)
#         while self.cap.isOpened() and rclpy.ok():
#             ret, frame = self.cap.read()
#             self.publish_camera_color(frame)
#             rate.sleep()

#     def publish_camera_color(self, color):
#         img = self.bridge.cv2_to_imgmsg(color, "bgr8")
#         img.header.frame_id = "camera"
#         img.header.stamp = self.get_clock().now().to_msg()
#         img.header.frame_id = self.camera_frame_id + "_color"
#         self.camera_color_publisher.publish(img)
#         camera_info = CameraInfo()
#         camera_info.header.frame_id = self.camera_frame_id + "_color"
#         camera_info.header.stamp = self.get_clock().now().to_msg()
#         self.camera_config_publisher.publish(camera_info)
#         t = TransformStamped()
#         t.header.stamp = self.get_clock().now().to_msg()
#         t.header.frame_id = self.camera_frame_id
#         t.child_frame_id = self.camera_frame_id + "_color"
#         t.transform.translation.x = 0.0
#         t.transform.translation.y = 0.0
#         t.transform.translation.z = 0.0
#         t.transform.rotation.x = 0.0
#         t.transform.rotation.y = 0.0
#         t.transform.rotation.z = 0.0
#         t.transform.rotation.w = 1.0
#         self.tf_broadcaster.sendTransform(t)


# def main():
#     rclpy.init()
#     ros_operator = RosOperator()
#     if ros_operator.init_camera():
#         print("camera opened")
#         thread = threading.Thread(target=ros_operator.run)
#         thread.start()
#         rclpy.spin(ros_operator)
#     else:
#         print("camera error")
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
