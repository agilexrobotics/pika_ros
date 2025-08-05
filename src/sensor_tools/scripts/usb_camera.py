#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import tf
from geometry_msgs.msg import TransformStamped


class RosOperator:
    def __init__(self):
        self.cap = None
        self.camera_port = None
        self.camera_hz = None
        self.camera_height = None
        self.camera_width = None
        self.bridge = None
        self.camera_color_publisher = None
        self.camera_config_publisher = None
        self.camera_frame_id = None
        self.init_ros()

    def init_ros(self):
        rospy.init_node('camera_fisheye', anonymous=True)
        self.camera_port = rospy.get_param("~camera_port", 0)
        self.camera_hz = rospy.get_param("~camera_fps", 30)
        self.camera_height = rospy.get_param("~camera_height", 480)
        self.camera_width = rospy.get_param("~camera_width", 640)
        self.camera_frame_id = rospy.get_param("~camera_frame_id", "camera_rgb")
        if self.camera_frame_id.startswith('/'):
            self.camera_frame_id = self.camera_frame_id[1:]
        self.bridge = CvBridge()
        self.camera_color_publisher = rospy.Publisher("/camera_rgb/color/image_raw", Image, queue_size=10)
        self.camera_config_publisher = rospy.Publisher("/camera_rgb/color/camera_info", CameraInfo, queue_size=10)

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
                continue
        return False

    def publish_camera_color(self, color):
        img = self.bridge.cv2_to_imgmsg(color, "bgr8")
        img.header.stamp = rospy.Time.now()
        img.header.frame_id = self.camera_frame_id + "_color"
        self.camera_color_publisher.publish(img)
        camera_info = CameraInfo()
        camera_info.header.frame_id = self.camera_frame_id + "_color"
        camera_info.header.stamp = rospy.Time.now()
        self.camera_config_publisher.publish(camera_info)
        br = tf.TransformBroadcaster()
        translation = (0.0, 0.0, 0.0)
        rotation = (0.0, 0.0, 0.0, 1.0)
        br.sendTransform(translation, rotation, rospy.Time.now(), self.camera_frame_id, self.camera_frame_id + "_color")

    def run(self):
        rate = rospy.Rate(self.camera_hz)
        while self.cap.isOpened() and not rospy.is_shutdown():
            ret, frame = self.cap.read()
            self.publish_camera_color(frame)
            rate.sleep()

def main():
    ros_operator = RosOperator()
    if ros_operator.init_camera():
        print("camera opened")
        ros_operator.run()
    else:
        print("camera error")

if __name__ == '__main__':
    main()

