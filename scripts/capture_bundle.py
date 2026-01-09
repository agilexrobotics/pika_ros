#!/usr/bin/env python3
import os
import csv
import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
# from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import String

# === CONFIGURACIÓN (ajústala si quieres) ===
OUTPUT_DIR = os.path.expanduser('~/capturas_ros/Manzana')  # dónde guardas los datos
IMAGE_TOPICS = {
    'depth':   '/gripper/camera/aligned_depth_to_color/image_raw',
    'color':   '/gripper/camera/color/image_raw',
    'fisheye': '/gripper/camera_fisheye/color/image_raw',
}
CSV_TOPICS = {
    'joint_states': '/joint_states',
    'end_pose':     '/piper_ctrl_single_node/end_pose',
    'status':       '/data_capture_status',
}
N_IMAGES_PER_TOPIC = 10  # cuántas imágenes guardas por tópico

def ns_now():
    return int(time.time() * 1e9)

def ensure_dir(p):
    os.makedirs(p, exist_ok=True)

def img_stamp_ns(msg):
    # Si el mensaje trae header, úsalo; si no, reloj del sistema
    try:
        return msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
    except Exception:
        return ns_now()

class CaptureBundle(Node):
    def __init__(self):
        super().__init__('capture_bundle')

        # Carpetas
        self.dir_images = os.path.join(OUTPUT_DIR, 'images')
        self.dir_csv    = os.path.join(OUTPUT_DIR, 'csv')
        ensure_dir(self.dir_images)
        ensure_dir(self.dir_csv)

        # CSV files & writers
        self.f_joint   = open(os.path.join(self.dir_csv, 'joint_states.csv'), 'a', newline='')
        self.w_joint   = csv.writer(self.f_joint)
        self._write_header_if_empty(self.f_joint, self.w_joint,
            ['stamp_ns','joint_name','position','velocity','effort'])

        self.f_pose    = open(os.path.join(self.dir_csv, 'end_pose.csv'), 'a', newline='')
        self.w_pose    = csv.writer(self.f_pose)
        self._write_header_if_empty(self.f_pose, self.w_pose,
            ['stamp_ns','x','y','z','qx','qy','qz','qw'])

        self.f_status  = open(os.path.join(self.dir_csv, 'data_capture_status.csv'), 'a', newline='')
        self.w_status  = csv.writer(self.f_status)
        self._write_header_if_empty(self.f_status, self.w_status, ['stamp_ns','text'])

        self.f_depthm  = open(os.path.join(self.dir_csv, 'depth_metrics.csv'), 'a', newline='')
        self.w_depthm  = csv.writer(self.f_depthm)
        self._write_header_if_empty(self.f_depthm, self.w_depthm,
            ['stamp_ns','filename','center_z_m','min_z_m','median_z_m','valid_px'])

        # Contadores de imágenes por tópico
        self.bridge = CvBridge()
        self.img_counts = {k: 0 for k in IMAGE_TOPICS.keys()}

        # QoS: imágenes = BEST_EFFORT, resto = RELIABLE
        qos_img = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=50)

        # Suscripciones imágenes
        self.sub_img = {}
        for nick, topic in IMAGE_TOPICS.items():
            self.sub_img[nick] = self.create_subscription(
                Image, topic, self._make_image_cb(nick, topic), qos_img)
            self.get_logger().info(f'📷 Subscrito IMG: {topic} -> {nick}')

        # Suscripciones CSV
        self.sub_joint = self.create_subscription(JointState, CSV_TOPICS['joint_states'],
                                                  self.cb_joint, qos_rel)
        self.get_logger().info(f'🧩 Subscrito CSV: {CSV_TOPICS["joint_states"]} (JointState)')

        # self.sub_pose  = self.create_subscription(PoseStamped, CSV_TOPICS['end_pose'],
        #                                           self.cb_pose, qos_rel)
        self.sub_pose  = self.create_subscription(Pose, CSV_TOPICS['end_pose'], 
                                                  self.cb_pose, qos_rel)
        self.get_logger().info(f'🧭 Subscrito CSV: {CSV_TOPICS["end_pose"]} (PoseStamped)')

        # /data_capture_status: si no es String, igualmente lo registramos como texto
        self.sub_status = self.create_subscription(String, CSV_TOPICS['status'],
                                                   self.cb_status_string, qos_rel)
        self.get_logger().info(f'📣 Subscrito CSV: {CSV_TOPICS["status"]} (String)')

        self.get_logger().info(f'Guardando en: {OUTPUT_DIR}')
        self.get_logger().info(f'N imágenes por tópico: {N_IMAGES_PER_TOPIC}')

    # ---- Utilidades CSV ----
    def _write_header_if_empty(self, fhandle, writer, header):
        if fhandle.tell() == 0:
            writer.writerow(header)
            fhandle.flush()

    # ---- Callbacks CSV ----
    def cb_joint(self, msg: JointState):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec if msg.header else ns_now()
        # una fila por articulación
        n = len(msg.name)
        for i in range(n):
            name = msg.name[i]
            pos  = msg.position[i] if i < len(msg.position) else ''
            vel  = msg.velocity[i] if i < len(msg.velocity) else ''
            eff  = msg.effort[i]   if i < len(msg.effort)   else ''
            self.w_joint.writerow([stamp, name, pos, vel, eff])
        self.f_joint.flush()

    # def cb_pose(self, msg: PoseStamped):
    #     stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec if msg.header else ns_now()
    #     p = msg.pose.position
    #     q = msg.pose.orientation
    #     self.w_pose.writerow([stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
    #     self.f_pose.flush()
    def cb_pose(self, msg: Pose):
        # No trae header: sello con tiempo de llegada
        stamp = ns_now()
        p = msg.position
        q = msg.orientation
        self.w_pose.writerow([stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        self.f_pose.flush()



    def cb_status_string(self, msg: String):
        # Si en tu sistema /data_capture_status NO es String, cambia la suscripción
        # o crea un callback adicional. Este guarda el texto con timestamp.
        stamp = ns_now()
        self.w_status.writerow([stamp, msg.data])
        self.f_status.flush()

    # ---- Callbacks de IMAGEN ----
    def _make_image_cb(self, nick, topic):
        def cb(msg: Image):
            if self.img_counts[nick] >= N_IMAGES_PER_TOPIC:
                return
            stamp = img_stamp_ns(msg)
            subdir = os.path.join(self.dir_images, nick)
            ensure_dir(subdir)

            # Depth 16UC1 → mantener 16 bits; RGB → bgr8
            if msg.encoding in ('16UC1','mono16'):
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                fname  = os.path.join(subdir, f'{nick}_{stamp}_{self.img_counts[nick]:04d}.png')
                cv2.imwrite(fname, cv_img)

                # Métricas de profundidad (m) ignorando 0
                arr = cv_img.astype(np.uint16)
                valid = arr[arr > 0]
                if valid.size > 0:
                    center = arr[arr.shape[0]//2, arr.shape[1]//2] / 1000.0  # mm→m
                    min_m  = np.min(valid) / 1000.0
                    med_m  = np.median(valid) / 1000.0
                    self.w_depthm.writerow([stamp, fname, center, min_m, med_m, int(valid.size)])
                    self.f_depthm.flush()
                else:
                    self.w_depthm.writerow([stamp, fname, '', '', '', 0])
                    self.f_depthm.flush()

            else:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                fname  = os.path.join(subdir, f'{nick}_{stamp}_{self.img_counts[nick]:04d}.png')
                cv2.imwrite(fname, cv_img)

            self.img_counts[nick] += 1
            self.get_logger().info(f'💾 {topic} -> {fname} ({self.img_counts[nick]}/{N_IMAGES_PER_TOPIC})')

            # Si terminamos todos, opcionalmente cerramos el nodo
            if all(self.img_counts[k] >= N_IMAGES_PER_TOPIC for k in self.img_counts):
                self.get_logger().info('🎉 Imágenes completas en todos los tópicos. Ctrl+C para salir.')
        return cb

    # ---- Shutdown limpio ----
    def destroy_node(self):
        try:
            self.f_joint.close()
            self.f_pose.close()
            self.f_status.close()
            self.f_depthm.close()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = CaptureBundle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':

    main()
