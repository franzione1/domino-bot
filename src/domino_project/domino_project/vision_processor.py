#!/usr/bin/env python3
"""
SmartDominoVision — perceives domino pieces on the table and publishes game-state.

Published topics:
  /domino_detections  (MarkerArray)
      Marker id=0, ns='center' : the central piece (closest to table centre)
      Marker id=1, ns='target' : the piece sharing a color with the central one
      Each marker encodes:
        pose.position.{x,y}   → real-world X,Y on the table
        pose.orientation.{z,w} → sin/cos of the piece yaw (half-angle encoding)
        color.{r,g,b}          → 1/0 flags: r=ROSSO, g=VERDE, b=BLU
        color.a                → matching color code (1=ROSSO, 2=VERDE, 3=BLU)
  /domino_pose  (Pose)  — legacy: target piece pose for backward compat.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Table centre in world coordinates (must match spawn positions)
TABLE_CENTER_X = 0.50
TABLE_CENTER_Y = 0.00
# Physical constants — must match robot_mover.cpp and models
# Reference project dimensions: 0.048 x 0.024 x 0.0075 m (lying flat)
TABLE_TOP_Z    = 1.30
DOMINO_H       = 0.0075
DOMINO_REST_Z  = TABLE_TOP_Z + DOMINO_H / 2.0   # 1.30375 m
# Minimum centre-to-centre distance (m) for two pieces to be considered SEPARATE
# (not yet touching).  Must exceed robot's CONTACT_OFFSET + 0.01 = 0.048 + 0.01 = 0.058 m
# to avoid proposing already-placed pieces as targets.
MIN_PIECE_SEPARATION = 0.06

class SmartDominoVision(Node):
    def __init__(self):
        super().__init__('smart_domino_vision')
        self.declare_parameter('camera.cam_x', 0.7)
        self.declare_parameter('camera.cam_y', 0.0)
        self.declare_parameter('camera.img_center_x', 400)
        self.declare_parameter('camera.img_center_y', 400)
        self.declare_parameter('camera.scale_x', -0.0020)
        self.declare_parameter('camera.scale_y', -0.0020)
        self.CAMERA_X = self.get_parameter('camera.cam_x').value
        self.CAMERA_Y = self.get_parameter('camera.cam_y').value
        self.IMG_CENTER_X = int(self.get_parameter('camera.img_center_x').value)
        self.IMG_CENTER_Y = int(self.get_parameter('camera.img_center_y').value)
        self.SCALE_X = self.get_parameter('camera.scale_x').value
        self.SCALE_Y = self.get_parameter('camera.scale_y').value

        self.COLORS = {
            "ROSSO": [
                (np.array([0, 70, 50]), np.array([10, 255, 255])),
                (np.array([160, 70, 50]), np.array([180, 255, 255]))
            ],
            "BLU": [
                (np.array([90, 50, 50]), np.array([140, 255, 255]))
            ],
            "VERDE": [
                (np.array([35, 50, 50]), np.array([85, 255, 255]))
            ]
        }
        
        self.COLOR_CODES = { "ROSSO": 1.0, "VERDE": 2.0, "BLU": 3.0 }

        qos_policy = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.subscription = self.create_subscription(Image, '/table_camera/image_raw', self.image_callback, qos_profile=qos_policy)
        
        # Legacy pose topic
        self.coord_pub = self.create_publisher(Pose, '/domino_pose', 10)
        # Rich game-state topic: two markers (center + target)
        self.detections_pub = self.create_publisher(MarkerArray, '/domino_detections', 10)
        
        self.bridge = CvBridge()
        self.last_print_time = 0
        self.last_detection_time = 0  # throttle detection publishing

        # Create the display window on a dedicated GUI thread so it stays
        # responsive even when the ROS callback is busy processing.
        cv2.namedWindow("Smart Vision", cv2.WINDOW_AUTOSIZE)
        cv2.startWindowThread()

        self.get_logger().info('VISION: Pronta. Logica di Gioco Domino ATTIVA.')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        blobs = []

        for color_name, ranges in self.COLORS.items():
            mask = np.zeros(hsv_image.shape[:2], dtype="uint8")
            for (lower, upper) in ranges:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv_image, lower, upper))

            kernel = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                if cv2.contourArea(c) > 80:
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        rx, ry = self.pixel_to_real(cx, cy)

                        rect = cv2.minAreaRect(c)
                        (_, _), (width, height), angle = rect
                        if width < height:
                            angle = angle - 90
                        yaw = math.radians(angle)

                        blobs.append({
                            'colore': color_name, 'cx': cx, 'cy': cy,
                            'rx': rx, 'ry': ry, 'yaw': yaw, 'rect': rect
                        })

        if not blobs:
            try:
                cv2.imshow("Smart Vision", cv_image)
                cv2.waitKey(1)
            except cv2.error:
                pass
            return

        # ── Step 1: group blobs that belong to the SAME domino piece ──────────
        # Two blobs whose centres are within DOMINO_HALF_LEN of each other are
        # halves of the same tile.  We represent each tile as the centroid of
        # its two halves plus the list of colours it carries.
        DOMINO_HALF_LEN = 0.035  # metres – max centre-to-centre of two halves of one tile
                                  # piece long axis = 0.048 m, half-centres are ~0.024 apart
        used = [False] * len(blobs)
        pieces = []  # list of dicts: {rx, ry, yaw, colors: [c1], [c1,c2]}

        for i, b in enumerate(blobs):
            if used[i]:
                continue
            piece_blobs = [b]
            used[i] = True
            for j, b2 in enumerate(blobs):
                if used[j]:
                    continue
                dist = math.sqrt((b['rx'] - b2['rx']) ** 2 + (b['ry'] - b2['ry']) ** 2)
                if dist < DOMINO_HALF_LEN:
                    piece_blobs.append(b2)
                    used[j] = True

            rx = sum(p['rx'] for p in piece_blobs) / len(piece_blobs)
            ry = sum(p['ry'] for p in piece_blobs) / len(piece_blobs)
            # Compute piece yaw (long-axis direction) from the LINE connecting
            # the two half-centres.  Each visual half is a 24×24 mm square, so
            # minAreaRect gives an unreliable angle for individual blobs.
            # atan2 of the inter-blob vector gives the long axis reliably.
            if len(piece_blobs) >= 2:
                dx = piece_blobs[1]['rx'] - piece_blobs[0]['rx']
                dy = piece_blobs[1]['ry'] - piece_blobs[0]['ry']
                yaw = math.atan2(dy, dx)
            else:
                yaw = piece_blobs[0]['yaw']  # fallback for single-blob detection
            colors = list({p['colore'] for p in piece_blobs})
            pieces.append({'rx': rx, 'ry': ry, 'yaw': yaw, 'colors': colors, 'blobs': piece_blobs})

        if not pieces:
            try:
                cv2.imshow("Smart Vision", cv_image)
                cv2.waitKey(1)
            except cv2.error:
                pass
            return

        # ── Step 2: identify the CENTER piece (closest to table origin) ───────
        pieces.sort(key=lambda p: math.sqrt((p['rx'] - TABLE_CENTER_X) ** 2 + (p['ry'] - TABLE_CENTER_Y) ** 2))
        center_piece = pieces[0]

        # ── Step 3: find the TARGET piece that shares at least one color ──────
        target_piece = None
        matching_color = None
        for piece in pieces[1:]:
            shared = set(center_piece['colors']) & set(piece['colors'])
            if shared:
                dist = math.sqrt((piece['rx'] - center_piece['rx']) ** 2 + (piece['ry'] - center_piece['ry']) ** 2)
                if dist > MIN_PIECE_SEPARATION:
                    target_piece = piece
                    matching_color = sorted(shared)[0]  # deterministic choice
                    break

        # ── Step 4: draw debug overlay ────────────────────────────────────────
        def draw_piece(piece, bgr_color, label):
            for b in piece['blobs']:
                box = np.int0(cv2.boxPoints(b['rect']))
                cv2.drawContours(cv_image, [box], 0, bgr_color, 2)
            px, py = int(piece['blobs'][0]['cx']), int(piece['blobs'][0]['cy'])
            cv2.putText(cv_image, label, (px - 40, py - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr_color, 2)

        draw_piece(center_piece, (0, 255, 255),
                   f"CTR {'+'.join(center_piece['colors'])}")

        if target_piece and matching_color:
            draw_piece(target_piece, (0, 255, 0),
                       f"TGT {'+'.join(target_piece['colors'])} match={matching_color}")

            # ── Step 5: build and publish the MarkerArray ─────────────────────
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if now_sec - self.last_detection_time < 0.5:
                # Throttle: don't flood robot_mover faster than 2 Hz
                try:
                    cv2.imshow("Smart Vision", cv_image)
                    cv2.waitKey(1)
                except cv2.error:
                    pass
                return
            self.last_detection_time = now_sec

            stamp = self.get_clock().now().to_msg()
            ma = MarkerArray()

            def make_marker(piece, ns, mid, match_color):
                """Pack a piece into a Marker.
                color.{r,g,b} ∈ {0,1} flags for ROSSO/VERDE/BLU.
                color.a       = matching color code (COLOR_CODES).
                orientation   = yaw half-angle quaternion (z/w only).
                """
                m = Marker()
                m.header = Header(frame_id='world')
                m.header.stamp = stamp
                m.ns = ns
                m.id = mid
                m.type = Marker.CUBE
                m.action = Marker.ADD
                m.pose.position.x = piece['rx']
                m.pose.position.y = piece['ry']
                m.pose.position.z = DOMINO_REST_Z
                m.pose.orientation.x = 0.0
                m.pose.orientation.y = 0.0
                m.pose.orientation.z = math.sin(piece['yaw'] / 2.0)
                m.pose.orientation.w = math.cos(piece['yaw'] / 2.0)
                m.scale.x = 0.06
                m.scale.y = 0.03
                # scale.z repurposed: world-frame angle (rad) from piece centre
                # to its matching-colour half.  The robot uses this to place
                # pieces end-to-end with matching colours physically touching.
                # (Visualisation: cube will appear flat in z — acceptable.)
                match_blob = next(
                    (b for b in piece['blobs'] if b['colore'] == match_color), None)
                if match_blob:
                    m.scale.z = math.atan2(
                        match_blob['ry'] - piece['ry'],
                        match_blob['rx'] - piece['rx'])
                else:
                    m.scale.z = piece['yaw']  # fallback: assume match on +X side
                m.color.r = 1.0 if 'ROSSO' in piece['colors'] else 0.0
                m.color.g = 1.0 if 'VERDE' in piece['colors'] else 0.0
                m.color.b = 1.0 if 'BLU' in piece['colors'] else 0.0
                m.color.a = self.COLOR_CODES.get(match_color, 0.0)
                return m

            # id=0 ns='center' → the piece that stays on the table
            ma.markers.append(make_marker(center_piece, 'center', 0, matching_color))
            # id=1 ns='target' → the piece the robot must pick and place
            ma.markers.append(make_marker(target_piece, 'target', 1, matching_color))
            self.detections_pub.publish(ma)

            # Legacy /domino_pose (target piece only)
            p = Pose()
            p.position.x = target_piece['rx']
            p.position.y = target_piece['ry']
            p.position.z = self.COLOR_CODES.get(matching_color, 0.0)
            p.orientation.x = 0.0
            p.orientation.y = 0.0
            p.orientation.z = math.sin(target_piece['yaw'] / 2.0)
            p.orientation.w = math.cos(target_piece['yaw'] / 2.0)
            self.coord_pub.publish(p)

            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.last_print_time > 2.0:
                self.get_logger().info(
                    f"CENTER {'+'.join(center_piece['colors'])} @ ({center_piece['rx']:.3f},{center_piece['ry']:.3f}) | "
                    f"TARGET {'+'.join(target_piece['colors'])} @ ({target_piece['rx']:.3f},{target_piece['ry']:.3f}) "
                    f"match={matching_color}")
                self.last_print_time = now

        try:
            cv2.imshow("Smart Vision", cv_image)
            cv2.waitKey(1)
        except cv2.error:
            pass

    def pixel_to_real(self, u, v):
        real_x = self.CAMERA_X + (v - self.IMG_CENTER_Y) * self.SCALE_X
        real_y = self.CAMERA_Y + (u - self.IMG_CENTER_X) * self.SCALE_Y
        return real_x, real_y

def main(args=None):
    rclpy.init(args=args)
    node = SmartDominoVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    try:
        cv2.destroyAllWindows()
    except cv2.error:
        pass

if __name__ == '__main__':
    main()