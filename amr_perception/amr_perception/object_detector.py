"""ROS 2 node: detect coloured objects in the camera image and place them on the map.

Pipeline
  1. OpenCV colour segmentation finds blobs in ``camera/image_raw``.
  2. The blob's pixel column + camera intrinsics give a bearing angle.
  3. The LiDAR range at that bearing gives the distance.
  4. The 2D point is transformed into ``target_frame`` (``map``) via TF.
  5. Repeated detections are merged into persistent objects shown as RViz markers.
"""

import json
import math

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from amr_perception.detection import (ObjectTracker, find_blobs, pixel_to_bearing,
                                      range_at_bearing, transform_point)

# label -> (marker type, size x/y/z, rgb)
MARKER_STYLE = {
    'ball': (Marker.SPHERE, (0.3, 0.3, 0.3), (0.9, 0.1, 0.1)),
    'box': (Marker.CUBE, (0.3, 0.3, 0.3), (0.1, 0.2, 0.9)),
    'barrel': (Marker.CYLINDER, (0.3, 0.3, 0.4), (0.1, 0.8, 0.2)),
}
DRAW_BGR = {'red': (0, 0, 255), 'green': (0, 200, 0), 'blue': (255, 80, 0)}


class ObjectDetector(Node):

    def __init__(self):
        super().__init__('object_detector')

        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('min_area', 60.0)
        self.declare_parameter('lidar_window_deg', 2.5)
        self.declare_parameter('max_range', 7.0)
        self.declare_parameter('object_radius', 0.15)
        self.declare_parameter('merge_distance', 0.6)
        self.declare_parameter('min_hits', 3)
        self.declare_parameter('label_red', 'ball')
        self.declare_parameter('label_green', 'barrel')
        self.declare_parameter('label_blue', 'box')

        self.target_frame = self.get_parameter('target_frame').value
        self.min_area = self.get_parameter('min_area').value
        self.window = math.radians(self.get_parameter('lidar_window_deg').value)
        self.max_range = self.get_parameter('max_range').value
        self.object_radius = self.get_parameter('object_radius').value
        self.labels = {
            'red': self.get_parameter('label_red').value,
            'green': self.get_parameter('label_green').value,
            'blue': self.get_parameter('label_blue').value,
        }

        self.tracker = ObjectTracker(
            merge_distance=self.get_parameter('merge_distance').value,
            min_hits=self.get_parameter('min_hits').value)

        self.bridge = CvBridge()
        self.camera_info = None
        self.scan = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, 'camera/camera_info', self.on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Image, 'camera/image_raw', self.on_image,
                                 qos_profile_sensor_data)

        self.image_pub = self.create_publisher(Image, 'camera/image_annotated', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'object_markers', 10)
        self.json_pub = self.create_publisher(String, 'detected_objects', 10)
        self.create_timer(0.5, self.publish_objects)

        self.get_logger().info(
            f'Object detector ready (objects are placed in the "{self.target_frame}" frame).')

    # ------------------------------------------------------------------ inputs
    def on_info(self, msg):
        self.camera_info = msg

    def on_scan(self, msg):
        self.scan = msg

    def on_image(self, msg):
        if self.camera_info is None or self.scan is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        annotated = frame.copy()
        fx = self.camera_info.k[0]
        cx = self.camera_info.k[2]
        scan = self.scan

        for blob in find_blobs(frame, self.min_area):
            label = self.labels[blob.color]
            bearing = pixel_to_bearing(blob.cx, cx, fx)
            distance = range_at_bearing(
                scan.ranges, scan.angle_min, scan.angle_increment, bearing,
                self.window, scan.range_min, scan.range_max)

            text = label if distance is None else f'{label} {distance:.1f}m'
            color = DRAW_BGR[blob.color]
            cv2.rectangle(annotated, (blob.x, blob.y), (blob.x + blob.w, blob.y + blob.h),
                          color, 2)
            cv2.putText(annotated, text, (blob.x, max(12, blob.y - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

            if distance is not None and distance <= self.max_range:
                self.register(label, bearing, distance, scan.header.frame_id)

        out = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        out.header = msg.header
        self.image_pub.publish(out)

    # --------------------------------------------------------------- tracking
    def register(self, label, bearing, distance, scan_frame):
        """Convert (bearing, range) into the target frame and update the tracker."""
        # The LiDAR hits the object's surface; shift to the object centre.
        r = distance + self.object_radius
        point = (r * math.cos(bearing), r * math.sin(bearing), 0.0)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, scan_frame, Time(), timeout=Duration(seconds=0.1))
        except TransformException as exc:
            self.get_logger().warn(
                f'No transform {self.target_frame} <- {scan_frame} yet: {exc}',
                throttle_duration_sec=5.0)
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        x, y, _ = transform_point((t.x, t.y, t.z), (q.x, q.y, q.z, q.w), point)
        track = self.tracker.update(label, x, y)
        if track.hits == self.tracker.min_hits:
            self.get_logger().info(
                f'New {label} confirmed at ({track.x:.2f}, {track.y:.2f}) '
                f'in {self.target_frame}')

    # ---------------------------------------------------------------- outputs
    def publish_objects(self):
        tracks = self.tracker.confirmed()
        if not tracks:
            return

        markers = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, track in enumerate(tracks):
            mtype, size, rgb = MARKER_STYLE.get(
                track.label, (Marker.SPHERE, (0.3, 0.3, 0.3), (0.8, 0.8, 0.8)))

            body = Marker()
            body.header.frame_id = self.target_frame
            body.header.stamp = now
            body.ns = 'objects'
            body.id = i
            body.type = mtype
            body.action = Marker.ADD
            body.pose.position = Point(x=track.x, y=track.y, z=size[2] / 2.0)
            body.pose.orientation.w = 1.0
            body.scale.x, body.scale.y, body.scale.z = size
            body.color.r, body.color.g, body.color.b = rgb
            body.color.a = 0.9
            markers.markers.append(body)

            text = Marker()
            text.header = body.header
            text.ns = 'labels'
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = Point(x=track.x, y=track.y, z=size[2] + 0.2)
            text.pose.orientation.w = 1.0
            text.scale.z = 0.2
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = f'{track.label} #{i}'
            markers.markers.append(text)

        self.marker_pub.publish(markers)

        summary = [{'id': i, 'label': t.label, 'x': round(t.x, 2), 'y': round(t.y, 2),
                    'frame': self.target_frame, 'hits': t.hits}
                   for i, t in enumerate(tracks)]
        self.json_pub.publish(String(data=json.dumps(summary)))


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
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
