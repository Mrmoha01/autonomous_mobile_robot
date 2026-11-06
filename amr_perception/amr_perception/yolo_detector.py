"""Optional ROS 2 node: run a pretrained YOLO model on the camera image.

Not needed for the main demo (the OpenCV detector is lighter). Requires extra packages::

    pip install ultralytics --break-system-packages

Note: the simulated primitives (coloured spheres/boxes) are not what YOLO was trained on,
so this node is mainly useful if you later add realistic models to the world.
"""

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class YoloDetector(Node):

    def __init__(self, model):
        super().__init__('yolo_detector')
        self.declare_parameter('confidence', 0.4)
        self.declare_parameter('every_n_frames', 3)

        self.model = model
        self.confidence = self.get_parameter('confidence').value
        self.every_n = max(1, self.get_parameter('every_n_frames').value)
        self.count = 0
        self.bridge = CvBridge()

        self.create_subscription(Image, 'camera/image_raw', self.on_image,
                                 qos_profile_sensor_data)
        self.pub = self.create_publisher(Image, 'camera/image_yolo', 10)
        self.get_logger().info('YOLO detector ready -> camera/image_yolo')

    def on_image(self, msg):
        self.count += 1
        if self.count % self.every_n:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        result = self.model(frame, conf=self.confidence, verbose=False)[0]
        out = self.bridge.cv2_to_imgmsg(result.plot(), 'bgr8')
        out.header = msg.header
        self.pub.publish(out)


def main(args=None):
    try:
        from ultralytics import YOLO
    except ImportError:
        print('ultralytics is not installed. Run: pip install ultralytics '
              '--break-system-packages')
        return

    rclpy.init(args=args)
    node = YoloDetector(YOLO('yolov8n.pt'))
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
