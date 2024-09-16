import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import depthai as dai

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(Image, 'camera/image', 10)
        self.pipeline = dai.Pipeline()
        cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        
        xout_rgb = self.pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.video.link(xout_rgb.input)

        self.device = dai.Device(self.pipeline)
        self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

        self.timer = self.create_timer(1.0 / 30.0, self.publish_image)

    def publish_image(self):
        frame = self.q_rgb.get().getCvFrame()
        if frame is not None:
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.height = frame.shape[0]
            msg.width = frame.shape[1]
            msg.encoding = "bgr8"
            msg.is_bigendian = False
            msg.step = frame.shape[1] * 3
            msg.data = frame.tobytes()
            self.publisher_.publish(msg)
        else:
            self.get_logger().error('Failed to retrieve frame from camera.')

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

