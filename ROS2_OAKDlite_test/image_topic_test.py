import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import depthai as dai
import cv2
import numpy as np

class OakDCameraNode(Node):
    def __init__(self):
        super().__init__('image_topic_test')

        # Create a pipeline
        pipeline = dai.Pipeline()

        # Create a color camera node
        cam_rgb = pipeline.createColorCamera()
        cam_rgb.setPreviewSize(320, 240)
        cam_rgb.setInterleaved(False)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        cam_rgb.setFps(30)

        # Create an XLinkOut node for streaming the video to the host
        xout_rgb = pipeline.createXLinkOut()
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)

        # Initialize the device and start the pipeline
        self.device = dai.Device(pipeline)

        # Create a ROS2 publisher for the image data
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 3)

        # Create a timer to periodically get frames from the camera and publish them
        self.timer = self.create_timer(0.03, self.timer_callback)  # 30 FPS

    def timer_callback(self):
        # Get frames from the device
        in_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False).get()

        # Convert to OpenCV format
        frame = in_rgb.getCvFrame()

        # Convert the frame to a ROS Image message
        msg = Image()# create Image type message in ROS2
        msg.header.stamp = self.get_clock().now().to_msg() #time stamp of the message
        '''!!! This time stamp is created when creating message, not when image captured !!!'''

        # image shape
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]

        # encoding message
        msg.encoding = 'bgr8'

        msg.is_bigendian = False

        # byte per line ex)640 width -> 640*3(RGB) =1920 byte
        msg.step = frame.shape[1] * 3

        # convert OpenCV frame to Numpy array
        msg.data = np.array(frame).tobytes()

        # Publish the image message
        self.image_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    oakd_camera_node = OakDCameraNode()

    try:
        rclpy.spin(oakd_camera_node)
    except KeyboardInterrupt:
        pass

    oakd_camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
