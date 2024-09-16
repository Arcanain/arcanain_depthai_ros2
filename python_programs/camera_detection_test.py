import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import depthai as dai


def main():

    pipeline = dai.Pipeline()

    # Create nodes, configure them and link them together

    # Connect to the device and upload the pipeline to it
    with dai.Device(pipeline) as device:
        # Print MxID, USB speed, and available cameras on the device
        print('Connect Successed!!!')
        print('--------')
        print('MxId:',device.getDeviceInfo().getMxId())
        print('USB speed:',device.getUsbSpeed())
        print('Connected cameras:',device.getConnectedCameras())
        print('--------')
if __name__ == '__main__':
    main()

