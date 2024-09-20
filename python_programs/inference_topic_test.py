#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose,BoundingBox2D
from cv_bridge import CvBridge
import cv2
import depthai as dai
import numpy as np
from pathlib import Path
import sys
import time

class DepthAIDetector(Node):
    def __init__(self):
        super().__init__('depthai_detector')
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/image_Raw', 10)
        self.detection_pub = self.create_publisher(Detection2DArray, '/Detections', 10)
        
        # YOLOv4-tiny model path
        nnPath = str((Path(__file__).parent / Path('models/yolo-v4-tiny-tf_openvino_2021.4_6shave.blob')).resolve().absolute())
        if not Path(nnPath).exists():
            raise FileNotFoundError(f'Required file/s not found, please run "install_requirements.py"')

        # Tiny YOLOv4 label map
        self.labelMap = [
        "person",         "bicycle",    "car",           "motorbike",     "aeroplane",   "bus",           "train",
        "truck",          "boat",       "traffic light", "fire hydrant",  "stop sign",   "parking meter", "bench",
        "bird",           "cat",        "dog",           "horse",         "sheep",       "cow",           "elephant",
        "bear",           "zebra",      "giraffe",       "backpack",      "umbrella",    "handbag",       "tie",
        "suitcase",       "frisbee",    "skis",          "snowboard",     "sports ball", "kite",          "baseball bat",
        "baseball glove", "skateboard", "surfboard",     "tennis racket", "bottle",      "wine glass",    "cup",
        "fork",           "knife",      "spoon",         "bowl",          "banana",      "apple",         "sandwich",
        "orange",         "broccoli",   "carrot",        "hot dog",       "pizza",       "donut",         "cake",
        "chair",          "sofa",       "pottedplant",   "bed",           "diningtable", "toilet",        "tvmonitor",
        "laptop",         "mouse",      "remote",        "keyboard",      "cell phone",  "microwave",     "oven",
        "toaster",        "sink",       "refrigerator",  "book",          "clock",       "vase",          "scissors",
        "teddy bear",     "hair drier", "toothbrush"
        ]


        syncNN = True

        # Create pipeline
        self.pipeline = dai.Pipeline()

        # Define sources and outputs
        self.camRgb = self.pipeline.create(dai.node.ColorCamera)
        self.detectionNetwork = self.pipeline.create(dai.node.YoloDetectionNetwork)
        xoutRgb = self.pipeline.create(dai.node.XLinkOut)
        nnOut = self.pipeline.create(dai.node.XLinkOut)

        xoutRgb.setStreamName("rgb")
        nnOut.setStreamName("nn")

        # Properties
        self.camRgb.setPreviewSize(416, 416)
        self.camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        self.camRgb.setInterleaved(False)
        self.camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        self.camRgb.setFps(30)

        # Network specific settings
        self.detectionNetwork.setConfidenceThreshold(0.5)
        self.detectionNetwork.setNumClasses(80)
        self.detectionNetwork.setCoordinateSize(4)
        self.detectionNetwork.setAnchors([10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319])
        self.detectionNetwork.setAnchorMasks({"side26": [1, 2, 3], "side13": [3, 4, 5]})
        self.detectionNetwork.setIouThreshold(0.5)
        self.detectionNetwork.setBlobPath(nnPath)
        self.detectionNetwork.setNumInferenceThreads(2)
        self.detectionNetwork.input.setBlocking(False)

        # Linking
        self.camRgb.preview.link(self.detectionNetwork.input)
        if syncNN:
            self.detectionNetwork.passthrough.link(xoutRgb.input)
        else:
            self.camRgb.preview.link(xoutRgb.input)

        self.detectionNetwork.out.link(nnOut.input)

        # Connect to device and start pipeline
        self.device = dai.Device(self.pipeline)
        self.qRgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        self.qDet = self.device.getOutputQueue(name="nn", maxSize=4, blocking=False)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def frameNorm(self, frame, bbox):
        normVals = np.full(len(bbox), frame.shape[0])
        normVals[::2] = frame.shape[1]
        return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

    def timer_callback(self):
        inRgb = self.qRgb.get()
        inDet = self.qDet.get()

        if inRgb is not None:
            frame = inRgb.getCvFrame()
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(ros_image)
        
        if inDet is not None:
            detections = inDet.detections
            print(detections)

            # TODO somehow I cant modify the data type from camera inference to massage
            
            '''
            detection_array = Detection2DArray()
            for detection in detections:
                detection_msg = Detection2D()
                bbox = self.frameNorm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
                
                detection_msg.bbox.size_x = float(bbox[2] - bbox[0])
                detection_msg.bbox.size_y = float(bbox[3] - bbox[1])

                # 修正箇所： center.x, center.y を position.x, position.y に変更
                detection_msg.bbox.center.position.x = float((bbox[0] + bbox[2]) / 2)
                detection_msg.bbox.center.position.y = float((bbox[1] + bbox[3]) / 2)
                
                # ObjectHypothesis インスタンスを作成して設定                
                hypothesis = ObjectHypothesisWithPose()
                print(dir(hypothesis))  # hypothesisオブジェクトの属性を表示
                hypothesis.class_id= int(detection.label)
                hypothesis.score = detection.confidence
                
                hypothesis_with_pose = ObjectHypothesisWithPose()
                hypothesis_with_pose.hypothesis = hypothesis

                detection_msg.results.append(hypothesis_with_pose)    
                #detection_msg.results.append(hypothesis)

                detection_array.detections.append(detection_msg)
    '''
        #self.detection_pub.publish(detections)
        
def main(args=None):
    rclpy.init(args=args)
    node = DepthAIDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
