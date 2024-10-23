#!/usr/bin/env python3

import sys
import cv2
import depthai as dai
import numpy as np
import time
import argparse
import json
import blobconverter
from pathlib import Path

import rclpy
from rclpy.node import Node

# tmp_path 
tmp_path = str(Path(__file__).parent.parent.parent.parent.parent.parent.parent)
package_name = 'arcanain_depthai_ros2'
class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')
        

        # parse command line 
        parser = argparse.ArgumentParser()
        # command line for <model>.blob 
        parser.add_argument("-m", "--model", help="Provide model name or model path for inference",
                            default=f"{tmp_path}/src/{package_name}/models/tiny-yolo-v4_openvino_2021.2_6shave.blob", type=str)
        #yolov4_tiny_coco_416x416'
        # command line for <config>.json
        parser.add_argument("-c", "--config", help="Provide config path for inference",
                            default=f"{tmp_path}/src/{package_name}/json/yolov4-tiny.json", type=str)
        args, unknown = parser.parse_known_args()


        # parse <config>.json
        configPath = Path(args.config)
        print("Config Path:", configPath)
        print(str(Path(__file__).parent.parent.parent.parent.parent.parent))
        print(str(Path(__file__).parent))
        if not configPath.exists():
            raise ValueError("Path {} does not exist!".format(configPath))

        with configPath.open() as f:
            config = json.load(f)
        nnConfig = config.get("nn_config", {})

        # 入力サイズの取得
        if "input_size" in nnConfig:
            W, H = tuple(map(int, nnConfig.get("input_size").split('x')))

        # メタデータの取得
        metadata = nnConfig.get("NN_specific_metadata", {})
        classes = metadata.get("classes", {})
        coordinates = metadata.get("coordinates", {})
        anchors = metadata.get("anchors", {})
        anchorMasks = metadata.get("anchor_masks", {})
        iouThreshold = metadata.get("iou_threshold", {})
        confidenceThreshold = metadata.get("confidence_threshold", {})

        self.get_logger().info(str(metadata))

        # ラベルの取得
        nnMappings = config.get("mappings", {})
        self.labels = nnMappings.get("labels", {})

        # モデルのパスを取得
        nnPath = args.model
        if not Path(nnPath).exists():
            self.get_logger().info("No blob found at {}. Looking into DepthAI model zoo.".format(nnPath))
            nnPath = str(blobconverter.from_zoo(args.model, shaves=6, zoo_type="depthai", use_cache=True))

        # パイプラインの作成
        pipeline = dai.Pipeline()

        # ノードの定義
        camRgb = pipeline.create(dai.node.ColorCamera)
        detectionNetwork = pipeline.create(dai.node.YoloDetectionNetwork)
        nnOut = pipeline.create(dai.node.XLinkOut)

        nnOut.setStreamName("nn")

        # プロパティの設定
        camRgb.setPreviewSize(W, H)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        camRgb.setFps(40)

        # ネットワークの設定
        detectionNetwork.setConfidenceThreshold(confidenceThreshold)
        detectionNetwork.setNumClasses(classes)
        detectionNetwork.setCoordinateSize(coordinates)
        detectionNetwork.setAnchors(anchors)
        detectionNetwork.setAnchorMasks(anchorMasks)
        detectionNetwork.setIouThreshold(iouThreshold)
        detectionNetwork.setBlobPath(nnPath)
        detectionNetwork.setNumInferenceThreads(2)
        detectionNetwork.input.setBlocking(False)

        # ノード間の接続
        camRgb.preview.link(detectionNetwork.input)
        detectionNetwork.out.link(nnOut.input)

        # デバイスへの接続とパイプラインの開始
        self.device = dai.Device(pipeline)
        self.qDet = self.device.getOutputQueue(name="nn", maxSize=4, blocking=False)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.startTime = time.monotonic()
        self.counter = 0

    def detection_to_tuple(self, detection):
        """
        dai.Detectionオブジェクトをタプルに変換する関数。

        Returns:
        - tuple: (label: str, confidence: float, bbox: tuple(float, float, float, float))
        """
        # ラベル名の取得
        label_id = detection.label
        label_name = self.labels[label_id] if label_id < len(self.labels) else str(label_id)

        # 信頼度の取得
        confidence = detection.confidence  # 0.0 ~ 1.0

        # バウンディングボックスの取得（正規化された座標：0.0 ~ 1.0）
        xmin = detection.xmin
        ymin = detection.ymin
        xmax = detection.xmax
        ymax = detection.ymax

        # タプルにまとめる
        detection_tuple = (
            label_name,            # ラベル名（str）
            confidence,            # 信頼度（float）
            (xmin, ymin, xmax, ymax)  # バウンディングボックス座標（tuple of float）
        )

        return detection_tuple

    def timer_callback(self):
        inDet = self.qDet.tryGet()

        if inDet is not None:
            detections = inDet.detections
            self.counter += 1
            detection_tuples = []
            for detection in detections:
                # 検出結果をタプルに変換
                detection_tuple = self.detection_to_tuple(detection)
                detection_tuples.append(detection_tuple)

            # ターミナルに出力
            for dt in detection_tuples:
                label_name, confidence, bbox = dt
                self.get_logger().info(f"Label: {label_name}, Confidence: {confidence:.2f}")
                self.get_logger().info(f"BBox: xmin={bbox[0]:.2f}, ymin={bbox[1]:.2f}, xmax={bbox[2]:.2f}, ymax={bbox[3]:.2f}")
                self.get_logger().info("-" * 30)

    def destroy_node(self):
        super().destroy_node()
        self.device.close()

def main(args=None):
    rclpy.init(args=args)

    node = YoloDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
