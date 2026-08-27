#!/usr/bin/env python3

import time

import cv2
import numpy as np
import easyocr
import torch

torch.set_num_threads(1)
torch.set_num_interop_threads(1)
cv2.setNumThreads(1)
import rclpy
#from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class OcrNode(Node):

    def __init__(self):
        super().__init__('plate_ocr_node')

        #self.bridge = CvBridge()
        self.reader = easyocr.Reader(
            ['en'],
            gpu=False,
            detector=False
        )

        self.subscription = self.create_subscription(
            Image,
            '/detections/crop',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.publisher = self.create_publisher(
            String,
            '/plate_number',
            10
        )

        # OCRは0.5秒に1回
        self.ocr_interval = 2.0
        self.last_ocr_time = 0.0

        self.get_logger().info('Plate OCR node started.')

    def image_callback(self, msg):
        now = time.monotonic()

        if now - self.last_ocr_time < self.ocr_interval:
            return

        self.last_ocr_time = now

        image = np.frombuffer(msg.data, dtype=np.uint8)
        image = image.reshape((msg.height, msg.step))
        image = image[:, :msg.width * 3]
        image = image.reshape((msg.height, msg.width, 3)).copy()

        # 小さい文字を読みやすくするため3倍に拡大
        image = cv2.resize(
            image,
            None,
            fx=3,
            fy=3,
            interpolation=cv2.INTER_CUBIC
        )

        h, w = image.shape[:2]

        # プレート外周の枠や小さい文字を除外して中央部分だけ使う
        x1 = int(w * 0.15)
        x2 = int(w * 0.85)
        y1 = int(h * 0.15)
        y2 = int(h * 0.85)

        image = image[y1:y2, x1:x2]

        h, w = image.shape[:2]

        results = self.reader.recognize(
            image,
            horizontal_list=[[0, w, 0, h]],
            free_list=[],
            allowlist='0123456789',
            detail=1
        )

        output = String()

        if not results:
            output.data = 'UNKNOWN'
        else:
            best = max(results, key=lambda x: x[2])

            text = best[1]
            confidence = float(best[2])

            if confidence >= 0.95 and len(text) == 3 and text.isdigit():
                output.data = text
            else:
                output.data = 'UNKNOWN'

            self.get_logger().info(
                f'OCR={text}, confidence={confidence:.3f}, result={output.data}'
            )

        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)

    node = OcrNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
PY