import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import depthai as dai
import numpy as np


class OakDCameraNode(Node):
    def __init__(self):
        super().__init__('image_topic_test')

        # -----------------------------
        # DepthAI v3 pipeline
        # -----------------------------
        self.pipeline = dai.Pipeline()

        # OAK-DのRGBカメラは通常CAM_A
        self.cam_rgb = self.pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_A
        )

        # 320×240、BGR、30 FPSで出力
        # BGR888i: BGR interleaved
        self.rgb_output = self.cam_rgb.requestOutput(
            size=(320, 240),
            type=dai.ImgFrame.Type.BGR888i,
            fps=30,
        )

        # DepthAI v3ではXLinkOutは作成しない
        # ROSタイマーを止めないため、non-blocking queueにする
        self.rgb_queue = self.rgb_output.createOutputQueue(4, False)

        # パイプライン開始
        self.pipeline.start()

        self.get_logger().info(
            f'DepthAI version: {dai.__version__}'
        )
        self.get_logger().info(
            'OAK-D RGB camera started: 320, 240, 30 FPS'
        )

        # -----------------------------
        # ROS 2 publisher
        # -----------------------------
        self.image_pub = self.create_publisher(
            Image,
            'camera/image_raw',
            3,
        )

        # 約30 FPS
        self.timer = self.create_timer(
            1.0 / 30.0,
            self.timer_callback,
        )

    def timer_callback(self):
        # non-blockingで最新フレームを取得
        in_rgb = self.rgb_queue.tryGet()

        # まだフレームが届いていない場合
        if in_rgb is None:
            return

        try:
            # DepthAI ImgFrame → OpenCV形式のBGR画像
            frame = in_rgb.getCvFrame()

            # 念のため連続したメモリ配置にする
            frame = np.ascontiguousarray(frame)

            # -----------------------------
            # OpenCV frame → ROS Image
            # -----------------------------
            msg = Image()

            # この時刻はROSメッセージ作成時刻
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'oak_rgb_camera_optical_frame'

            msg.height = frame.shape[0]
            msg.width = frame.shape[1]
            msg.encoding = 'bgr8'
            msg.is_bigendian = False

            # 1行あたりのバイト数
            msg.step = frame.strides[0]

            # numpy配列をbytesに変換
            msg.data = frame.tobytes()

            self.image_pub.publish(msg)

        except Exception as error:
            self.get_logger().error(
                f'Failed to process camera frame: {error}'
            )

    def stop_camera(self):
        """DepthAI pipelineを停止する。"""
        if hasattr(self, 'pipeline'):
            try:
                self.pipeline.stop()
                self.pipeline.wait()
            except Exception as error:
                self.get_logger().warning(
                    f'Failed to stop DepthAI pipeline cleanly: {error}'
                )


def main(args=None):
    rclpy.init(args=args)

    oakd_camera_node = None

    try:
        oakd_camera_node = OakDCameraNode()
        rclpy.spin(oakd_camera_node)

    except KeyboardInterrupt:
        pass

    except Exception as error:
        if oakd_camera_node is not None:
            oakd_camera_node.get_logger().error(str(error))
        else:
            print(f'Failed to start OAK-D camera: {error}')

    finally:
        if oakd_camera_node is not None:
            oakd_camera_node.stop_camera()
            oakd_camera_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()