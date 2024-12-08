import os
from depthai_sdk import OakCamera, RecordType
import time

# ディレクトリ作成
os.makedirs('./recorded_data', exist_ok=True)

try:
    with OakCamera() as oak:
        # カメラの初期化
        color = oak.create_camera('color', resolution='1080P', fps=20, encode='H265')
        left = oak.create_camera('left', resolution='480P', fps=20, encode='H265')
        right = oak.create_camera('right', resolution='480P', fps=20, encode='H265')

        # 動画を保存する設定
        oak.record([color.out.encoded, left.out.encoded, right.out.encoded], './recorded_data', RecordType.VIDEO)

        # カラー映像をリアルタイムで表示
        oak.visualize([color.out.camera], scale=2/3, fps=True)

        # 録画開始
        oak.start(blocking=False)

        print("録画を開始しました。Ctrl + C で終了します。")
        while True:  # 無限ループで録画を続ける
            time.sleep(1)

except KeyboardInterrupt:  # Ctrl + C でスクリプトを終了
    print("\n録画を終了しました。保存先: ./recorded_data")
