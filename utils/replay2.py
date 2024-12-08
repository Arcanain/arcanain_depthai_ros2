from depthai_sdk import Replay

# 録画データのパスを指定
replay = Replay('recorded_data/1-18443010C17C121300/CAM_A_bitstream.mp4')  # ファイル名を確認して修正

# ループ再生を有効化（必要に応じて）
replay.set_loop(True)

# 再生を開始
replay.run()
