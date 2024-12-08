from depthai_sdk import Replay

# 録画データのディレクトリパスを指定
replay = Replay('recorded_data/1-18443010C17C121300')

# 再生の設定
replay.set_loop(True)  # ループ再生を有効にする場合
replay.set_speed(1.0)  # 再生速度を1.0倍に設定

# 再生を開始
replay.run()
