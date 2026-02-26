import math

class parameter:
    def __init__(self):

        # 電圧
        self.V = 12.0

        # 制御ゲイン
        self.distance_Kp = 1.5
        self.distance_Kd = 3.0
        self.bearing_Kp  = 0.3
        self.bearing_Kd  = 2.0

        # 最大推力
        self.STRAGHT_MAX_THRUST = 12.0
        self.KEEP_MAX_THRUST    = 10.0 * (math.sqrt(2.0) / 2)

        # シミュレーション設定
        self.TIME_STEP = 100   # [ms]

        # ウェイポイントファイル
        self.way_point_file = './waypoint/test.csv'

        # 制御モード
        self.mode = 2
        # 0: 直進モード, 2: 定点維持モード

        # センサ設定
        self.gps_error_mode = False
        # False: ノイズなしGPS使用, True: ノイズありGPS使用

        # 外乱設定
        self.disturbance_mode = False
        # False: 外乱なし（動作確認用）, True: 外乱あり

        # 到達判定距離
        self.main_distance_tolerance = 3.0  # [m]

        # ラップ数
        self.max_lap = 1
