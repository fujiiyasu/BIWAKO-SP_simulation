import math

class parameter:
    def __init__(self):

        self.max_simulation_time = 24 * 3600  # 最大シミュレーション時間 [s]（24時間）

        # 電圧
        self.V = 12.0

        # 消費電力モデル
        # T200スラスタ（12V）強度p → 電流I の2次近似
        # I(p) = a*p² + b*p + c  (p >= dead_zone)
        # T200 Public Performance Data (12V) から同定
        self.current_a         =  22.43   # [A]
        self.current_b         =  -5.53   # [A]
        self.current_c         =   0.46   # [A]
        self.thruster_dead_zone =  0.1    # デッドゾーン（p < 0.1 → I=0）
        self.P_other            =  3.0    # [W] 搭載機器の消費電力

        # 太陽光発電モデル（論文式(1)）
        self.eta_pv   = 0.166    # パネル変換効率
        self.A_pv     = 0.3016   # パネル面積 [m²]
        self.eta_mppt = 0.95     # MPPT変換効率

        # 制御ゲイン
        self.distance_Kp = 1.5
        self.distance_Kd = 3.0
        self.bearing_Kp  = 0.3
        self.bearing_Kd  = 2.0

        # 最大推力
        self.STRAGHT_MAX_THRUST = 12.0
        self.KEEP_MAX_THRUST    = 12.0 * (math.sqrt(2.0) / 2)
        self.MAX_THRUST = 0.5

        # シミュレーション設定
        self.TIME_STEP = 1000   # [ms]

        # ウェイポイントファイル
        self.way_point_file = './waypoint/test.csv'

        # 制御モード
        self.mode = 2
        # 0: 直進モード, 2: 定点維持モード

        # センサ設定
        self.gps_error_mode = True
        # False: ノイズなしGPS使用, True: ノイズありGPS使用

        # 外乱設定
        self.disturbance_mode = True
        self.disturbance_strength    = 0.2    # 外乱強度の上限 [m/s]
        self.disturbance_angle       = 0.0    # 外乱方向（未使用、ランダム時）
        self.disturbance_seed     = 42     # シード値（再現性のため）
        self.disturbance_interval = 900.0  # 外乱更新間隔 [s]（15分）

        # ラップ数
        self.max_lap = 1

        # 定点維持戦略
        self.main_distance_tolerance = 10.0   # 通常許容誤差 dTA-TP [m]
        self.temp_distance_tolerance = 1.5   # 一時目標点の許容誤差 [m]
        self.overshoot_r             = 1.5   # 一時目標点のオーバーシュート距離 [m]

        # SOC連動モード閾値
        self.SOC_high = 0.8
        self.SOC_low  = 0.3

        # モードごとの許容誤差
        self.d_high   = 3.0   # 高精度モード [m]
        self.d_std    = 5.0   # 標準モード [m]
        self.d_save   = 10.0  # 省エネモード [m]

        # ヒステリシス（チャタリング防止）
        self.hysteresis_time = 60.0  # モード継続確認時間 [s]

        # SOC連動戦略切替
        self.adaptive_mode = True  # True: SOC連動あり, False: 固定戦略（標準モード固定）

        # 日射量モデル
        self.G_max    = 950.0    # 最大日射量 [W/m²]（シナリオに応じて変更）
        self.t_rise   = 5.0      # 日の出時刻 [h]
        self.t_set    = 19.0     # 日の入り時刻 [h]
        # 日射量モデル
        # self.G_max    = 550.0    # 最大日射量 [W/m²]（シナリオに応じて変更）
        # self.t_rise   = 7.0      # 日の出時刻 [h]
        # self.t_set    = 17.0     # 日の入り時刻 [h]
        
        self.sim_start_hour = 0.0  # シミュレーション開始時刻 [h]（深夜）

        # 雲係数モデル（S3シナリオ用）
        self.cloud_mode          = True   # True: kcloud有効（S3）, False: 常に1.0
        self.cloud_period        = 7200.0  # 雲の周期 [s]（2時間）
        self.cloud_k_min         = 0.2     # kcloud の最小値（雲が最も厚いとき）
        self.cloud_k_max         = 1.0     # kcloud の最大値（快晴のとき）