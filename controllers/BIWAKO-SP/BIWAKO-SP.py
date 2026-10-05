import csv
import math
import random
from datetime import datetime
from geopy.distance import geodesic
from controller import Supervisor

from const import parameter
parameter = parameter()

# Supervisorのインスタンスを作成
supervisor = Supervisor()

class BIWAKO_SP:
    def __init__(self):
        self.STRAIGHT_MAX_THRUST = parameter.STRAGHT_MAX_THRUST
        self.KEEP_MAX_THRUST     = parameter.KEEP_MAX_THRUST

        self.time_step = int(supervisor.getBasicTimeStep())

        # スラスタ
        self.thruster1 = supervisor.getDevice('thruster1')
        self.thruster2 = supervisor.getDevice('thruster2')
        self.thruster3 = supervisor.getDevice('thruster3')
        self.thruster4 = supervisor.getDevice('thruster4')

        # GPS（ノイズあり）
        self.gps_noise       = supervisor.getDevice('gps')
        self.gps_noise_value = [0, 0, 0]
        # GPS（ノイズなし）
        self.gps       = supervisor.getDevice('a_gps')
        self.gps_value = [0, 0, 0]
        self.speed     = 0.0

        # コンパス
        self.compass       = supervisor.getDevice('compass')
        self.compass_value = [0, 0, 0]

        # 電流・電圧・電力
        self.battery_capacity_Wh = 12.0 * 20.0  # 12V × 20Ah = 240Wh
        self.SOC = 1.0                           # 初期SOC 100%
        self.A = 0.0
        self.V = 12.0
        self.W = 0.0
        self.P_solar = 0.0

        # ウェイポイント
        self.waypoint_list = []
        self.waypoint_num  = 0
        self.waypoint      = [0.0, 0.0]

        # ラップ
        self.lap_count = 0
        self.max_lap   = 5
        
        # 定点維持戦略
        self.station_target   = None   # 定点維持の目標点 [lat, lon]
        self.temp_target      = None   # 一時目標点 [lat, lon]（Noneなら通常制御）
        self.is_deviated      = False  # 許容誤差範囲から逸脱中フラグ

        # 雲係数
        self.kcloud = 1.0
        self.next_cloud_update = 0.0

        # 制御ログ用
        self.thrust             = 0.0
        self.thruster_direction = [0.0, 0.0, 0.0, 0.0]
        self.distance           = 0.0
        self.angle              = 0.0

        self.control_mode        = 'standard'  # 現在のモード high / standard / save
        self.mode_candidate      = None        # 切替候補モード
        self.mode_candidate_time = 0.0         # 候補モードに入った時刻

    # ─────────────────────────────────────────────────────────────
    # 初期化
    # ─────────────────────────────────────────────────────────────
    def init_device(self):
        thruster_list = [self.thruster1, self.thruster2,
                         self.thruster3, self.thruster4]
        for thruster in thruster_list:
            thruster.setPosition(float('inf'))
            thruster.setVelocity(0.0)

        self.gps_noise.enable(self.time_step)
        self.gps.enable(self.time_step)
        self.compass.enable(self.time_step)

        self.gps_noise_value = self.gps_noise.getValues()
        self.gps_value       = self.gps.getValues()
        self.compass_value   = self.compass.getValues()

    # ─────────────────────────────────────────────────────────────
    # ウェイポイント
    # ─────────────────────────────────────────────────────────────
    def load_waypoints_from_csv(self, file_path):
        try:
            with open(file_path, mode='r', encoding='utf-8') as file:
                csv_reader = csv.reader(file)
                for row in csv_reader:
                    if not row:
                        continue
                    try:
                        latitude  = float(row[0])
                        longitude = float(row[1])
                        self.waypoint_list.append([latitude, longitude])
                    except ValueError as e:
                        print(f"Skipping row: {row}, Error: {e}")
            self.waypoint = self.waypoint_list[self.waypoint_num]
            print(f"Waypoints loaded: {self.waypoint_list}")
        except FileNotFoundError:
            print(f"File not found: {file_path}")
        except Exception as e:
            print(f"An error occurred: {e}")

    def update_waypoint(self):
        self.waypoint = self.waypoint_list[self.waypoint_num]

    def update_waypoint_num(self):
        if self.waypoint_num < len(self.waypoint_list) - 1:
            self.waypoint_num += 1
        else:
            self.lap_count += 1
            if self.lap_count >= self.max_lap:
                self.waypoint_num = -1
            else:
                self.waypoint_num = 0

    def calc_temp_target(self, curr_pos, target):
        try:
            R    = 6378137.0
            r    = parameter.overshoot_r
            late = 1.0 / R * (180 / math.pi)
            lone = 1.0 / (R * math.cos(math.radians(target[0]))) * (180 / math.pi)
            dlat = target[0] - curr_pos[0]
            dlon = target[1] - curr_pos[1]
            alpha = math.atan2(dlon / lone, dlat / late)
            lat_nt = target[0] + r * math.cos(alpha) * late
            lon_nt = target[1] + r * math.sin(alpha) * lone
            if math.isnan(lat_nt) or math.isnan(lon_nt):
                print(f"calc_temp_target: nan detected, returning station_target")
                return target
            return [lat_nt, lon_nt]
        except Exception as e:
            print(f"calc_temp_target error: {e}")
            return target

    # ─────────────────────────────────────────────────────────────
    # スラスタ制御
    # ─────────────────────────────────────────────────────────────
    def set_thruster_velocity(self, thruster_direction, thrust):
        thruster_list = [self.thruster1, self.thruster2,
                         self.thruster3, self.thruster4]
        for i, thruster in enumerate(thruster_list):
            thruster.setVelocity(thruster_direction[i] * thrust)

    def stop_thrusters(self):
        self.set_thruster_velocity([0, 0, 0, 0], 0.0)

    # ─────────────────────────────────────────────────────────────
    # センサ値の更新
    # ─────────────────────────────────────────────────────────────
    def update_gps_noise(self):
        v = self.gps_noise.getValues()
        if any(math.isnan(x) for x in v):
            return  # 前回値を維持
        self.gps_noise_value = [v[1], v[0]]

    def update_robot_position(self):
        v = self.gps.getValues()
        if any(math.isnan(x) for x in v):
            return  # 前回値を維持
        self.gps_value = [v[1], v[0]]

    def update_compass_value(self):
        v = self.compass.getValues()
        if any(math.isnan(x) for x in v):
            return  # 前回値を維持
        self.compass_value = v

    def update_speed(self):
        v = self.gps.getSpeed()
        if math.isnan(v):
            return  # 前回値を維持
        self.speed = v

    def update_thrust(self, value):
        self.thrust = value

    def update_thruster_direction(self, direction_list):
        self.thruster_direction = direction_list

    def update_distance(self, value):
        self.distance = value

    def update_angle(self, value):
        self.angle = value

    def update_robot_state(self, thrust, thruster_direction,
                           distance=None, angle=None):
        self.update_gps_noise()
        self.update_robot_position()
        self.update_compass_value()
        self.update_ampere(thrust, thruster_direction)
        self.update_power()
        self.update_speed()
        self.update_thrust(thrust)
        self.update_thruster_direction(thruster_direction)
        if distance is not None:
            self.update_distance(distance)
        if angle is not None:
            self.update_angle(angle)

    # ─────────────────────────────────────────────────────────────
    # 航法計算
    # ─────────────────────────────────────────────────────────────
    def calc_distance(self, gps_value, target=None):
        if target is None:
            target = self.waypoint
        try:
            current_position = [gps_value[0], gps_value[1]]
            distance = geodesic(current_position, target).m
            return distance
        except Exception as e:
            print(f"calc_distance error: {e}, gps={gps_value}, target={target}")
            return 0.0

    def calc_angle(self, compass_value, gps_value, target_position):
        current_heading = -math.degrees(
            math.atan2(compass_value[2], compass_value[0]))
        current_heading = (current_heading - 90 + 360) % 360

        lat1, lon1 = map(math.radians, gps_value)
        lat2, lon2 = map(math.radians, target_position)
        dlon = lon2 - lon1
        target_bearing = math.degrees(math.atan2(
            math.sin(dlon) * math.cos(lat2),
            math.cos(lat1) * math.sin(lat2)
            - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        ))
        target_bearing = (target_bearing + 360) % 360

        diff_angle = (target_bearing - current_heading + 180) % 360 - 180
        return diff_angle

    # ─────────────────────────────────────────────────────────────
    # PD制御
    # ─────────────────────────────────────────────────────────────
    def PD_distance_control(self, prev_distance, curr_distance):
        distance_diff  = curr_distance - prev_distance
        K_p = parameter.distance_Kp
        K_d = parameter.distance_Kd
        control_output = K_p * curr_distance - K_d * distance_diff
        thrust = max(min(control_output, self.KEEP_MAX_THRUST), 0)
        return thrust
    def update_control_mode(self, soc, timestamp):
        """
        SOCに基づくモード切替（ヒステリシス付き）
        """
        # 次のモードを判定
        if soc >= parameter.SOC_high:
            candidate = 'high'
        elif soc < parameter.SOC_low:
            candidate = 'save'
        else:
            candidate = 'standard'

        # 現在と同じなら即反映
        if candidate == self.control_mode:
            self.mode_candidate      = None
            self.mode_candidate_time = 0.0
            return

        # ヒステリシス：候補が変わったら時刻を記録
        if candidate != self.mode_candidate:
            self.mode_candidate      = candidate
            self.mode_candidate_time = timestamp

        # 一定時間継続したら切替
        if timestamp - self.mode_candidate_time >= parameter.hysteresis_time:
            self.control_mode        = candidate
            self.mode_candidate      = None
            self.mode_candidate_time = 0.0
            print(f"  >> Mode changed to: {self.control_mode}")

    def get_active_tolerance(self):
        if self.control_mode == 'high':
            return parameter.d_high
        elif self.control_mode == 'save':
            return parameter.d_save
        else:
            return parameter.d_std

    # ─────────────────────────────────────────────────────────────
    # 移動制御
    # ─────────────────────────────────────────────────────────────
    def straight_control(self, prev_distance, curr_distance,
                         prev_angle, curr_angle):
        def control_heading(prev_angle, curr_angle):
            angle_diff = curr_angle - prev_angle
            K_p = parameter.bearing_Kp
            K_d = parameter.bearing_Kd
            a = curr_angle
            if   0   <= a <  90: a = abs(a - 180)
            elif -180 <= a < -90: a = abs(a + 180)
            elif -90  <= a <   0: a = abs(a)
            control_output = K_p * a - K_d * angle_diff
            balance = max(min(1 - (2 * (control_output / 90)), 1), -1)
            return balance

        t = control_heading(prev_angle, curr_angle)
        if t <= 0.5:
            t = 0.5

        if   0   <= curr_angle <  90:
            direction_str = "Forward Counter-Clockwise"
            thruster_directions = [1, -t, -1, 1]
        elif  90 <= curr_angle < 180:
            direction_str = "Backward Clockwise"
            thruster_directions = [1, -1, -t, 1]
        elif -180 <= curr_angle < -90:
            direction_str = "Backward Counter-Clockwise"
            thruster_directions = [1, -1, -1, t]
        else:
            direction_str = "Forward Clockwise"
            thruster_directions = [t, -1, -1, 1]

        thrust = self.PD_distance_control(prev_distance, curr_distance)
        return thruster_directions, thrust, direction_str

    def keep_control(self, curr_angle):
        # 北: -22.5 ~ +22.5 deg
        if -22.5 <= curr_angle < 22.5:
            direction_str = "D5 South [Push T3+T4]"
            thruster_directions = [1, -1, 0, 0]

        # 北西: +22.5 ~ +67.5 deg
        elif 22.5 <= curr_angle < 67.5:
            direction_str = "D4 SouthWest [Diagonal T2+T3]"
            thruster_directions = [0, -1, 0, 1]

        # 西: +67.5 ~ +112.5 deg
        elif 67.5 <= curr_angle < 112.5:
            direction_str = "D3 West [Push T1+T3]"
            thruster_directions = [0, -1, 1, 0]

        # 南西: +112.5 ~ +157.5 deg
        elif 112.5 <= curr_angle < 157.5:
            direction_str = "D2 NorthWest [Diagonal T1+T4]"
            thruster_directions = [-1, 0, 1, 0]

        # 南: ±157.5 ~ ±180 deg
        elif curr_angle >= 157.5 or curr_angle < -157.5:
            direction_str = "D1 North [Push T1+T2]"
            thruster_directions = [0, 0, 1, -1]

        # 南東: -157.5 ~ -112.5 deg
        elif -157.5 <= curr_angle < -112.5:
            direction_str = "D8 NorthEast [Diagonal T2+T3]"
            thruster_directions = [0, 1, 0, -1]

        # 東: -112.5 ~S -67.5 deg
        elif -112.5 <= curr_angle < -67.5:
            direction_str = "D7 East [Push T2+T4]"
            thruster_directions = [1, 0, 0, -1]

        # 北東: -67.5 ~ -22.5 deg
        else:
            direction_str = "D6 SouthEast [Diagonal T1+T4]"
            thruster_directions = [1, 0, -1, 0]

        thrust = self.KEEP_MAX_THRUST # ON-OFF制御
        return thruster_directions, thrust, direction_str
    
    # ─────────────────────────────────────────────────────────────
    # 電力計算
    # ─────────────────────────────────────────────────────────────
    def calc_solar_power(self, time_s):
        """
        太陽光発電出力 [W]（論文式(1) + 雲係数式(8)）
        G(t) = G_max * sin(...) * kcloud(t)
        """
        hour   = (time_s / 3600.0) % 24.0
        t_rise = parameter.t_rise
        t_set  = parameter.t_set
        if hour <= t_rise or hour >= t_set:
            return 0.0
        G = parameter.G_max * math.sin(
            math.pi * (hour - t_rise) / (t_set - t_rise)
        )
        # ── 雲係数を乗算（cloud_mode が True のとき有効）──
        G *= self.kcloud
        return parameter.eta_pv * parameter.A_pv * G * parameter.eta_mppt
    
    def update_kcloud(self, time_s):
        """
        kcloud(t) をsin波で周期的に変動させる。
        cloud_mode=False なら常に 1.0。

        kcloud = k_min + (k_max - k_min) * (1 - sin(2π * t / T)) / 2
        ・sin が 1.0 のとき → kcloud = k_min（最も曇り）
        ・sin が-1.0 のとき → kcloud = k_max（最も晴れ）
        """
        if not parameter.cloud_mode:
            self.kcloud = 1.0
            return

        T = parameter.cloud_period
        k_min = parameter.cloud_k_min
        k_max = parameter.cloud_k_max

        sin_val = math.sin(2 * math.pi * time_s / T)
        self.kcloud = k_min + (k_max - k_min) * (1 - sin_val) / 2

    def update_SOC(self, time_s):
        delta_t_h  = (parameter.TIME_STEP / 1000) / 3600  # self.time_stepではなくparameter.TIME_STEP
        P_solar    = self.calc_solar_power(time_s)
        net_power  = P_solar - self.W
        self.SOC  += net_power * delta_t_h / self.battery_capacity_Wh
        self.SOC   = max(0.0, min(1.0, self.SOC))
        self.P_solar = P_solar

    def calc_ampere(self, thrust, thruster_direction):
        total_current = 0.0
        for d in thruster_direction:
            p = abs(d) * (thrust / (self.KEEP_MAX_THRUST/ parameter.MAX_THRUST))
            if p < parameter.thruster_dead_zone:
                continue
            total_current += (parameter.current_a * p**2
                              + parameter.current_b * p
                              + parameter.current_c)
        self.A = total_current

    def update_ampere(self, thrust, thruster_direction):
        self.calc_ampere(thrust, thruster_direction)

    def calc_power(self):
        self.W = self.A * self.V + parameter.P_other

    def update_power(self):
        self.calc_power()

    # ─────────────────────────────────────────────────────────────
    # Getter
    # ─────────────────────────────────────────────────────────────
    def get_gps_noise(self):          return self.gps_noise_value
    def get_gps_value(self):          return self.gps_value
    def get_compass_value(self):      return self.compass_value
    def get_ampere(self):             return self.A
    def get_power(self):              return self.W
    def get_waypoint(self):           return self.waypoint
    def get_waypoint_num(self):       return self.waypoint_num
    def get_thrust(self):             return self.thrust
    def get_thruster_direction(self): return self.thruster_direction
    def get_distance(self):           return self.distance
    def get_angle(self):              return self.angle
    def get_speed(self):              return self.speed

    # ─────────────────────────────────────────────────────────────
    # ログ
    # ─────────────────────────────────────────────────────────────
    def get_state(self, time_s, disturbance_strength=0.0, disturbance_angle=0.0):
        return [
            time_s,
            *self.get_gps_noise(),
            *self.get_gps_value(),
            *self.get_compass_value(),
            self.get_ampere(),
            self.get_power(),
            self.P_solar,
            self.kcloud,
            self.SOC,
            disturbance_strength,
            disturbance_angle,
            self.get_waypoint()[0],
            self.get_waypoint()[1],
            self.get_waypoint_num(),
            self.get_thrust(),
            *self.get_thruster_direction(),
            self.get_distance(),
            self.get_angle(),
            self.get_speed(),
        ]

    def save_log(self, filename, log_data):
        header = [
                    "time_s",
                    "gps_noise_lat", "gps_noise_lon",
                    "gps_lat", "gps_lon",
                    "compass_north", "compass_east", "compass_down",
                    "ampere_A", "power_W",
                    "solar_W", 
                    "kcloud",
                    "SOC",
                    "disturbance_strength", "disturbance_angle",
                    "waypoint_lat", "waypoint_lon", "waypoint_num",
                    "thrust",
                    "thruster_dir_1", "thruster_dir_2",
                    "thruster_dir_3", "thruster_dir_4",
                    "distance_m", "angle_deg", "speed_mps",
                ]
        with open(filename, mode="a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(log_data)


# ==========================================================
# メイン
# ==========================================================
robot = BIWAKO_SP()
robot.load_waypoints_from_csv(parameter.way_point_file)
robot.init_device()

TIME_STEP = parameter.TIME_STEP

# 外乱設定
fluid_node = supervisor.getFromDef("STILL_WATER")
stream_vel = fluid_node.getField("streamVelocity")

def set_disturbance_polar(strength, angle_deg):
    """
    極座標で外乱を設定
    strength: 強度 [m/s]
    angle_deg: 方向 [deg]（北=0, 東=90, NUE座標系）
    """
    angle_rad = math.radians(angle_deg)
    x = strength * math.sin(angle_rad)
    z = strength * math.cos(angle_rad)
    stream_vel.setSFVec3f([x, 0.0, z])

if parameter.disturbance_mode:
    set_disturbance_polar(parameter.disturbance_strength,
                          parameter.disturbance_angle)
else:
    set_disturbance_polar(0.0, 0.0)

log_data  = []
timestamp = 0.0

thrust             = 0.0
thruster_direction = [0, 0, 0, 0]
prev_distance      = 0.0
curr_distance      = 0.0
prev_angle         = 0.0
curr_angle         = 0.0
direction_str      = "---"

if __name__ == "__main__":
    now      = datetime.now()
    filename = now.strftime("../../results/csv/"+"%Y%m%d_%H%M%S.csv")

    robot.max_lap   = parameter.max_lap
    robot.lap_count = 0

    # GPS値が安定するまで数ステップ待機
    print("Initializing sensors...")
    for _ in range(5):
        supervisor.step(TIME_STEP)
        robot.update_robot_position()
        robot.update_gps_noise()
        robot.update_compass_value()
    print("Sensors ready.")

    # CSVから1点目を定点維持目標として設定
    robot.station_target = robot.waypoint_list[0]
    robot.temp_target    = None
    robot.is_deviated    = False

    # random.seed(parameter.disturbance_seed)
    disturbance_strength = 0.0
    disturbance_angle    = 0.0
    next_disturbance_update = 0.0  # 次の更新時刻 [s]
    
    def update_disturbance(time_s):
        strength = random.uniform(0, parameter.disturbance_strength)
        angle    = random.uniform(0, 360)
        angle_rad = math.radians(angle)
        x = strength * math.sin(angle_rad)
        z = strength * math.cos(angle_rad)
        stream_vel.setSFVec3f([x, 0.0, z])
        return strength, angle

    while supervisor.step(TIME_STEP) != -1:

        # 0. 状態更新
        robot.update_robot_state(thrust, thruster_direction,
                                curr_distance, curr_angle)
        robot.update_kcloud(timestamp)
        robot.update_SOC(timestamp + parameter.sim_start_hour * 3600)
        if parameter.disturbance_mode and timestamp >= next_disturbance_update:
            disturbance_strength, disturbance_angle = update_disturbance(timestamp)
            next_disturbance_update = timestamp + parameter.disturbance_interval
            print(f"  >> Disturbance updated: "
                f"strength={disturbance_strength:.3f} m/s  "
                f"angle={disturbance_angle:.1f} deg")

        # 1. センサ値取得
        gps_value       = robot.get_gps_value()
        gps_noise_value = robot.get_gps_noise()
        compass_value   = robot.get_compass_value()

        curr_gps = gps_noise_value if parameter.gps_error_mode else gps_value

      # 2. 制御目標の決定（一時目標 or 定点目標）
        if robot.temp_target is not None:
            active_target    = robot.temp_target
            active_tolerance = parameter.temp_distance_tolerance
            target_str       = "TEMP"
        else:
            active_target = robot.station_target
            target_str    = "STATION"
            # SOC連動モード切替
            if parameter.adaptive_mode:
                robot.update_control_mode(robot.SOC, timestamp)
            else:
                robot.control_mode = 'standard'
            active_tolerance = robot.get_active_tolerance()

        # 3. 距離・角度計算
        prev_distance = curr_distance
        curr_distance = robot.calc_distance(curr_gps, active_target)
        prev_angle    = curr_angle
        curr_angle    = robot.calc_angle(compass_value, curr_gps, active_target)

        # # ── デバッグ表示 ──────────────────────────────────────────
        # print("=" * 55)
        # print(f"[t={timestamp:6.1f}s]  Target={target_str}  "
        #     f"Deviated={robot.is_deviated}")
        # print(f"  GPS      : lat={curr_gps[0]:.8f}  "
        #     f"lon={curr_gps[1]:.8f}")
        # print(f"  Target   : lat={active_target[0]:.8f}  "
        #     f"lon={active_target[1]:.8f}")
        # print(f"  Distance : {curr_distance:7.3f} m  "
        #     f"(tolerance={active_tolerance} m)")
        # print(f"  Angle    : {curr_angle:7.2f} deg  "
        #     f"-> Direction: {direction_str}")
        # print(f"  Compass  : N={compass_value[0]:.3f}  "
        #     f"E={compass_value[1]:.3f}  "
        #     f"down={compass_value[2]:.3f}")
        # print(f"  Thrust   : {thrust:6.3f}  "
        #     f"Dir={[round(d,2) for d in thruster_direction]}")
        # print(f"  Speed    : {robot.get_speed():.3f} m/s")
        # print(f"  SOC      : {robot.SOC:.4f}  "
        #     f"Solar={robot.P_solar:.2f}W  Power={robot.W:.2f}W")
        # # ─────────────────────────────────────────────────────────

        # 4. ログ
        log_data.append(robot.get_state(timestamp,
                                disturbance_strength,
                                disturbance_angle))

        # 5. 行動判断
        if curr_distance < active_tolerance:
            if robot.temp_target is not None:
                # 一時目標点に到達 → 通常制御に戻る
                print(f"  >> Temp target reached. Back to station keeping.")
                robot.temp_target = None
                robot.is_deviated = False
            # 許容誤差内 → 停止
            robot.stop_thrusters()
            thrust             = 0.0
            thruster_direction = [0, 0, 0, 0]
            direction_str      = "STOP"
        else:
            if not robot.is_deviated:
                # 逸脱を検出した瞬間 → 一時目標点を設定
                print(f"  SOC      : {robot.SOC:.4f}  "
                     f"Solar={robot.P_solar:.2f}W  Power={robot.W:.2f}W")
                robot.is_deviated = True
                robot.temp_target = robot.calc_temp_target(curr_gps,
                                                            robot.station_target)
                print(f"  >> Deviated! Temp target set: "
                    f"lat={robot.temp_target[0]:.8f}  "
                    f"lon={robot.temp_target[1]:.8f}")
            thruster_direction, thrust, direction_str = \
                robot.keep_control(curr_angle)
            robot.set_thruster_velocity(thruster_direction, thrust)

        # 6. 終了判断
        if robot.SOC <= 0.0:
            print("  >> Battery depleted. Stopping.")
            robot.stop_thrusters()
            break

        if timestamp >= parameter.max_simulation_time:
            print("  >> Max simulation time reached. Stopping.")
            robot.stop_thrusters()
            break

        timestamp += TIME_STEP / 1000

    robot.save_log(filename, log_data)
    print(f"ログを {filename} に保存しました")
    supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)