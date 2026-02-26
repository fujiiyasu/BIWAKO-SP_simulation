import csv
import math
from datetime import datetime
from geopy.distance import geodesic
from controller import Supervisor

from const import parameter
parameter = parameter()

# Supervisorのインスタンスを作成
supervisor = Supervisor()

class BIWAKO_X:
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
        self.A = 0.0
        self.V = 12.0
        self.W = 0.0

        # ウェイポイント
        self.waypoint_list = []
        self.waypoint_num  = 0
        self.waypoint      = [0.0, 0.0]

        # ラップ
        self.lap_count = 0
        self.max_lap   = 5

        # 制御ログ用
        self.thrust             = 0.0
        self.thruster_direction = [0.0, 0.0, 0.0, 0.0]
        self.distance           = 0.0
        self.angle              = 0.0

        # ヨー安定化制御用
        self.target_heading = None  # 初回センサ読み取り後に設定
        self.yaw_Kp         = 0.05  # ヨーP制御ゲイン（要調整）

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
        self.gps_noise_value = [v[1], v[0]]

    def update_robot_position(self):
        v = self.gps.getValues()
        self.gps_value = [v[1], v[0]]

    def update_compass_value(self):
        self.compass_value = self.compass.getValues()

    def update_speed(self):
        self.speed = self.gps.getSpeed()

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
    def calc_distance(self, gps_value):
        try:
            current_position = [gps_value[0], gps_value[1]]
            distance = geodesic(current_position, self.waypoint).m
            return distance
        except Exception as e:
            print(f"calc_distance error: {e}")
            return 0.0

    def calc_heading(self, compass_value):
        """
        コンパス値から現在の絶対方位を計算 [0, 360)
        NUE座標系: compass = North方向のロボットローカル座標での表現
          compass[0]=X(North成分), compass[2]=Z(East成分)
        正しい方位 = atan2(compass[0], -compass[2])
        """
        heading = math.degrees(math.atan2(compass_value[0], -compass_value[2]))
        heading = (heading + 360) % 360
        return heading

    def calc_angle(self, compass_value, gps_value, target_position):
        # ロボット前方向(ローカル-X,-Z)のワールド方位
        cx = compass_value[0]
        cz = compass_value[2]
        sqrt2 = math.sqrt(2)
        front_north = (-cx - cz) / sqrt2
        front_east  = ( cx - cz) / sqrt2
        current_heading = math.degrees(math.atan2(front_east, front_north))
        current_heading = (current_heading + 360) % 360

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
    def PD_distance_control(self, prev_distance, curr_distance,
                            max_thrust=None):
        if max_thrust is None:
            max_thrust = self.STRAIGHT_MAX_THRUST
        distance_diff  = curr_distance - prev_distance
        K_p = parameter.distance_Kp
        K_d = parameter.distance_Kd
        control_output = K_p * curr_distance - K_d * distance_diff
        thrust = max(min(control_output, max_thrust), 0)
        return thrust

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

        thrust = self.PD_distance_control(prev_distance, curr_distance,
                                          max_thrust=self.KEEP_MAX_THRUST)
        return thruster_directions, thrust, direction_str

    def keep_control(self, prev_distance, curr_distance, curr_angle):
        """
        定点維持制御（論文Table I準拠・Push/Diagonal 8方向）

        スラスタ配置（論文Fig.2に対応）:
              D1(北)
          T3 ↗  ↖ T4
        D3←    →D7
          T1 ↘  ↙ T2
              D5(南)

        thruster_directions = [T1, T2, T3, T4]
        P=+1, N=-1, -=0

        直線方向(Push): 進行方向後ろ側の2台を正転(P)
          D1(北): T1(P), T2(P)  → [1, 1, 0, 0]
          D3(西): T1(P), T3(P)  → [1, 0, 1, 0]
          D5(南): T3(P), T4(P)  → [0, 0, 1, 1]
          D7(東): T2(P), T4(P)  → [0, 1, 0, 1]

        斜め方向(Diagonal): 隣接2台を正転(P)+逆転(N)
          D2(北西): T1(P), T4(N) → [1, 0, 0, -1]
          D4(南西): T2(N), T3(P) → [0, -1, 1, 0]
          D6(南東): T1(N), T4(P) → [-1, 0, 0, 1]
          D8(北東): T2(P), T3(N) → [0, 1, -1, 0]

        ヨー補正: 移動に使っていない2台を使用
        """
        # D1 北: -22.5 ~ +22.5 deg
        if -22.5 <= curr_angle < 22.5:
            direction_str = "D1 North [Push T1+T2]"
            thruster_directions = [1, 1, 0, 0]
            yaw_idx = (2, 3)   # 空き: T3, T4

        # D2 北西: +22.5 ~ +67.5 deg
        elif 22.5 <= curr_angle < 67.5:
            direction_str = "D2 NorthWest [Diagonal T1+T4]"
            thruster_directions = [1, 0, 0, -1]
            yaw_idx = (1, 2)   # 空き: T2, T3

        # D3 西: +67.5 ~ +112.5 deg
        elif 67.5 <= curr_angle < 112.5:
            direction_str = "D3 West [Push T1+T3]"
            thruster_directions = [1, 0, 1, 0]
            yaw_idx = (1, 3)   # 空き: T2, T4

        # D4 南西: +112.5 ~ +157.5 deg
        elif 112.5 <= curr_angle < 157.5:
            direction_str = "D4 SouthWest [Diagonal T2+T3]"
            thruster_directions = [0, -1, 1, 0]
            yaw_idx = (0, 3)   # 空き: T1, T4

        # D5 南: ±157.5 ~ ±180 deg
        elif curr_angle >= 157.5 or curr_angle < -157.5:
            direction_str = "D5 South [Push T3+T4]"
            thruster_directions = [0, 0, 1, 1]
            yaw_idx = (0, 1)   # 空き: T1, T2

        # D6 南東: -157.5 ~ -112.5 deg
        elif -157.5 <= curr_angle < -112.5:
            direction_str = "D6 SouthEast [Diagonal T1+T4]"
            thruster_directions = [-1, 0, 0, 1]
            yaw_idx = (1, 2)   # 空き: T2, T3

        # D7 東: -112.5 ~ -67.5 deg
        elif -112.5 <= curr_angle < -67.5:
            direction_str = "D7 East [Push T2+T4]"
            thruster_directions = [0, 1, 0, 1]
            yaw_idx = (0, 2)   # 空き: T1, T3

        # D8 北東: -67.5 ~ -22.5 deg
        else:
            direction_str = "D8 NorthEast [Diagonal T2+T3]"
            thruster_directions = [0, 1, -1, 0]
            yaw_idx = (0, 3)   # 空き: T1, T4

        thrust = self.PD_distance_control(prev_distance, curr_distance,
                                          max_thrust=self.KEEP_MAX_THRUST)

        # ── ヨー安定化補正 ────────────────────────────────────────
        td = list(thruster_directions)
        if self.target_heading is not None:
            curr_heading   = self.calc_heading(self.compass_value)
            yaw_error      = (curr_heading - self.target_heading + 180) % 360 - 180
            yaw_correction = self.yaw_Kp * yaw_error
            td[yaw_idx[0]] = max(min( yaw_correction, 1.0), -1.0)
            td[yaw_idx[1]] = max(min(-yaw_correction, 1.0), -1.0)
            print(f"  yaw_error={yaw_error:.2f}deg  correction={yaw_correction:.3f}  "
                  f"yaw_slots=T{yaw_idx[0]+1},T{yaw_idx[1]+1}")

        return td, thrust, direction_str


    def set_target_heading(self, compass_value):
        """現在の向きを目標ヨーとして記憶する"""
        self.target_heading = self.calc_heading(compass_value)
        print(f"Target heading set: {self.target_heading:.2f} deg")

    # ─────────────────────────────────────────────────────────────
    # 電力計算
    # ─────────────────────────────────────────────────────────────
    def calc_ampere(self, thrust, thruster_direction):
        sum_t_dir = sum(abs(d) ** 2 for d in thruster_direction)
        self.A = 1.5 * (thrust / self.STRAIGHT_MAX_THRUST) * sum_t_dir

    def update_ampere(self, thrust, thruster_direction):
        self.calc_ampere(thrust, thruster_direction)

    def calc_power(self):
        self.W = self.A * self.V

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
    def get_state(self, time_s):
        return [
            time_s,
            *self.get_gps_noise(),
            *self.get_gps_value(),
            *self.get_compass_value(),
            self.get_ampere(),
            self.get_power(),
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
            "waypoint_lat", "waypoint_lon", "waypoint_num",
            "thrust",
            "thruster_dir_1", "thruster_dir_2",
            "thruster_dir_3", "thruster_dir_4",
            "distance_m", "angle_deg", "speed_mps",
        ]
        with open(filename, mode="w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(log_data)


# ==========================================================
# メイン
# ==========================================================
robot = BIWAKO_X()
robot.load_waypoints_from_csv(parameter.way_point_file)
robot.init_device()

TIME_STEP = parameter.TIME_STEP

# 外乱設定
fluid_node = supervisor.getFromDef("STILL_WATER")
stream_vel = fluid_node.getField("streamVelocity")

def set_disturbance(x, z):
    stream_vel.setSFVec3f([x, 0.0, z])

set_disturbance(0.0, 0.0)

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
    filename = now.strftime("%Y%m%d_%H%M%S.csv")

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

    # 初期向きを目標ヨーとして記憶
    robot.set_target_heading(robot.compass_value)

    while supervisor.step(TIME_STEP) != -1:

        # 0. 状態更新
        robot.update_robot_state(thrust, thruster_direction,
                                 curr_distance, curr_angle)

        # 1. センサ値取得
        gps_value       = robot.get_gps_value()
        gps_noise_value = robot.get_gps_noise()
        compass_value   = robot.get_compass_value()

        curr_gps = gps_noise_value if parameter.gps_error_mode else gps_value

        # 2. 距離・角度計算
        prev_distance = curr_distance
        curr_distance = robot.calc_distance(curr_gps)
        prev_angle    = curr_angle
        curr_angle    = robot.calc_angle(compass_value, curr_gps,
                                         robot.get_waypoint())

        # ── デバッグ表示 ──────────────────────────────────────────
        print("=" * 55)
        print(f"[t={timestamp:6.1f}s]  WP#{robot.get_waypoint_num()}  "
              f"lap={robot.lap_count}/{robot.max_lap}")
        print(f"  GPS      : lat={curr_gps[0]:.8f}  "
              f"lon={curr_gps[1]:.8f}")
        print(f"  Target   : lat={robot.get_waypoint()[0]:.8f}  "
              f"lon={robot.get_waypoint()[1]:.8f}")
        print(f"  Distance : {curr_distance:7.3f} m  "
              f"(tolerance={parameter.main_distance_tolerance} m)")
        print(f"  Angle    : {curr_angle:7.2f} deg  "
              f"-> Direction: {direction_str}")
        print(f"  Compass  : N={compass_value[0]:.3f}  "
              f"E={compass_value[1]:.3f}  "
              f"down={compass_value[2]:.3f}")
        print(f"  Thrust   : {thrust:6.3f}  "
              f"Dir={[round(d,2) for d in thruster_direction]}")
        print(f"  Speed    : {robot.get_speed():.3f} m/s")
        # ─────────────────────────────────────────────────────────

        # 3. ログ
        log_data.append(robot.get_state(timestamp))

        # 4. 行動判断
        if curr_distance < parameter.main_distance_tolerance:
            print(f"  >> Waypoint {robot.get_waypoint_num()} reached!")
            robot.update_waypoint_num()
            robot.update_waypoint()

            if robot.get_waypoint_num() == -1:
                print("  >> Mission complete. Stopping.")
                robot.stop_thrusters()
                break
            else:
                print(f"  >> Next waypoint: {robot.get_waypoint()}")
        else:
            if parameter.mode == 0:
                thruster_direction, thrust, direction_str = \
                    robot.straight_control(prev_distance, curr_distance,
                                          prev_angle, curr_angle)
            elif parameter.mode == 2:
                thruster_direction, thrust, direction_str = \
                    robot.keep_control(prev_distance, curr_distance,
                                       curr_angle)
            robot.set_thruster_velocity(thruster_direction, thrust)

        timestamp += TIME_STEP / 1000

    robot.save_log(filename, log_data)
    print(f"ログを {filename} に保存しました")
    supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
