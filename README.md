# BIWAKO-SP_simulation

Webots 上で、4 基のスラスタを持つ小型水上ロボット **BIWAKO-SP** の**定点維持（Station Keeping）**をシミュレートするプロジェクトです。
ソーラーパネルによる発電とスラスタ・搭載機器による消費をモデル化してバッテリ SOC を時々刻々計算し、走行・電力データを CSV に記録します。

- SOC に応じて定点維持の許容誤差を切り替える **SOC 連動型の定点維持戦略**
- 日射量（日周変化＋雲の変動）を考慮した **太陽光発電モデル**
- T200 スラスタの実測データに基づく **消費電力モデル**
- ランダムに変化する **水流外乱** と **GPS ノイズ**

---

## ディレクトリ構成

```
BIWAKO-SP/
├── worlds/
│   └── BIWAKO-SP.wbt          # シミュレーション用ワールド（水槽・水流・ロボット・目標マーカ）
├── protos/
│   └── BIWAKO8.proto          # ロボット本体モデル（4スラスタ・ソーラーパネル・GPS・コンパス）
├── controllers/BIWAKO-SP/
│   ├── BIWAKO-SP.py           # メインコントローラ（定点維持＋電力収支＋ログ出力）
│   ├── BIWAKO-8.py            # 旧コントローラ（直進／定点維持モード、電力収支なし）
│   ├── const.py               # 全パラメータ定義
│   └── waypoint/test.csv      # 目標点（緯度, 経度）
└── results/csv/               # 実行ログ出力先（.gitignore 対象）
    └── 結果/
        ├── analyze_all.py             # 誤差の時系列・ヒストグラム作成
        └── plot_results_separate.py   # 軌跡・消費電力量・発電量・SOC グラフ作成
```

## 動作環境

| 項目 | 内容 |
|---|---|
| シミュレータ | Webots **R2021b**（ワールド・PROTO は `R2021b`、座標系 `NUE`） |
| Python | 3.8 系（Webots のコントローラ用 Python） |
| 必要パッケージ | `geopy`（コントローラ）、`pandas` / `numpy` / `matplotlib`（解析スクリプト） |

```bash
pip install geopy pandas numpy matplotlib
```

## 実行方法

1. Webots で `worlds/BIWAKO-SP.wbt` を開く。
2. シミュレーションを開始する（長時間になるので「Fast」モード推奨）。
3. 次のいずれかで自動的に停止し、ログを保存してシミュレーションを一時停止する。
   - シミュレーション時間が `max_simulation_time`（既定 24 時間）に到達
   - SOC が 0 になる
4. ログは `results/csv/YYYYMMDD_HHMMSS.csv` に保存される（コントローラディレクトリからの相対パス `../../results/csv/`）。

> `results/csv/` ディレクトリが存在しないと保存に失敗するので、事前に作成しておくこと。
> ログはシミュレーション終了時にまとめて書き出すため、途中で Webots を閉じると保存されない。

---

## シミュレーションモデル

### ワールド（`worlds/BIWAKO-SP.wbt`）

- 半径 500 m の円形水槽（`Fluid` ノード `STILL_WATER`）。
- GPS 基準点は `35.0491 N, 135.924 E`（琵琶湖付近）。
- 水流 `streamVelocity` をコントローラ（Supervisor）から書き換えて外乱を与える。
- `WP1`〜`WP3` は半径 1.5 / 2.5 / 5 m の円盤で、許容誤差範囲の目安となる表示用マーカ。
- 物理ステップ `basicTimeStep` は 10 ms、制御周期は `const.py` の `TIME_STEP`（1000 ms）。

### ロボット（`protos/BIWAKO8.proto`）

- 中央チャンバ＋十字に伸びた 4 本のアーム。各アーム先端にスラスタ 1 基とフロート 2 個。
  - `thruster1`: 南（-Z）、`thruster2`: 西（-X）、`thruster3`: 北（+Z）、`thruster4`: 東（+X）
- 上部に 500 × 500 mm（中央に 150 × 150 mm の穴）のソーラーパネル。
- センサ
  - `gps`: ノイズあり（`accuracy 1.5` m、`noiseCorrelation 0.9`）— 実機相当
  - `a_gps`: ノイズなし — 真値の記録用
  - `compass`
- `supervisor TRUE` のため、コントローラから水流（外乱）を操作できる。

---

## コントローラの処理（`controllers/BIWAKO-SP/BIWAKO-SP.py`）

### 全体の流れ

```
初期化（ウェイポイント読込・センサ有効化・5ステップ待機）
  └─ waypoint/test.csv の 1 点目を定点維持目標（station_target）に設定
ループ（1 s ごと）
  0. 状態更新：センサ値、電流・電力、雲係数、発電量・SOC、外乱（15分ごと）
  1. センサ値取得（gps_error_mode=True ならノイズあり GPS を制御に使用）
  2. 制御目標の決定：一時目標点があればそれを、なければ定点目標を使用
     └─ 定点目標のときは SOC 連動でモード（許容誤差）を更新
  3. 目標までの距離・相対方位を計算
  4. ログを記録
  5. 行動判断
     ├─ 距離 < 許容誤差 → スラスタ停止（一時目標に到達したら定点目標へ戻る）
     └─ 距離 ≥ 許容誤差 → 一時目標点を設定し、8方向 ON-OFF 制御で移動
  6. 終了判断（SOC = 0 または最大時間到達）
終了時に CSV を保存
```

### 定点維持戦略

1. 定点目標からの距離が許容誤差を超えたら「逸脱」と判定する。
2. 逸脱した瞬間に、**現在位置 → 定点目標の方向へ `overshoot_r`（1.5 m）だけ行き過ぎた点**を一時目標点として設定する（`calc_temp_target`）。外乱に流される分を見込んで、目標の少し風上側まで戻す狙い。
3. 一時目標点の `temp_distance_tolerance`（1.5 m）以内に入ったら停止し、再び定点目標を基準に監視する。
4. 移動中は目標への相対方位を 45° ごとの 8 方向に分け、対応する 2 基のスラスタを `KEEP_MAX_THRUST` で駆動する（`keep_control`、ON-OFF 制御）。

### SOC 連動モード（`adaptive_mode = True`）

| モード | 条件 | 許容誤差 |
|---|---|---|
| high（高精度） | SOC ≥ 0.8 | 3 m |
| standard（標準） | 0.3 ≤ SOC < 0.8 | 5 m |
| save（省エネ） | SOC < 0.3 | 10 m |

チャタリング防止のため、切替候補が `hysteresis_time`（60 s）続いたときにだけモードを切り替える。
`adaptive_mode = False` にすると standard（5 m）固定となり、比較実験に使える。

### 電力モデル

**消費電力**

スラスタ 1 基の強度 `p` から電流を 2 次式で求める（T200, 12 V の公開データから同定）。

```
p    = |方向係数| × thrust / (KEEP_MAX_THRUST / MAX_THRUST)    # 最大 0.5
I(p) = 22.43 p² − 5.53 p + 0.46   [A]   （p < 0.1 は I = 0）
P    = V × ΣI + P_other            [W]   （V = 12 V, P_other = 3 W）
```

**太陽光発電**

```
G(t)      = G_max × sin(π (h − t_rise) / (t_set − t_rise)) × kcloud(t)   （日中のみ）
P_solar   = η_pv × A_pv × G(t) × η_mppt
kcloud(t) = k_min + (k_max − k_min) × (1 − sin(2π t / T)) / 2             （cloud_mode=True 時）
```

既定値は `G_max = 950 W/m²`、日の出 5 時・日の入り 19 時、`η_pv = 0.166`、`A_pv = 0.3016 m²`、`η_mppt = 0.95`（最大約 45 W）。雲は周期 2 時間で `kcloud` が 0.2〜1.0 を変動する。

**バッテリ**

容量 240 Wh（12 V × 20 Ah）、初期 SOC 100 %。毎ステップ次式で更新し、0〜1 にクリップする。

```
SOC += (P_solar − P) × Δt[h] / 240
```

### 外乱

`disturbance_mode = True` のとき、`disturbance_interval`（900 s = 15 分）ごとに強度 0〜`disturbance_strength`（0.2 m/s）、方向 0〜360° を一様乱数で決め、水流速度として与える。
現状、乱数シード設定（`random.seed`）はコメントアウトされているため、実行ごとに外乱が変わる。

---

## パラメータ（`controllers/BIWAKO-SP/const.py`）

実験条件はすべてこのファイルで変更する。主なもの：

| カテゴリ | パラメータ | 既定値 | 説明 |
|---|---|---|---|
| 全体 | `max_simulation_time` | 86400 s | 最大シミュレーション時間（24 h） |
| | `TIME_STEP` | 1000 ms | 制御周期 |
| | `sim_start_hour` | 0.0 | 開始時刻 [h]（発電量計算用） |
| | `way_point_file` | `./waypoint/test.csv` | 定点目標ファイル |
| センサ | `gps_error_mode` | True | 制御にノイズあり GPS を使うか |
| 外乱 | `disturbance_mode` | True | 外乱の有無 |
| | `disturbance_strength` | 0.2 m/s | 外乱強度の上限 |
| | `disturbance_interval` | 900 s | 外乱の更新間隔 |
| 戦略 | `adaptive_mode` | True | SOC 連動モード切替の有無 |
| | `SOC_high` / `SOC_low` | 0.8 / 0.3 | モード切替閾値 |
| | `d_high` / `d_std` / `d_save` | 3 / 5 / 10 m | 各モードの許容誤差 |
| | `hysteresis_time` | 60 s | モード切替の継続確認時間 |
| | `temp_distance_tolerance` | 1.5 m | 一時目標点の許容誤差 |
| | `overshoot_r` | 1.5 m | 一時目標点のオーバーシュート距離 |
| 推力 | `KEEP_MAX_THRUST` | 12 × √2/2 | 定点維持時のスラスタ速度指令 |
| | `MAX_THRUST` | 0.5 | 電流計算時の最大強度 |
| 消費 | `current_a/b/c` | 22.43 / −5.53 / 0.46 | 電流近似式の係数 |
| | `P_other` | 3.0 W | 搭載機器の消費電力 |
| 発電 | `G_max` | 950 W/m² | 最大日射量 |
| | `t_rise` / `t_set` | 5 / 19 h | 日の出・日の入り |
| | `cloud_mode` | True | 雲係数の有無 |
| | `cloud_period` | 7200 s | 雲の変動周期 |
| | `cloud_k_min` / `cloud_k_max` | 0.2 / 1.0 | 雲係数の範囲 |

`const.py` には冬季想定（`G_max = 550`、日の出 7 時・日の入り 17 時）の設定がコメントで残してある。

> `mode`、`max_lap`、`main_distance_tolerance`、`distance_Kp/Kd`、`bearing_Kp/Kd`、`STRAGHT_MAX_THRUST` は旧コントローラ（`BIWAKO-8.py`）用、または直進制御用のパラメータで、現在の `BIWAKO-SP.py` の定点維持ループでは使われていない。

### ウェイポイント（`waypoint/test.csv`）

1 行に `緯度, 経度` を書く。`#` で始まる行は数値変換に失敗してスキップされるため、コメントとして使える。
`BIWAKO-SP.py` では **1 行目の有効な点のみ** を定点維持目標として使う。

```csv
# 35.04907898749316, 135.92446350770626
35.0491, 135.924
```

---

## 出力データ

`results/csv/YYYYMMDD_HHMMSS.csv`（1 行 = 1 制御ステップ = 1 s）

| 列 | 内容 |
|---|---|
| `time_s` | 経過時間 [s] |
| `gps_noise_lat`, `gps_noise_lon` | ノイズあり GPS の緯度・経度 |
| `gps_lat`, `gps_lon` | ノイズなし GPS（真値）の緯度・経度 |
| `compass_north`, `compass_east`, `compass_down` | コンパス値 |
| `ampere_A` | スラスタ合計電流 [A] |
| `power_W` | 消費電力 [W]（スラスタ＋搭載機器） |
| `solar_W` | 発電電力 [W] |
| `kcloud` | 雲係数 |
| `SOC` | バッテリ残量（0〜1） |
| `disturbance_strength`, `disturbance_angle` | 外乱強度 [m/s]・方向 [deg] |
| `waypoint_lat`, `waypoint_lon`, `waypoint_num` | ウェイポイント情報 |
| `thrust` | スラスタ速度指令 |
| `thruster_dir_1`〜`thruster_dir_4` | 各スラスタの方向係数（−1, 0, 1） |
| `distance_m` | 制御中の目標（定点または一時目標）までの距離 [m] |
| `angle_deg` | 目標への相対方位 [deg] |
| `speed_mps` | 速度 [m/s] |

> `distance_m` は一時目標点への距離になっている区間があるため、定点目標からの誤差を評価するときは `gps_lat`/`gps_lon` と定点目標から計算し直すのが確実。

## 結果の解析（`results/csv/結果/`）

```bash
# 誤差距離の時系列グラフとヒストグラム
python analyze_all.py [フォルダ or CSV] [出力先]

# 軌跡・累積消費電力量・発電量・SOC のグラフ
python plot_results_separate.py [フォルダ or CSV]
```

いずれもスクリプト冒頭の定数（グラフ軸の上限、許容誤差半径、SOC 閾値など）で表示を調整できる。
`結果/` 以下の `1-1.csv`、`4-2.csv` などは条件別の実験結果、`prev/`・`グラフプロット/` は過去の出力。

> `results/` は `.gitignore` で除外されているため、ログと解析スクリプトはリポジトリに含まれない。

---

## 旧コントローラ（`BIWAKO-8.py`）

電力収支を持たない以前のバージョン。`const.py` の `mode` で直進モード（0：複数ウェイポイントを PD 制御で巡回）と定点維持モード（2）を切り替える。使う場合はワールドの `controller` フィールドを変更し、ファイル名とコントローラ名の対応に注意すること。
