# SCOUT (Moorebot) API リファレンス

PC側からROS経由で利用可能なAPIと、マニュアル記載のScratch APIをまとめたドキュメント。

---

## 1. ROS Service API（PC側から呼べるもの）

`rosservice list | grep -E "Util|Nav|Core|Motor"` で確認済み。

### 1.1 UtilNode（移動・AI制御）

| Service | 型 | 実機テスト | 説明 |
|---|---|---|---|
| `/UtilNode/algo_move` | `float32 xDistance, float32 yDistance, float32 speed → int32 ret` | ✅ 動作確認済み | 前後移動。xDistance=0固定、yDistance>0で前進、<0で後退。ret>=0で成功 |
| `/UtilNode/algo_action` | `float32 xSpeed, float32 ySpeed, float32 rotatedSpeed, int32 time → int32 ret` | ✅ 動作確認済み | 時間指定の旋回。rotatedSpeed>0で左旋回、<0で右旋回。timeはミリ秒 |
| `/UtilNode/algo_roll` | `float32 angle, float32 rotatedSpeed, int32 timeout, float32 error → int32 ret` | ❌ ret=-1（無効） | 角度指定の旋回。サービスは存在するが全パターンで失敗。使用不可 |
| `/UtilNode/ai_get_detect_setting` | 未確認 | 🔧 未テスト | AI検知設定の取得（人・犬・猫検知のON/OFF状態） |
| `/UtilNode/ai_set_detect_setting` | 未確認 | 🔧 未テスト | AI検知設定の変更 |

**algo_move の使い方:**
```bash
# 前進 20cm
rosservice call /UtilNode/algo_move "xDistance: 0.0
yDistance: 0.20
speed: 0.15"

# 後退 20cm
rosservice call /UtilNode/algo_move "xDistance: 0.0
yDistance: -0.20
speed: 0.15"
```

**algo_action の使い方（旋回）:**
```bash
# 右旋回（約18度: 0.4 rad/s × 0.8s）
rosservice call /UtilNode/algo_action "xSpeed: 0.0
ySpeed: 0.0
rotatedSpeed: -0.4
time: 800"

# 左旋回
rosservice call /UtilNode/algo_action "xSpeed: 0.0
ySpeed: 0.0
rotatedSpeed: 0.4
time: 800"
```

**角度→時間の変換式:**
```
time_ms = (angle_deg × π / 180) / rotatedSpeed × 1000
例: 30° at 0.4 rad/s → (30 × 3.14159 / 180) / 0.4 × 1000 ≒ 1309 ms
```

### 1.2 NavPathNode（パトロール・ナビゲーション）

| Service | 型 | 実機テスト | 説明 |
|---|---|---|---|
| `/NavPathNode/nav_list_path` | `→ string[] path_list, int32[] size_list, string[] create_time_list, string[] name_list` | ✅ 動作確認済み | 登録済みパトロールルート一覧を取得 |
| `/NavPathNode/nav_path_start` | `int8 isFromOutStart, string name →` | 🔧 未テスト | パトロール開始。name=ルート名、isFromOutStart=0(充電台から)/1(外から) |
| `/NavPathNode/nav_get_status` | `→ int32 status` | ✅ 動作確認済み | ナビ状態取得。0=idle, 1=running, 2=paused 等 |
| `/NavPathNode/nav_patrol_stop` | `→` | 🔧 未テスト | パトロール停止（引数なし） |
| `/NavPathNode/nav_cancel` | `→` | ✅ 動作確認済み | ナビゲーション全般のキャンセル |
| `/NavPathNode/nav_exit` | 未確認 | 🔧 未テスト | ナビモード終了 |
| `/NavPathNode/nav_path_save` | 未確認 | 🔧 未テスト | パトロールパスの保存 |
| `/NavPathNode/nav_delete_path` | 未確認 | 🔧 未テスト | パトロールパスの削除 |
| `/NavPathNode/nav_patrol` | 未確認 | 🔧 未テスト | パトロール実行（nav_path_startとの違い不明） |
| `/NavPathNode/nav_get_patrol_name` | 未確認 | 🔧 未テスト | 現在のパトロール名取得 |
| `/NavPathNode/nav_waypoint_add` | 未確認 | 🔧 未テスト | ウェイポイント追加 |
| `/NavPathNode/nav_waypoint_query` | 未確認 | 🔧 未テスト | ウェイポイント照会 |
| `/NavPathNode/nav_calibration_get_status` | 未確認 | 🔧 未テスト | キャリブレーション状態 |
| `/NavPathNode/nav_mag_calibra` | 未確認 | 🔧 未テスト | 磁気キャリブレーション |
| `/NavPathNode/enable_vio` | 未確認 | 🔧 未テスト | VIO(Visual Inertial Odometry)の有効化 |

**nav_list_path の実行結果例:**
```
path_list: ['/path/table2', '/path/test', '/path/table']
size_list: [107796, 108305, 185653]
create_time_list: ['2025-11-19 01:00:12', '2025-11-19 01:32:26', '2025-11-19 00:53:46']
name_list: ['table2', 'test', 'table']
```

### 1.3 CoreNode（システム制御）

| Service | 型 | 実機テスト | 説明 |
|---|---|---|---|
| `/CoreNode/nav_cancel` | 未確認 | 🔧 未テスト | NavPathNodeのnav_cancelと重複？ |
| `/CoreNode/night_get` | 未確認 | 🔧 未テスト | ナイトビジョン状態取得 |
| `/CoreNode/adjust_light` | 未確認 | 🔧 未テスト | 照明（IR LED等）調整 |
| `/CoreNode/adjust_exposure_time` | 未確認 | 🔧 未テスト | カメラ露出時間調整 |
| `/CoreNode/motion_set_zone` | 未確認 | 🔧 未テスト | モーション検知ゾーン設定 |
| `/CoreNode/stop_detect` | 未確認 | 🔧 未テスト | 検知停止 |
| `/CoreNode/getDiffAngleWhenPatrol` | 未確認 | 🔧 未テスト | パトロール中の角度差取得 |
| `/CoreNode/saveTmpPicForStartPath` | 未確認 | 🔧 未テスト | パス開始用一時画像保存 |

---

## 2. ROS Topics（購読可能）

| Topic | 型 | 用途 |
|---|---|---|
| `/scout/camera/image_raw` | `sensor_msgs/Image` | カメラ画像（前方FHDカメラ） |
| `/MotorNode/baselink_odom_relative` | `nav_msgs/Odometry` | オドメトリ（位置・速度・姿勢） |

---

## 3. Scratch API（SCOUT内部用・参考情報）

マニュアル Appendix I 記載。SCOUTのROS上でPythonとして実行される。PC側から直接呼ぶにはROSサービス経由の対応が必要。

### 3.1 System

| Scratch モジュール | Python API | 機能 |
|---|---|---|
| start | `rollereye.start()` | プログラム開始 |
| program stop | `rollereye.stop()` | プログラム停止 |
| timer start/stop/pause | `rollereye.timerStart()` / `timerPause()` / `timerStop()` | タイマー制御 |
| timer time | `rollereye.getTimerTime()` | 経過時間取得 (ms) |
| program runtime | `rollereye.getRunTime()` | 起動からの経過時間 (ms) |
| current timestamp | `rollereye.getCurrentTime()` | UNIXタイムスタンプ (ms) |

### 3.2 Media

| Scratch モジュール | Python API | 機能 |
|---|---|---|
| set media value | `rollereye.set_soundVolume(int value)` | 音量設定 (0-100) |
| play sound effect | `rollereye.play_sound(int effectID, bool isFinished)` | 効果音再生 |
| take photo | `rollereye.capture()` | 写真撮影・保存 |
| start/stop video recording | `rollereye.record_start()` / `record_stop()` | 動画録画 |

### 3.3 Motion Control

| Scratch モジュール | Python API | 機能 |
|---|---|---|
| set translation speed | `rollereye.set_translationSpeed(float speed)` | 並進速度設定 (0-1 m/s) |
| set rotation speed | `rollereye.set_rotationSpeed(float speed)` | 回転速度設定 (0-1 m/s) |
| set wheel | `rollereye.set_wheel(int fl, int fr, int rl, int rr)` | 各ホイール個別制御 (-1000-1000 rpm) |
| translate at angle | `rollereye.set_translate(int degree)` | 指定方向に並進 (0-360°) |
| translate at angle for time | `rollereye.set_translate_2(int degree, int seconds)` | 指定方向に時間指定並進 |
| translate at angle for distance | `rollereye.set_translate_3(int degree, int meters)` | 指定方向に距離指定並進 |
| translate at angle at speed | `rollereye.set_translate_4(int degree, float speed)` | 指定方向・速度で並進 |
| rotate left/right | `rollereye.set_rotate(int direction)` | 回転 |
| rotate for time | `rollereye.set_rotate_2(int direction, int seconds)` | 時間指定回転 |
| rotate for angle | `rollereye.set_rotate_3(int direction, int degree)` | 角度指定回転 |
| translate + rotate | `rollereye.set_translate_rotate(int degree, int direction)` | 並進しながら回転 |
| stop motion | `rollereye.stop_move()` | 移動停止 |

### 3.4 AI Functions

| Scratch モジュール | Python API | 機能 |
|---|---|---|
| enable/disable identification | `rollereye.enable_reg(reg.person/cat/dog)` | 人・犬・猫の認識ON/OFF |
| when identified | `rollereye.recogResult()` | 認識結果のイベント待ち |
| check if identified | `rollereye.recResult(reg.person/cat/dog)` | 認識されたか判定 (True/False) |
| wait until identified | `rollereye.recWait(reg.person/cat/dog)` | 認識されるまで待機 |
| enable/disable motion detect | `rollereye.enable_detection()` / `disable_detection()` | モーション検知ON/OFF |
| when motion detected | `rollereye.motionDetected()` | モーション検知イベント |
| enable/disable follow | `rollereye.enable_follow()` / `disable_follow()` | 追従モードON/OFF |

---

## 4. MulmoChat での実装状況

### 実装済みツール（実機動作確認済み）

| ツール名 | 使用するAPI | 入力例 |
|---|---|---|
| `getScoutStatus` | カメラ画像 + odom + nav_get_status | 「状態を見せて」 |
| `moveCorrection` | `/UtilNode/algo_move` | 「0.2m前進して」 |
| `rotateCorrection` | `/UtilNode/algo_action` | 「右に30度回って」 |
| `stopScout` | `/NavPathNode/nav_cancel` | 「止まって」 |
| `showPatrolRoutes` | `/NavPathNode/nav_list_path` | 「ルート一覧を見せて」 |
| `askOperatorDecision` | （LLM生成） | LLMが自動的に使用 |

### 実装済み・未テスト

| ツール名 | 使用するAPI | 備考 |
|---|---|---|
| `startPatrol` | `/NavPathNode/nav_path_start` | 充電台からテスト必要 |
| `stopPatrol` | `/NavPathNode/nav_patrol_stop` | パトロール中にテスト必要 |
| `getPatrolStatus` | `/NavPathNode/nav_get_status` + カメラ | パトロール中にテスト必要 |

### 将来追加候補

| ツール名 | 使用するAPI | 機能 |
|---|---|---|
| `toggleNightVision` | `/CoreNode/night_get` + `/CoreNode/adjust_light` | ナイトビジョン制御 |
| `toggleDetection` | `/UtilNode/ai_set_detect_setting` | 人・犬・猫検知ON/OFF |
| `presentDiagnosis` | VPRデータ + カメラ画像 | VPR診断パネル（Phase 3） |

---

## 5. ハードウェア仕様（マニュアルより）

| 項目 | 仕様 |
|---|---|
| CPU | Quad-Core ARM A7 @1.2GHz |
| メモリ | 512MB LPDDR III |
| ストレージ | 4GB eMMC |
| OS | Linux + ROS |
| カメラ | 2MP CMOS (1080P), 広角120°, IRナイトビジョン |
| モーター | 4× 高速DCモーター |
| ホイール | 4× メカナムホイール |
| センサ | 6DoF IMU, 光センサ, ToF |
| 通信 | Dual-band 2.4G/5G Wi-Fi, Bluetooth 4.2 |
| バッテリー | 18650 Li-ion 2000mAh（稼働2時間以上） |
| 最高速度 | 約2km/h |
| 動作温度 | 0-40℃ |
