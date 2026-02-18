# SCOUT (Moorebot) API 繝ｪ繝輔ぃ繝ｬ繝ｳ繧ｹ

PC蛛ｴ縺九ｉROS邨檎罰縺ｧ蛻ｩ逕ｨ蜿ｯ閭ｽ縺ｪAPI縺ｨ縲√・繝九Η繧｢繝ｫ險倩ｼ峨・Scratch API繧偵∪縺ｨ繧√◆繝峨く繝･繝｡繝ｳ繝医・
---

## 1. ROS Service API・・C蛛ｴ縺九ｉ蜻ｼ縺ｹ繧九ｂ縺ｮ・・
`rosservice list | grep -E "Util|Nav|Core|Motor"` 縺ｧ遒ｺ隱肴ｸ医∩縲・
### 1.1 UtilNode・育ｧｻ蜍輔・AI蛻ｶ蠕｡・・
| Service | 蝙・| 螳滓ｩ溘ユ繧ｹ繝・| 隱ｬ譏・|
|---|---|---|---|
| `/UtilNode/algo_move` | `float32 xDistance, float32 yDistance, float32 speed 竊・int32 ret` | 笨・蜍穂ｽ懃｢ｺ隱肴ｸ医∩ | 蜑榊ｾ檎ｧｻ蜍輔ＹDistance=0蝗ｺ螳壹【Distance>0縺ｧ蜑埼ｲ縲・0縺ｧ蠕碁縲Ｓet>=0縺ｧ謌仙粥 |
| `/UtilNode/algo_action` | `float32 xSpeed, float32 ySpeed, float32 rotatedSpeed, int32 time 竊・int32 ret` | 笨・蜍穂ｽ懃｢ｺ隱肴ｸ医∩ | 譎る俣謖・ｮ壹・譌句屓縲ＳotatedSpeed>0縺ｧ蟾ｦ譌句屓縲・0縺ｧ蜿ｳ譌句屓縲Ｕime縺ｯ繝溘Μ遘・|
| `/UtilNode/algo_roll` | `float32 angle, float32 rotatedSpeed, int32 timeout, float32 error 竊・int32 ret` | 笶・ret=-1・育┌蜉ｹ・・| 隗貞ｺｦ謖・ｮ壹・譌句屓縲ゅし繝ｼ繝薙せ縺ｯ蟄伜惠縺吶ｋ縺悟・繝代ち繝ｼ繝ｳ縺ｧ螟ｱ謨励ゆｽｿ逕ｨ荳榊庄 |
| `/UtilNode/ai_get_detect_setting` | 譛ｪ遒ｺ隱・| 肌 譛ｪ繝・せ繝・| AI讀懃衍險ｭ螳壹・蜿門ｾ暦ｼ井ｺｺ繝ｻ迥ｬ繝ｻ迪ｫ讀懃衍縺ｮON/OFF迥ｶ諷具ｼ・|
| `/UtilNode/ai_set_detect_setting` | 譛ｪ遒ｺ隱・| 肌 譛ｪ繝・せ繝・| AI讀懃衍險ｭ螳壹・螟画峩 |

**algo_move 縺ｮ菴ｿ縺・婿:**
```bash
# 蜑埼ｲ 20cm
rosservice call /UtilNode/algo_move "xDistance: 0.0
yDistance: 0.20
speed: 0.15"

# 蠕碁 20cm
rosservice call /UtilNode/algo_move "xDistance: 0.0
yDistance: -0.20
speed: 0.15"
```

**algo_action 縺ｮ菴ｿ縺・婿・域雷蝗橸ｼ・**
```bash
# 蜿ｳ譌句屓・育ｴ・8蠎ｦ: 0.4 rad/s ﾃ・0.8s・・rosservice call /UtilNode/algo_action "xSpeed: 0.0
ySpeed: 0.0
rotatedSpeed: -0.4
time: 800"

# 蟾ｦ譌句屓
rosservice call /UtilNode/algo_action "xSpeed: 0.0
ySpeed: 0.0
rotatedSpeed: 0.4
time: 800"
```

**隗貞ｺｦ竊呈凾髢薙・螟画鋤蠑・**
```
time_ms = (angle_deg ﾃ・ﾏ / 180) / rotatedSpeed ﾃ・1000
萓・ 30ﾂｰ at 0.4 rad/s 竊・(30 ﾃ・3.14159 / 180) / 0.4 ﾃ・1000 竕・1309 ms
```

### 1.2 NavPathNode・医ヱ繝医Ο繝ｼ繝ｫ繝ｻ繝翫ン繧ｲ繝ｼ繧ｷ繝ｧ繝ｳ・・
| Service | 蝙・| 螳滓ｩ溘ユ繧ｹ繝・| 隱ｬ譏・|
|---|---|---|---|
| `/NavPathNode/nav_list_path` | `竊・string[] path_list, int32[] size_list, string[] create_time_list, string[] name_list` | 笨・蜍穂ｽ懃｢ｺ隱肴ｸ医∩ | 逋ｻ骭ｲ貂医∩繝代ヨ繝ｭ繝ｼ繝ｫ繝ｫ繝ｼ繝井ｸ隕ｧ繧貞叙蠕・|
| `/NavPathNode/nav_path_start` | `int8 isFromOutStart, string name 竊蛋 | 肌 譛ｪ繝・せ繝・| 繝代ヨ繝ｭ繝ｼ繝ｫ髢句ｧ九Ｏame=繝ｫ繝ｼ繝亥錐縲（sFromOutStart=0(蜈・崕蜿ｰ縺九ｉ)/1(螟悶°繧・ |
| `/NavPathNode/nav_get_status` | `竊・int32 status` | 笨・蜍穂ｽ懃｢ｺ隱肴ｸ医∩ | 繝翫ン迥ｶ諷句叙蠕励・=idle, 1=running, 2=paused 遲・|
| `/NavPathNode/nav_patrol_stop` | `竊蛋 | 肌 譛ｪ繝・せ繝・| 繝代ヨ繝ｭ繝ｼ繝ｫ蛛懈ｭ｢・亥ｼ墓焚縺ｪ縺暦ｼ・|
| `/NavPathNode/nav_cancel` | `竊蛋 | 笨・蜍穂ｽ懃｢ｺ隱肴ｸ医∩ | 繝翫ン繧ｲ繝ｼ繧ｷ繝ｧ繝ｳ蜈ｨ闊ｬ縺ｮ繧ｭ繝｣繝ｳ繧ｻ繝ｫ |
| `/NavPathNode/nav_exit` | 譛ｪ遒ｺ隱・| 肌 譛ｪ繝・せ繝・| 繝翫ン繝｢繝ｼ繝臥ｵゆｺ・|
| `/NavPathNode/nav_path_save` | 譛ｪ遒ｺ隱・| 肌 譛ｪ繝・せ繝・| 繝代ヨ繝ｭ繝ｼ繝ｫ繝代せ縺ｮ菫晏ｭ・|
| `/NavPathNode/nav_delete_path` | 譛ｪ遒ｺ隱・| 肌 譛ｪ繝・せ繝・| 繝代ヨ繝ｭ繝ｼ繝ｫ繝代せ縺ｮ蜑企勁 |
| `/NavPathNode/nav_patrol` | 譛ｪ遒ｺ隱・| 肌 譛ｪ繝・せ繝・| 繝代ヨ繝ｭ繝ｼ繝ｫ螳溯｡鯉ｼ・av_path_start縺ｨ縺ｮ驕輔＞荳肴・・・|
| `/NavPathNode/nav_get_patrol_name` | 譛ｪ遒ｺ隱・| 肌 譛ｪ繝・せ繝・| 迴ｾ蝨ｨ縺ｮ繝代ヨ繝ｭ繝ｼ繝ｫ蜷榊叙蠕・|
| `/NavPathNode/nav_waypoint_add` | 譛ｪ遒ｺ隱・| 肌 譛ｪ繝・せ繝・| 繧ｦ繧ｧ繧､繝昴う繝ｳ繝郁ｿｽ蜉 |
| `/NavPathNode/nav_waypoint_query` | 譛ｪ遒ｺ隱・| 肌 譛ｪ繝・せ繝・| 繧ｦ繧ｧ繧､繝昴う繝ｳ繝育・莨・|
| `/NavPathNode/nav_calibration_get_status` | 譛ｪ遒ｺ隱・| 肌 譛ｪ繝・せ繝・| 繧ｭ繝｣繝ｪ繝悶Ξ繝ｼ繧ｷ繝ｧ繝ｳ迥ｶ諷・|
| `/NavPathNode/nav_mag_calibra` | 譛ｪ遒ｺ隱・| 肌 譛ｪ繝・せ繝・| 逎∵ｰ励く繝｣繝ｪ繝悶Ξ繝ｼ繧ｷ繝ｧ繝ｳ |
| `/NavPathNode/enable_vio` | 譛ｪ遒ｺ隱・| 肌 譛ｪ繝・せ繝・| VIO(Visual Inertial Odometry)縺ｮ譛牙柑蛹・|

**nav_list_path 縺ｮ螳溯｡檎ｵ先棡萓・**
```
path_list: ['/path/table2', '/path/test', '/path/table']
size_list: [107796, 108305, 185653]
create_time_list: ['2025-11-19 01:00:12', '2025-11-19 01:32:26', '2025-11-19 00:53:46']
name_list: ['table2', 'test', 'table']
```

### 1.3 CoreNode・医す繧ｹ繝・Β蛻ｶ蠕｡・・
| Service | 蝙・| 螳滓ｩ溘ユ繧ｹ繝・| 隱ｬ譏・|
|---|---|---|---|
| `/CoreNode/nav_cancel` | 譛ｪ遒ｺ隱・| 肌 譛ｪ繝・せ繝・| NavPathNode縺ｮnav_cancel縺ｨ驥崎､・ｼ・|
| `/CoreNode/night_get` | 譛ｪ遒ｺ隱・| 肌 譛ｪ繝・せ繝・| 繝翫う繝医ン繧ｸ繝ｧ繝ｳ迥ｶ諷句叙蠕・|
| `/CoreNode/adjust_light` | 譛ｪ遒ｺ隱・| 肌 譛ｪ繝・せ繝・| 辣ｧ譏趣ｼ・R LED遲会ｼ芽ｪｿ謨ｴ |
| `/CoreNode/adjust_exposure_time` | 譛ｪ遒ｺ隱・| 肌 譛ｪ繝・せ繝・| 繧ｫ繝｡繝ｩ髴ｲ蜃ｺ譎る俣隱ｿ謨ｴ |
| `/CoreNode/motion_set_zone` | 譛ｪ遒ｺ隱・| 肌 譛ｪ繝・せ繝・| 繝｢繝ｼ繧ｷ繝ｧ繝ｳ讀懃衍繧ｾ繝ｼ繝ｳ險ｭ螳・|
| `/CoreNode/stop_detect` | 譛ｪ遒ｺ隱・| 肌 譛ｪ繝・せ繝・| 讀懃衍蛛懈ｭ｢ |
| `/CoreNode/getDiffAngleWhenPatrol` | 譛ｪ遒ｺ隱・| 肌 譛ｪ繝・せ繝・| 繝代ヨ繝ｭ繝ｼ繝ｫ荳ｭ縺ｮ隗貞ｺｦ蟾ｮ蜿門ｾ・|
| `/CoreNode/saveTmpPicForStartPath` | 譛ｪ遒ｺ隱・| 肌 譛ｪ繝・せ繝・| 繝代せ髢句ｧ狗畑荳譎ら判蜒丈ｿ晏ｭ・|

---

## 2. ROS Topics・郁ｳｼ隱ｭ蜿ｯ閭ｽ・・
| Topic | 蝙・| 逕ｨ騾・|
|---|---|---|
| `/scout/camera/image_raw` | `sensor_msgs/Image` | 繧ｫ繝｡繝ｩ逕ｻ蜒擾ｼ亥燕譁ｹFHD繧ｫ繝｡繝ｩ・・|
| `/MotorNode/baselink_odom_relative` | `nav_msgs/Odometry` | 繧ｪ繝峨Γ繝医Μ・井ｽ咲ｽｮ繝ｻ騾溷ｺｦ繝ｻ蟋ｿ蜍｢・・|

---

## 3. Scratch API・・COUT蜀・Κ逕ｨ繝ｻ蜿り・ュ蝣ｱ・・
繝槭ル繝･繧｢繝ｫ Appendix I 險倩ｼ峨４COUT縺ｮROS荳翫〒Python縺ｨ縺励※螳溯｡後＆繧後ｋ縲１C蛛ｴ縺九ｉ逶ｴ謗･蜻ｼ縺ｶ縺ｫ縺ｯROS繧ｵ繝ｼ繝薙せ邨檎罰縺ｮ蟇ｾ蠢懊′蠢・ｦ√・
### 3.1 System

| Scratch 繝｢繧ｸ繝･繝ｼ繝ｫ | Python API | 讖溯・ |
|---|---|---|
| start | `rollereye.start()` | 繝励Ο繧ｰ繝ｩ繝髢句ｧ・|
| program stop | `rollereye.stop()` | 繝励Ο繧ｰ繝ｩ繝蛛懈ｭ｢ |
| timer start/stop/pause | `rollereye.timerStart()` / `timerPause()` / `timerStop()` | 繧ｿ繧､繝槭・蛻ｶ蠕｡ |
| timer time | `rollereye.getTimerTime()` | 邨碁℃譎る俣蜿門ｾ・(ms) |
| program runtime | `rollereye.getRunTime()` | 襍ｷ蜍輔°繧峨・邨碁℃譎る俣 (ms) |
| current timestamp | `rollereye.getCurrentTime()` | UNIX繧ｿ繧､繝繧ｹ繧ｿ繝ｳ繝・(ms) |

### 3.2 Media

| Scratch 繝｢繧ｸ繝･繝ｼ繝ｫ | Python API | 讖溯・ |
|---|---|---|
| set media value | `rollereye.set_soundVolume(int value)` | 髻ｳ驥剰ｨｭ螳・(0-100) |
| play sound effect | `rollereye.play_sound(int effectID, bool isFinished)` | 蜉ｹ譫憺浹蜀咲函 |
| take photo | `rollereye.capture()` | 蜀咏悄謦ｮ蠖ｱ繝ｻ菫晏ｭ・|
| start/stop video recording | `rollereye.record_start()` / `record_stop()` | 蜍慕判骭ｲ逕ｻ |

### 3.3 Motion Control

| Scratch 繝｢繧ｸ繝･繝ｼ繝ｫ | Python API | 讖溯・ |
|---|---|---|
| set translation speed | `rollereye.set_translationSpeed(float speed)` | 荳ｦ騾ｲ騾溷ｺｦ險ｭ螳・(0-1 m/s) |
| set rotation speed | `rollereye.set_rotationSpeed(float speed)` | 蝗櫁ｻ｢騾溷ｺｦ險ｭ螳・(0-1 m/s) |
| set wheel | `rollereye.set_wheel(int fl, int fr, int rl, int rr)` | 蜷・・繧､繝ｼ繝ｫ蛟句挨蛻ｶ蠕｡ (-1000-1000 rpm) |
| translate at angle | `rollereye.set_translate(int degree)` | 謖・ｮ壽婿蜷代↓荳ｦ騾ｲ (0-360ﾂｰ) |
| translate at angle for time | `rollereye.set_translate_2(int degree, int seconds)` | 謖・ｮ壽婿蜷代↓譎る俣謖・ｮ壻ｸｦ騾ｲ |
| translate at angle for distance | `rollereye.set_translate_3(int degree, int meters)` | 謖・ｮ壽婿蜷代↓霍晞屬謖・ｮ壻ｸｦ騾ｲ |
| translate at angle at speed | `rollereye.set_translate_4(int degree, float speed)` | 謖・ｮ壽婿蜷代・騾溷ｺｦ縺ｧ荳ｦ騾ｲ |
| rotate left/right | `rollereye.set_rotate(int direction)` | 蝗櫁ｻ｢ |
| rotate for time | `rollereye.set_rotate_2(int direction, int seconds)` | 譎る俣謖・ｮ壼屓霆｢ |
| rotate for angle | `rollereye.set_rotate_3(int direction, int degree)` | 隗貞ｺｦ謖・ｮ壼屓霆｢ |
| translate + rotate | `rollereye.set_translate_rotate(int degree, int direction)` | 荳ｦ騾ｲ縺励↑縺後ｉ蝗櫁ｻ｢ |
| stop motion | `rollereye.stop_move()` | 遘ｻ蜍募●豁｢ |

### 3.4 AI Functions

| Scratch 繝｢繧ｸ繝･繝ｼ繝ｫ | Python API | 讖溯・ |
|---|---|---|
| enable/disable identification | `rollereye.enable_reg(reg.person/cat/dog)` | 莠ｺ繝ｻ迥ｬ繝ｻ迪ｫ縺ｮ隱崎ｭ楼N/OFF |
| when identified | `rollereye.recogResult()` | 隱崎ｭ倡ｵ先棡縺ｮ繧､繝吶Φ繝亥ｾ・■ |
| check if identified | `rollereye.recResult(reg.person/cat/dog)` | 隱崎ｭ倥＆繧後◆縺句愛螳・(True/False) |
| wait until identified | `rollereye.recWait(reg.person/cat/dog)` | 隱崎ｭ倥＆繧後ｋ縺ｾ縺ｧ蠕・ｩ・|
| enable/disable motion detect | `rollereye.enable_detection()` / `disable_detection()` | 繝｢繝ｼ繧ｷ繝ｧ繝ｳ讀懃衍ON/OFF |
| when motion detected | `rollereye.motionDetected()` | 繝｢繝ｼ繧ｷ繝ｧ繝ｳ讀懃衍繧､繝吶Φ繝・|
| enable/disable follow | `rollereye.enable_follow()` / `disable_follow()` | 霑ｽ蠕薙Δ繝ｼ繝碓N/OFF |

---

## 4. MulmoChat 縺ｧ縺ｮ螳溯｣・憾豕・
### 螳溯｣・ｸ医∩繝・・繝ｫ・亥ｮ滓ｩ溷虚菴懃｢ｺ隱肴ｸ医∩・・
| 繝・・繝ｫ蜷・| 菴ｿ逕ｨ縺吶ｋAPI | 蜈･蜉帑ｾ・|
|---|---|---|
| `getScoutStatus` | 繧ｫ繝｡繝ｩ逕ｻ蜒・+ odom + nav_get_status | 縲檎憾諷九ｒ隕九○縺ｦ縲・|
| `moveCorrection` | `/UtilNode/algo_move` | 縲・.2m蜑埼ｲ縺励※縲・|
| `rotateCorrection` | `/UtilNode/algo_action` | 縲悟承縺ｫ30蠎ｦ蝗槭▲縺ｦ縲・|
| `stopScout` | `/NavPathNode/nav_cancel` | 縲梧ｭ｢縺ｾ縺｣縺ｦ縲・|
| `showPatrolRoutes` | `/NavPathNode/nav_list_path` | 縲後Ν繝ｼ繝井ｸ隕ｧ繧定ｦ九○縺ｦ縲・|
| `askOperatorDecision` | ・・LM逕滓・・・| LLM縺瑚・蜍慕噪縺ｫ菴ｿ逕ｨ |

### 螳溯｣・ｸ医∩繝ｻ譛ｪ繝・せ繝・
| 繝・・繝ｫ蜷・| 菴ｿ逕ｨ縺吶ｋAPI | 蛯呵・|
|---|---|---|
| `startPatrol` | `/NavPathNode/nav_path_start` | 蜈・崕蜿ｰ縺九ｉ繝・せ繝亥ｿ・ｦ・|
| `stopPatrol` | `/NavPathNode/nav_patrol_stop` | 繝代ヨ繝ｭ繝ｼ繝ｫ荳ｭ縺ｫ繝・せ繝亥ｿ・ｦ・|
| `getPatrolStatus` | `/NavPathNode/nav_get_status` + 繧ｫ繝｡繝ｩ | 繝代ヨ繝ｭ繝ｼ繝ｫ荳ｭ縺ｫ繝・せ繝亥ｿ・ｦ・|

### 蟆・擂霑ｽ蜉蛟呵｣・
| 繝・・繝ｫ蜷・| 菴ｿ逕ｨ縺吶ｋAPI | 讖溯・ |
|---|---|---|
| `toggleNightVision` | `/CoreNode/night_get` + `/CoreNode/adjust_light` | 繝翫う繝医ン繧ｸ繝ｧ繝ｳ蛻ｶ蠕｡ |
| `toggleDetection` | `/UtilNode/ai_set_detect_setting` | 莠ｺ繝ｻ迥ｬ繝ｻ迪ｫ讀懃衍ON/OFF |
| `presentDiagnosis` | VPR繝・・繧ｿ + 繧ｫ繝｡繝ｩ逕ｻ蜒・| VPR險ｺ譁ｭ繝代ロ繝ｫ・・hase 3・・|

---

## 5. 繝上・繝峨え繧ｧ繧｢莉墓ｧ假ｼ医・繝九Η繧｢繝ｫ繧医ｊ・・
| 鬆・岼 | 莉墓ｧ・|
|---|---|
| CPU | Quad-Core ARM A7 @1.2GHz |
| 繝｡繝｢繝ｪ | 512MB LPDDR III |
| 繧ｹ繝医Ξ繝ｼ繧ｸ | 4GB eMMC |
| OS | Linux + ROS |
| 繧ｫ繝｡繝ｩ | 2MP CMOS (1080P), 蠎・ｧ・20ﾂｰ, IR繝翫う繝医ン繧ｸ繝ｧ繝ｳ |
| 繝｢繝ｼ繧ｿ繝ｼ | 4ﾃ・鬮倬櫂C繝｢繝ｼ繧ｿ繝ｼ |
| 繝帙う繝ｼ繝ｫ | 4ﾃ・繝｡繧ｫ繝翫Β繝帙う繝ｼ繝ｫ |
| 繧ｻ繝ｳ繧ｵ | 6DoF IMU, 蜈峨そ繝ｳ繧ｵ, ToF |
| 騾壻ｿ｡ | Dual-band 2.4G/5G Wi-Fi, Bluetooth 4.2 |
| 繝舌ャ繝・Μ繝ｼ | 18650 Li-ion 2000mAh・育ｨｼ蜒・譎る俣莉･荳奇ｼ・|
| 譛鬮倬溷ｺｦ | 邏・km/h |
| 蜍穂ｽ懈ｸｩ蠎ｦ | 0-40邃・|
