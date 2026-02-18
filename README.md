# SCOUT ﾃ・MulmoChat

**蟇ｾ隧ｱ蝙九Ο繝懊ャ繝磯°逕ｨ繧､繝ｳ繧ｿ繝輔ぉ繝ｼ繧ｹ 窶・GUI Chat Protocol for Robot Operation**

LLM縺ｮ蟇ｾ隧ｱ縺ｮ荳ｭ縺ｫ繧ｫ繝｡繝ｩ逕ｻ蜒上・迥ｶ諷九ヱ繝阪Ν繝ｻ驕ｸ謚櫁い繝輔か繝ｼ繝縺後う繝ｳ繝ｩ繧､繝ｳ縺ｧ蜃ｺ迴ｾ縺吶ｋ縲∵眠縺励＞繝ｭ繝懊ャ繝磯°逕ｨUI縺ｮ螳溯｣・〒縺吶・
<p align="center">
  <img src="docs/images/screenshot_status.png" width="400" alt="Status Panel" />
  <img src="docs/images/screenshot_action.png" width="400" alt="Action Result" />
</p>

## 讎りｦ・
蟾｡蝗槭Ο繝懊ャ繝茨ｼ・Moorebot Scout](https://www.moorebot.com/)・峨ｒ縲∬・辟ｶ險隱槭〒蟇ｾ隧ｱ縺励↑縺後ｉ謫堺ｽ懊・逶｣隕悶☆繧九す繧ｹ繝・Β縺ｧ縺吶・
**蠕捺擂縺ｮ繝ｭ繝懊ャ繝磯°逕ｨUI縺ｮ隱ｲ鬘・**
- 繝繝・す繝･繝懊・繝牙梛: 逡ｰ蟶ｸ譎ゅ↓繝ｭ繧ｰ繧定ｪｭ縺ｿ縲∝挨逕ｻ髱｢縺ｧ謫堺ｽ・竊・迥ｶ豕∵滑謠｡縺ｫ譎る俣縺後°縺九ｋ
- LLM繝・く繧ｹ繝医・縺ｿ: 蟇ｾ隧ｱ縺ｯ縺ｧ縺阪ｋ縺袈I陦ｨ遉ｺ縺後↑縺・竊・逕ｻ蜒乗ｯ碑ｼ・ｄ讒矩蛹悶＆繧後◆驕ｸ謚槭′縺ｧ縺阪↑縺・
**譛ｬ繧ｷ繧ｹ繝・Β縺ｮ繧｢繝励Ο繝ｼ繝・ GUI Chat Protocol**

[MulmoChat](https://github.com/receptron/MulmoChat) 縺ｮ GUI Chat Protocol 縺ｮ諤晄Φ繧貞ｿ懃畑縺励´LM縺ｮ function call 繧呈僑蠑ｵ縺励∪縺吶ょ推繝・・繝ｫ縺ｮ螳溯｡檎ｵ先棡縺・**縲鍬LM縺ｸ縺ｮ繝・く繧ｹ繝医搾ｼ九袈I縺ｸ縺ｮ繝ｪ繝・メ繝・・繧ｿ縲・* 縺ｮ2遞ｮ鬘槭ｒ蜷梧凾縺ｫ霑斐☆縺薙→縺ｧ縲√メ繝｣繝・ヨ縺ｮ豬√ｌ繧貞｣翫＆縺壹↓繧ｫ繝｡繝ｩ逕ｻ蜒上・迥ｶ諷九ヱ繝阪Ν繝ｻ謫堺ｽ懊・繧ｿ繝ｳ縺後う繝ｳ繝ｩ繧､繝ｳ縺ｧ蜃ｺ迴ｾ縺励∪縺吶・
```
繝ｦ繝ｼ繧ｶ繝ｼ: 縲檎憾諷九ｒ隕九○縺ｦ縲・    竊・LLM 竊・function call: getScoutStatus()
    竊・繝・・繝ｫ螳溯｡・竊・2遞ｮ鬘槭・繝・・繧ｿ繧定ｿ斐☆:
  笏懌楳笏 llm_text: "status=idle, position=x:1.2 y:4.5"  竊・LLM縺ｮ谺｡縺ｮ蛻､譁ｭ譚先侭
  笏披楳笏 gui_data: {type:"status_panel", image:..., odom:...}  竊・UI縺ｫ繝代ロ繝ｫ陦ｨ遉ｺ
    竊・繝√Ε繝・ヨ蜀・↓繧ｫ繝｡繝ｩ逕ｻ蜒擾ｼ狗憾諷九ユ繝ｼ繝悶Ν縺悟・迴ｾ + LLM縺瑚・辟ｶ險隱槭〒隗｣隱ｬ
```

## 迚ｹ蠕ｴ

- **蟇ｾ隧ｱ縺ｨUI縺ｮ邨ｱ蜷・* 窶・繧ｫ繝｡繝ｩ逕ｻ蜒上・迥ｶ諷九ヱ繝阪Ν繝ｻ驕ｸ謚櫁い縺後メ繝｣繝・ヨ蜀・↓繧､繝ｳ繝ｩ繧､繝ｳ陦ｨ遉ｺ縺輔ｌ繧・- **LLM縺ｯ蛻､譁ｭ縺ｮ縺ｿ** 窶・蛻ｶ蠕｡縺ｯ縺吶∋縺ｦROS Service邨檎罰縲・LM縺梧垓襍ｰ縺励※繧ゅΟ繝懊ャ繝医・螳牙・
- **LLM髱樔ｾ晏ｭ・* 窶・Claude / GPT / Ollama 遲峨↓蟾ｮ縺玲崛縺亥庄閭ｽ・・llm_client.py` 縺ｮ縺ｿ螟画峩・・- **霆ｽ驥・* 窶・Python + Flask縲１ython 3.8+縺ｧ蜍穂ｽ・- **讒矩蛹悶Ο繧ｰ** 窶・蜈ｨ謫堺ｽ懊′JSON Lines蠖｢蠑上〒險倬鹸縺輔ｌ縲∫皮ｩｶ隧穂ｾ｡縺ｫ逶ｴ邨・- **諡｡蠑ｵ螳ｹ譏・* 窶・繝・・繝ｫ螳夂ｾｩ繧定ｿｽ蜉縺吶ｋ縺縺代〒譁ｰ讖溯・縺御ｽｿ縺医ｋ

## 繧｢繝ｼ繧ｭ繝・け繝√Ε

```
笏娯楳笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏・笏・ Operator (Browser)                          笏・笏・ 繝√Ε繝・ヨ蜈･蜉・竊・GUI繝代ロ繝ｫ陦ｨ遉ｺ                   笏・笏披楳笏笏笏笏笏笏笏笏笏笏笏笏笏笏ｬ笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏・               笏・HTTP
笏娯楳笏笏笏笏笏笏笏笏笏笏笏笏笏笆ｼ笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏・笏・ app_flask.py  (Flask Server)                笏・笏・ HTML/JS 縺・gui_data 繧偵う繝ｳ繝ｩ繧､繝ｳ繝ｬ繝ｳ繝繝ｪ繝ｳ繧ｰ     笏・笏披楳笏笏笏笏笏笏笏笏笏笏笏笏笏笏ｬ笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏・               笏・笏娯楳笏笏笏笏笏笏笏笏笏笏笏笏笏笆ｼ笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏・笏・ llm_client.py  (Claude API / tool_use loop) 笏・笏・ 繝ｦ繝ｼ繧ｶ繝ｼ逋ｺ隧ｱ 竊・LLM 竊・function call 竊・郢ｰ繧願ｿ斐＠  笏・笏披楳笏笏笏笏笏笏笏笏笏笏笏笏笏笏ｬ笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏・               笏・function call
笏娯楳笏笏笏笏笏笏笏笏笏笏笏笏笏笆ｼ笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏・笏・ tool_executor.py  笘・GUI Chat Protocol 譬ｸ蠢・  笏・笏・ 繝・・繝ｫ螳溯｡・竊・{ llm_text + gui_data } 繧定ｿ斐☆    笏・笏披楳笏笏笏笏笏笏笏笏笏笏笏笏笏笏ｬ笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏・               笏・笏娯楳笏笏笏笏笏笏笏笏笏笏笏笏笏笆ｼ笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏・笏・ ros_client.py  (rospy)                      笏・笏・ /UtilNode/algo_move, algo_action            笏・笏・ /NavPathNode/nav_path_start, nav_list_path  笏・笏披楳笏笏笏笏笏笏笏笏笏笏笏笏笏笏ｬ笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏・               笏・ROS Service Call
笏娯楳笏笏笏笏笏笏笏笏笏笏笏笏笏笆ｼ笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏・笏・ SCOUT (Moorebot Scout / roller_eye ROS)     笏・笏・ Camera, Odom, Motors, Sensors               笏・笏披楳笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏笏・```

## 繝・Δ

| 蜈･蜉・| 蜍穂ｽ・|
|---|---|
| 縲檎憾諷九ｒ隕九○縺ｦ縲・| 藤 繧ｫ繝｡繝ｩ逕ｻ蜒擾ｼ倶ｽ咲ｽｮ・矩溷ｺｦ・起av迥ｶ諷九・繝代ロ繝ｫ陦ｨ遉ｺ |
| 縲・.2m蜑埼ｲ縺励※縲・| 笨・Before/After逕ｻ蜒乗ｯ碑ｼ・ｻ倥″縺ｮ遘ｻ蜍慕ｵ先棡陦ｨ遉ｺ |
| 縲悟承縺ｫ30蠎ｦ蝗槭▲縺ｦ縲・| 笨・algo_action螳溯｡鯉ｼ狗ｵ先棡繝代ロ繝ｫ |
| 縲後Ν繝ｼ繝井ｸ隕ｧ繧定ｦ九○縺ｦ縲・| 搭 逋ｻ骭ｲ貂医∩繝代ヨ繝ｭ繝ｼ繝ｫ繝ｫ繝ｼ繝医・繧ｯ繝ｪ繝・け蜿ｯ閭ｽ繝ｪ繧ｹ繝・|
| 縲悟ｷ｡蝗槭＠縺ｦ縲・| 垳 繝代ヨ繝ｭ繝ｼ繝ｫ髢句ｧ具ｼ矩ｲ謐励ヱ繝阪Ν |
| 逡ｰ蟶ｸ逋ｺ逕滓凾 | ､・LLM縺瑚・蜍慕噪縺ｫ驕ｸ謚櫁い繝輔か繝ｼ繝繧堤函謌舌・謠千､ｺ |

## 繧ｻ繝・ヨ繧｢繝・・

### 蠢・ｦ√↑繧ゅ・

- Moorebot Scout・・OS謗･邯壽ｸ医∩・・- PC・・buntu, ROS Noetic, Python 3.8+・・- Anthropic API 繧ｭ繝ｼ・・蜿門ｾ玲婿豕評(https://console.anthropic.com/)・・
### 繧､繝ｳ繧ｹ繝医・繝ｫ

```bash
git clone https://github.com/YOUR_USERNAME/scout-mulmochat.git
cd scout-mulmochat

pip3 install anthropic flask
```

### 繝｢繝・け繝｢繝ｼ繝峨〒蜍穂ｽ懃｢ｺ隱搾ｼ・OS荳崎ｦ・ｼ・
```bash
export ANTHROPIC_API_KEY=<YOUR_ANTHROPIC_API_KEY>
python3 app_flask.py --mock
# 繝悶Λ繧ｦ繧ｶ縺ｧ http://localhost:7860
```

### 螳滓ｩ滓磁邯・
```bash
# ROS迺ｰ蠅・′險ｭ螳壹＆繧後◆繧ｿ繝ｼ繝溘リ繝ｫ縺ｧ
export ROS_MASTER_URI=http://<SCOUT_IP>:11311
export ANTHROPIC_API_KEY=<YOUR_ANTHROPIC_API_KEY>
python3 app_flask.py
```

## 繝輔ぃ繧､繝ｫ讒区・

```
scout-mulmochat/
笏懌楳笏 app_flask.py          # Flask 繝√Ε繝・ヨUI + GUI繝ｬ繝ｳ繝繝ｪ繝ｳ繧ｰ・医Γ繧､繝ｳ繧ｨ繝ｳ繝医Μ・・笏懌楳笏 llm_client.py         # LLM騾壻ｿ｡ + tool_use 繝ｫ繝ｼ繝・笏懌楳笏 tool_definitions.py   # 繝・・繝ｫ螳夂ｾｩ + System Prompt
笏懌楳笏 tool_executor.py      # 繝・・繝ｫ螳溯｡鯉ｼ・UI Chat Protocol 譬ｸ蠢・ｼ・笏懌楳笏 ros_client.py         # SCOUT ROS 騾壻ｿ｡繧ｯ繝ｩ繧､繧｢繝ｳ繝・笏懌楳笏 interaction_log.py    # 讒矩蛹悶う繝ｳ繧ｿ繝ｩ繧ｯ繧ｷ繝ｧ繝ｳ繝ｭ繧ｰ
笏懌楳笏 docs/
笏・  笏懌楳笏 images/           # 繧ｹ繧ｯ繝ｪ繝ｼ繝ｳ繧ｷ繝ｧ繝・ヨ
笏・  笏披楳笏 api-reference.md  # SCOUT API 繝ｪ繝輔ぃ繝ｬ繝ｳ繧ｹ
笏懌楳笏 LICENSE
笏披楳笏 README.md
```

## GUI Chat Protocol 縺ｮ險ｭ險医ヱ繧ｿ繝ｼ繝ｳ

譛ｬ繝励Ο繧ｸ繧ｧ繧ｯ繝医・譬ｸ蠢・・ `tool_executor.py` 縺ｫ縺ゅｊ縺ｾ縺吶ょ推繝・・繝ｫ縺瑚ｿ斐☆繝・・繧ｿ讒矩・・
```python
def execute_tool(tool_name, tool_input):
    # ... 繝・・繝ｫ螳溯｡・...
    return {
        "llm_text": "status=idle, battery=85%",    # LLM縺ｮ諤晁・攝譁・        "gui_data": {                               # UI縺ｫ繝代ロ繝ｫ陦ｨ遉ｺ
            "type": "status_panel",                 # 繝代ロ繝ｫ遞ｮ蛻･
            "camera_image_b64": "...",              # base64逕ｻ蜒・            "odom": {"x": 1.2, "y": 4.5, ...},    # 讒矩蛹悶ョ繝ｼ繧ｿ
        }
    }
```

`gui_data.type` 縺ｫ繧医▲縺ｦ繝輔Ο繝ｳ繝医お繝ｳ繝峨・繝ｬ繝ｳ繝繝ｩ縺梧ｱｺ縺ｾ繧翫∪縺呻ｼ・
| type | 陦ｨ遉ｺ蜀・ｮｹ |
|---|---|
| `status_panel` | 繧ｫ繝｡繝ｩ逕ｻ蜒擾ｼ狗憾諷九ユ繝ｼ繝悶Ν |
| `action_result` | Before/After逕ｻ蜒乗ｯ碑ｼ・ｼ区・蜷ｦ |
| `decision_form` | 繧ｯ繝ｪ繝・け蜿ｯ閭ｽ縺ｪ驕ｸ謚櫁い繝輔か繝ｼ繝 |
| `route_list` | 繝代ヨ繝ｭ繝ｼ繝ｫ繝ｫ繝ｼ繝井ｸ隕ｧ |
| `patrol_started` | 繝代ヨ繝ｭ繝ｼ繝ｫ髢句ｧ矩夂衍 |
| `patrol_status` | 繝代ヨ繝ｭ繝ｼ繝ｫ騾ｲ謐暦ｼ九き繝｡繝ｩ逕ｻ蜒・|
| `error` | 繧ｨ繝ｩ繝ｼ陦ｨ遉ｺ |

**譁ｰ縺励＞繝・・繝ｫ縺ｮ霑ｽ蜉譁ｹ豕包ｼ・繧ｹ繝・ャ繝暦ｼ・**
1. `tool_definitions.py` 縺ｫ繝・・繝ｫ螳夂ｾｩ繧定ｿｽ蜉
2. `tool_executor.py` 縺ｫ `_exec_繝・・繝ｫ蜷港 繝｡繧ｽ繝・ラ繧定ｿｽ蜉
3. `app_flask.py` 縺ｮ JavaScript 縺ｫ `render` 髢｢謨ｰ繧定ｿｽ蜉

LLM蛛ｴ縺ｮ螟画峩縺ｯ荳崎ｦ√〒縺吶・
## 莉悶・繝ｭ繝懊ャ繝医∈縺ｮ驕ｩ逕ｨ

縺薙・繧｢繝ｼ繧ｭ繝・け繝√Ε縺ｯ SCOUT 蝗ｺ譛峨〒縺ｯ縺ゅｊ縺ｾ縺帙ｓ縲Ａros_client.py` 繧貞ｷｮ縺玲崛縺医ｌ縺ｰ莉悶・ROS繝ｭ繝懊ャ繝医↓繧る←逕ｨ縺ｧ縺阪∪縺呻ｼ・
```python
# 萓・ TurtleBot3 逕ｨ
class TurtleBot3ROSClient:
    def algo_move(self, y_distance, speed=0.15):
        # cmd_vel 縺ｫ螟画鋤縺励※ publish
        ...
    def get_camera_image_b64(self):
        # /camera/rgb/image_raw 繧貞叙蠕・        ...
```

ROS莉･螟悶・繝ｭ繝懊ャ繝茨ｼ・TTP API縲√す繝ｪ繧｢繝ｫ騾壻ｿ｡遲会ｼ峨〒繧ゅ∝酔縺倥う繝ｳ繧ｿ繝輔ぉ繝ｼ繧ｹ繧貞ｮ溯｣・☆繧後・蛻ｩ逕ｨ蜿ｯ閭ｽ縺ｧ縺吶・
## 遐皮ｩｶ逧・レ譎ｯ

譛ｬ繧ｷ繧ｹ繝・Β縺ｯ莉･荳九・遐皮ｩｶ隱ｲ鬘後↓蜿悶ｊ邨・ｓ縺ｧ縺・∪縺呻ｼ・
- **Mixed-Initiative Robot Operation**: 逡ｰ蟶ｸ譎ゅ↓繝ｭ繝懊ャ繝医・LLM繝ｻ繧ｪ繝壹Ξ繝ｼ繧ｿ縺悟鵠隱ｿ縺励※蠕ｩ蟶ｰ
- **Explainable Robot Operation**: 繝・く繧ｹ繝育函謌舌〒縺ｯ縺ｪ縺酋I謠千､ｺ縺ｧ隱ｬ譏主庄閭ｽ諤ｧ繧呈球菫・- **Structured Interaction Logging**: 莠ｺ縺ｮ莉句・繧谷SON蠖｢蠑上〒閾ｪ蜍戊ｨ倬鹸縺苓ｩ穂ｾ｡蜿ｯ閭ｽ縺ｫ

GUI Chat Protocol 縺ｮ隧ｳ邏ｰ縺ｯ [MulmoChat](https://github.com/receptron/MulmoChat) 繧貞盾辣ｧ縺励※縺上□縺輔＞縲・
## 繝ｩ繧､繧ｻ繝ｳ繧ｹ

MIT License

## 隰晁ｾ・
- [MulmoChat / GUI Chat Protocol](https://github.com/receptron/MulmoChat) 窶・UI謠千､ｺ繝励Ο繝医さ繝ｫ縺ｮ逹諠ｳ蜈・- [Moorebot Scout](https://www.moorebot.com/) 窶・蟇ｾ雎｡繝ｭ繝懊ャ繝医・繝ｩ繝・ヨ繝輔か繝ｼ繝
- [Anthropic Claude API](https://docs.anthropic.com/) 窶・LLM繝舌ャ繧ｯ繧ｨ繝ｳ繝・
