# SCOUT ÁEMulmoChat

**対話型ロボット運用インタフェース  EGUI Chat Protocol for Robot Operation**

LLMの対話の中にカメラ画像�E状態パネル・選択肢フォームがインラインで出現する、新しいロボット運用UIの実裁E��す、E
<p align="center">
  <img src="docs/images/screenshot_status.png" width="400" alt="Status Panel" />
  <img src="docs/images/screenshot_action.png" width="400" alt="Action Result" />
</p>

## 概要E
巡回ロボット！EMoorebot Scout](https://www.moorebot.com/)�E�を、�E然言語で対話しながら操作�E監視するシスチE��です、E
**従来のロボット運用UIの課顁E**
- ダチE��ュボ�Eド型: 異常時にログを読み、別画面で操佁EↁE状況把握に時間がかかる
- LLMチE��スト�Eみ: 対話はできるがUI表示がなぁEↁE画像比輁E��構造化された選択ができなぁE
**本シスチE��のアプローチE GUI Chat Protocol**

[MulmoChat](https://github.com/receptron/MulmoChat) の GUI Chat Protocol の思想を応用し、LLMの function call を拡張します。各チE�Eルの実行結果ぁE**「LLMへのチE��スト」＋「UIへのリチE��チE�Eタ、E* の2種類を同時に返すことで、チャチE��の流れを壊さずにカメラ画像�E状態パネル・操作�Eタンがインラインで出現します、E
```
ユーザー: 「状態を見せて、E    ↁELLM ↁEfunction call: getScoutStatus()
    ↁEチE�Eル実衁EↁE2種類�EチE�Eタを返す:
  ├── llm_text: "status=idle, position=x:1.2 y:4.5"  ↁELLMの次の判断材料
  └── gui_data: {type:"status_panel", image:..., odom:...}  ↁEUIにパネル表示
    ↁEチャチE��冁E��カメラ画像＋状態テーブルが�E現 + LLMが�E然言語で解説
```

## 特徴

- **対話とUIの統吁E*  Eカメラ画像�E状態パネル・選択肢がチャチE��冁E��インライン表示されめE- **LLMは判断のみ**  E制御はすべてROS Service経由、ELMが暴走してもロボット�E安�E
- **LLM非依孁E*  EClaude / GPT / Ollama 等に差し替え可能�E�Ellm_client.py` のみ変更�E�E- **軽釁E*  EPython + Flask。Python 3.8+で動佁E- **構造化ログ**  E全操作がJSON Lines形式で記録され、研究評価に直絁E- **拡張容昁E*  EチE�Eル定義を追加するだけで新機�Eが使える

## アーキチE��チャ

```
┌──────────────────────────────────────────────━E━E Operator (Browser)                          ━E━E チャチE��入劁EↁEGUIパネル表示                   ━E└──────────────┬───────────────────────────────━E               ━EHTTP
┌──────────────▼───────────────────────────────━E━E app_flask.py  (Flask Server)                ━E━E HTML/JS ぁEgui_data をインラインレンダリング     ━E└──────────────┬───────────────────────────────━E               ━E┌──────────────▼───────────────────────────────━E━E llm_client.py  (Claude API / tool_use loop) ━E━E ユーザー発話 ↁELLM ↁEfunction call ↁE繰り返し  ━E└──────────────┬───────────────────────────────━E               ━Efunction call
┌──────────────▼───────────────────────────────━E━E tool_executor.py  ☁EGUI Chat Protocol 核忁E  ━E━E チE�Eル実衁EↁE{ llm_text + gui_data } を返す    ━E└──────────────┬───────────────────────────────━E               ━E┌──────────────▼───────────────────────────────━E━E ros_client.py  (rospy)                      ━E━E /UtilNode/algo_move, algo_action            ━E━E /NavPathNode/nav_path_start, nav_list_path  ━E└──────────────┬───────────────────────────────━E               ━EROS Service Call
┌──────────────▼───────────────────────────────━E━E SCOUT (Moorebot Scout / roller_eye ROS)     ━E━E Camera, Odom, Motors, Sensors               ━E└──────────────────────────────────────────────━E```

## チE��

| 入劁E| 動佁E|
|---|---|
| 「状態を見せて、E| 📡 カメラ画像＋位置�E�速度�E�Nav状態�Eパネル表示 |
| 、E.2m前進して、E| ✁EBefore/After画像比輁E��きの移動結果表示 |
| 「右に30度回って、E| ✁Ealgo_action実行＋結果パネル |
| 「ルート一覧を見せて、E| 📋 登録済みパトロールルート�EクリチE��可能リスチE|
| 「巡回して、E| 🚶 パトロール開始＋進捗パネル |
| 異常発生時 | 🤁ELLMが�E動的に選択肢フォームを生成�E提示 |

## セチE��アチE�E

### 忁E��なも�E

- Moorebot Scout�E�EOS接続済み�E�E- PC�E�Ebuntu, ROS Noetic, Python 3.8+�E�E- Anthropic API キー�E�E取得方法](https://console.anthropic.com/)�E�E
### インスト�Eル

```bash
git clone https://github.com/YOUR_USERNAME/scout-mulmochat.git
cd scout-mulmochat

pip3 install anthropic flask
```

### モチE��モードで動作確認！EOS不要E��E
```bash
export ANTHROPIC_API_KEY=<YOUR_ANTHROPIC_API_KEY>
python3 app_flask.py --mock
# ブラウザで http://localhost:7860
```

### 実機接綁E
```bash
# ROS環墁E��設定されたターミナルで
export ROS_MASTER_URI=http://<SCOUT_IP>:11311
export ANTHROPIC_API_KEY=<YOUR_ANTHROPIC_API_KEY>
python3 app_flask.py
```

## ファイル構�E

```
scout-mulmochat/
├── app_flask.py          # Flask チャチE��UI + GUIレンダリング�E�メインエントリ�E�E├── llm_client.py         # LLM通信 + tool_use ルーチE├── tool_definitions.py   # チE�Eル定義 + System Prompt
├── tool_executor.py      # チE�Eル実行！EUI Chat Protocol 核忁E��E├── ros_client.py         # SCOUT ROS 通信クライアンチE├── interaction_log.py    # 構造化インタラクションログ
├── docs/
━E  ├── images/           # スクリーンショチE��
━E  └── api-reference.md  # SCOUT API リファレンス
├── LICENSE
└── README.md
```

## GUI Chat Protocol の設計パターン

本プロジェクト�E核忁E�E `tool_executor.py` にあります。各チE�Eルが返すチE�Eタ構造�E�E
```python
def execute_tool(tool_name, tool_input):
    # ... チE�Eル実衁E...
    return {
        "llm_text": "status=idle, battery=85%",    # LLMの思老E��斁E        "gui_data": {                               # UIにパネル表示
            "type": "status_panel",                 # パネル種別
            "camera_image_b64": "...",              # base64画僁E            "odom": {"x": 1.2, "y": 4.5, ...},    # 構造化データ
        }
    }
```

`gui_data.type` によってフロントエンド�Eレンダラが決まります！E
| type | 表示冁E�� |
|---|---|
| `status_panel` | カメラ画像＋状態テーブル |
| `action_result` | Before/After画像比輁E���E否 |
| `decision_form` | クリチE��可能な選択肢フォーム |
| `route_list` | パトロールルート一覧 |
| `patrol_started` | パトロール開始通知 |
| `patrol_status` | パトロール進捗＋カメラ画僁E|
| `error` | エラー表示 |

**新しいチE�Eルの追加方法！EスチE��プ！E**
1. `tool_definitions.py` にチE�Eル定義を追加
2. `tool_executor.py` に `_exec_チE�Eル名` メソチE��を追加
3. `app_flask.py` の JavaScript に `render` 関数を追加

LLM側の変更は不要です、E
## 他�Eロボットへの適用

こ�EアーキチE��チャは SCOUT 固有ではありません。`ros_client.py` を差し替えれば他�EROSロボットにも適用できます！E
```python
# 侁E TurtleBot3 用
class TurtleBot3ROSClient:
    def algo_move(self, y_distance, speed=0.15):
        # cmd_vel に変換して publish
        ...
    def get_camera_image_b64(self):
        # /camera/rgb/image_raw を取征E        ...
```

ROS以外�Eロボット！ETTP API、シリアル通信等）でも、同じインタフェースを実裁E��れ�E利用可能です、E
## 研究皁E��景

本シスチE��は以下�E研究課題に取り絁E��でぁE��す！E
- **Mixed-Initiative Robot Operation**: 異常時にロボット�ELLM・オペレータが協調して復帰
- **Explainable Robot Operation**: チE��スト生成ではなくUI提示で説明可能性を担俁E- **Structured Interaction Logging**: 人の介�EをJSON形式で自動記録し評価可能に

GUI Chat Protocol の詳細は [MulmoChat](https://github.com/receptron/MulmoChat) を参照してください、E
## ライセンス

MIT License

## 謝辁E
- [MulmoChat / GUI Chat Protocol](https://github.com/receptron/MulmoChat)  EUI提示プロトコルの着想允E- [Moorebot Scout](https://www.moorebot.com/)  E対象ロボット�EラチE��フォーム
- [Anthropic Claude API](https://docs.anthropic.com/)  ELLMバックエンチE
