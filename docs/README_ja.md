# SCOUT x MulmoChat

**対話型ロボット運用インタフェース — GUI Chat Protocol for Robot Operation**

[English version](../README.md)

LLMの対話の中にカメラ画像・状態パネル・選択肢フォームがインラインで出現する、新しいロボット運用UIの実装です。

<p align="center">
  <img src="images/screenshot_status.png" width="400" alt="Status Panel" />
  <img src="images/screenshot_action.png" width="400" alt="Action Result" />
</p>

## 概要

巡回ロボット（[Moorebot Scout](https://www.moorebot.com/)）を、自然言語で対話しながら操作・監視するシステムです。

**従来のロボット運用UIの課題:**
- ダッシュボード型: 異常時にログを読み、別画面で操作 → 状況把握に時間がかかる
- LLMテキストのみ: 対話はできるがUI表示がない → 画像比較や構造化された選択ができない

**本システムのアプローチ: GUI Chat Protocol**

[MulmoChat](https://github.com/receptron/MulmoChat) の GUI Chat Protocol の思想を応用し、LLMの function call を拡張します。各ツールの実行結果が **「LLMへのテキスト」＋「UIへのリッチデータ」** の2種類を同時に返すことで、チャットの流れを壊さずにカメラ画像・状態パネル・操作ボタンがインラインで出現します。

```
ユーザー: 「状態を見せて」
    ↓
LLM → function call: getScoutStatus()
    ↓
ツール実行 → 2種類のデータを返す:
  ├── llm_text: "status=idle, position=x:1.2 y:4.5"  → LLMの次の判断材料
  └── gui_data: {type:"status_panel", image:..., odom:...}  → UIにパネル表示
    ↓
チャット内にカメラ画像＋状態テーブルが出現 + LLMが自然言語で解説
```

## 特徴

- **対話とUIの統合** — カメラ画像・状態パネル・選択肢がチャット内にインライン表示される
- **LLMは判断のみ** — 制御はすべてROS Service経由。LLMが暴走してもロボットは安全
- **LLM非依存** — Claude / GPT / Ollama 等に差し替え可能（`llm_client.py` のみ変更）
- **軽量** — Python + Flask。Python 3.8+で動作
- **構造化ログ** — 全操作がJSON Lines形式で記録され、研究評価に直結
- **拡張容易** — ツール定義を追加するだけで新機能が使える

## アーキテクチャ

```
+----------------------------------------------+
|  Operator (Browser)                          |
|  チャット入力 → GUIパネル表示                   |
+--------------+-------------------------------+
               | HTTP
+--------------v-------------------------------+
|  app_flask.py  (Flask Server)                |
|  HTML/JS が gui_data をインラインレンダリング     |
+--------------+-------------------------------+
               |
+--------------v-------------------------------+
|  llm_client.py  (Claude API / tool_use loop) |
|  ユーザー発話 → LLM → function call → 繰り返し  |
+--------------+-------------------------------+
               | function call
+--------------v-------------------------------+
|  tool_executor.py  ★ GUI Chat Protocol 核心   |
|  ツール実行 → { llm_text + gui_data } を返す    |
+--------------+-------------------------------+
               |
+--------------v-------------------------------+
|  ros_client.py  (rospy)                      |
|  /UtilNode/algo_move, algo_action            |
|  /NavPathNode/nav_path_start, nav_list_path  |
+--------------+-------------------------------+
               | ROS Service Call
+--------------v-------------------------------+
|  SCOUT (Moorebot Scout / roller_eye ROS)     |
|  Camera, Odom, Motors, Sensors               |
+----------------------------------------------+
```

## デモ

| 入力 | 動作 |
|---|---|
| 「状態を見せて」 | カメラ画像＋位置＋速度＋Nav状態のパネル表示 |
| 「0.2m前進して」 | Before/After画像比較付きの移動結果表示 |
| 「右に30度回って」 | algo_action実行＋結果パネル |
| 「ルート一覧を見せて」 | 登録済みパトロールルートのクリック可能リスト |
| 「巡回して」 | パトロール開始＋進捗パネル |
| 異常発生時 | LLMが自動的に選択肢フォームを生成・提示 |

## セットアップ

### 必要なもの

- Moorebot Scout（ROS接続済み）
- PC（Ubuntu, ROS Noetic, Python 3.8+）
- Anthropic API キー（[取得方法](https://console.anthropic.com/)）

### インストール

```bash
git clone https://github.com/YOUR_USERNAME/scout-mulmochat.git
cd scout-mulmochat

pip3 install -r requirements.txt
```

### モックモードで動作確認（ROS不要）

```bash
export ANTHROPIC_API_KEY=sk-ant-your-key
python3 app_flask.py --mock
# ブラウザで http://localhost:7860
```

### 実機接続

```bash
# ROS環境が設定されたターミナルで
export ROS_MASTER_URI=http://<SCOUT_IP>:11311
export ANTHROPIC_API_KEY=sk-ant-your-key
python3 app_flask.py
```

## ファイル構成

```
scout-mulmochat/
├── app_flask.py          # Flask チャットUI + GUIレンダリング（メインエントリ）
├── llm_client.py         # LLM通信 + tool_use ループ
├── tool_definitions.py   # ツール定義 + System Prompt
├── tool_executor.py      # ツール実行（GUI Chat Protocol 核心）
├── ros_client.py         # SCOUT ROS 通信クライアント
├── interaction_log.py    # 構造化インタラクションログ
├── docs/
│   ├── images/           # スクリーンショット
│   ├── api-reference.md  # SCOUT API リファレンス
│   └── README_ja.md      # 日本語版README（このファイル）
├── requirements.txt
├── LICENSE
└── README.md             # 英語版README
```

## GUI Chat Protocol の設計パターン

本プロジェクトの核心は `tool_executor.py` にあります。各ツールが返すデータ構造：

```python
def execute_tool(tool_name, tool_input):
    # ... ツール実行 ...
    return {
        "llm_text": "status=idle, battery=85%",    # LLMの思考材料
        "gui_data": {                               # UIにパネル表示
            "type": "status_panel",                 # パネル種別
            "camera_image_b64": "...",              # base64画像
            "odom": {"x": 1.2, "y": 4.5, ...},    # 構造化データ
        }
    }
```

`gui_data.type` によってフロントエンドのレンダラが決まります：

| type | 表示内容 |
|---|---|
| `status_panel` | カメラ画像＋状態テーブル |
| `action_result` | Before/After画像比較＋成否 |
| `decision_form` | クリック可能な選択肢フォーム |
| `route_list` | パトロールルート一覧 |
| `patrol_started` | パトロール開始通知 |
| `patrol_status` | パトロール進捗＋カメラ画像 |
| `error` | エラー表示 |

**新しいツールの追加方法（3ステップ）:**
1. `tool_definitions.py` にツール定義を追加
2. `tool_executor.py` に `_exec_ツール名` メソッドを追加
3. `app_flask.py` の JavaScript に `render` 関数を追加

LLM側の変更は不要です。

## 他のロボットへの適用

このアーキテクチャは SCOUT 固有ではありません。`ros_client.py` を差し替えれば他のROSロボットにも適用できます：

```python
# 例: TurtleBot3 用
class TurtleBot3ROSClient:
    def algo_move(self, y_distance, speed=0.15):
        # cmd_vel に変換して publish
        ...
    def get_camera_image_b64(self):
        # /camera/rgb/image_raw を取得
        ...
```

ROS以外のロボット（HTTP API、シリアル通信等）でも、同じインタフェースを実装すれば利用可能です。

## 研究的背景

本システムは以下の研究課題に取り組んでいます：

- **Mixed-Initiative Robot Operation**: 異常時にロボット・LLM・オペレータが協調して復帰
- **Explainable Robot Operation**: テキスト生成ではなくUI提示で説明可能性を担保
- **Structured Interaction Logging**: 人の介入をJSON形式で自動記録し評価可能に

GUI Chat Protocol の詳細は [MulmoChat](https://github.com/receptron/MulmoChat) を参照してください。

## ライセンス

MIT License

## 謝辞

- [MulmoChat / GUI Chat Protocol](https://github.com/receptron/MulmoChat) — UI提示プロトコルの着想元
- [Moorebot Scout](https://www.moorebot.com/) — 対象ロボットプラットフォーム
- [Anthropic Claude API](https://docs.anthropic.com/) — LLMバックエンド
