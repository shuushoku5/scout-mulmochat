#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tool_definitions.py — LLM に渡すツール定義（Claude API tool_use 形式）

GUI Chat Protocol の核心:
  各ツールの実行結果は2つのデータを返す
    1. llm_text  → LLMに返すテキスト（次の判断材料）
    2. gui_data  → チャットUIに表示するリッチデータ（画像・パネル・選択肢等）
"""

# ============================================================
# Claude API に渡す tools リスト
# ============================================================
SCOUT_TOOLS = [
    {
        "name": "getScoutStatus",
        "description": (
            "SCOUTの現在状態を取得する。"
            "カメラ画像・位置(odom)・バッテリー・移動中かどうか・ナビ状態を返す。"
            "ユーザーが「状態を見せて」「今どうなってる？」と聞いた時に使う。"
        ),
        "input_schema": {
            "type": "object",
            "properties": {},
            "required": [],
        },
    },
    {
        "name": "moveCorrection",
        "description": (
            "SCOUTを前後に指定距離だけ移動させる（algo_move）。"
            "distance_m が正なら前進、負なら後退。最大1.0m。"
            "ユーザーが「前に進んで」「0.3m後退して」などと言った時に使う。"
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "distance_m": {
                    "type": "number",
                    "description": "移動距離(m)。正=前進、負=後退。範囲: -1.0〜1.0",
                },
                "speed": {
                    "type": "number",
                    "description": "移動速度(m/s)。省略時0.15",
                },
            },
            "required": ["distance_m"],
        },
    },
    {
        "name": "rotateCorrection",
        "description": (
            "SCOUTを指定角度だけ回転させる（algo_roll）。"
            "angle_deg が正なら左回転（反時計回り）、負なら右回転。最大180度。"
            "ユーザーが「右に30度回って」「左を向いて」などと言った時に使う。"
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "angle_deg": {
                    "type": "number",
                    "description": "回転角度(度)。正=左回転、負=右回転。範囲: -180〜180",
                },
                "speed": {
                    "type": "number",
                    "description": "回転速度(rad/s)。省略時0.3",
                },
            },
            "required": ["angle_deg"],
        },
    },
    {
        "name": "stopScout",
        "description": (
            "SCOUTの現在の動作を停止する（nav_cancel）。"
            "ユーザーが「止まって」「停止」と言った時に使う。"
        ),
        "input_schema": {
            "type": "object",
            "properties": {},
            "required": [],
        },
    },
    {
        "name": "showPatrolRoutes",
        "description": (
            "登録済みのパトロールルート一覧を表示する。"
            "ルート名・サイズ・作成日時を返す。"
            "ユーザーが「ルート一覧」「どのルートがある？」と聞いた時に使う。"
        ),
        "input_schema": {
            "type": "object",
            "properties": {},
            "required": [],
        },
    },
    {
        "name": "startPatrol",
        "description": (
            "指定したルート名でパトロールを開始する。"
            "まず showPatrolRoutes で利用可能なルートを確認してから使う。"
            "ユーザーが「巡回して」「パトロール開始」と言った時に使う。"
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "route_name": {
                    "type": "string",
                    "description": "パトロールルート名（showPatrolRoutesで表示されたもの）",
                },
                "from_outside": {
                    "type": "boolean",
                    "description": "充電台の外から開始するか。省略時false（充電台から開始）",
                },
            },
            "required": ["route_name"],
        },
    },
    {
        "name": "stopPatrol",
        "description": (
            "現在実行中のパトロールを停止する。"
            "ユーザーが「巡回やめて」「パトロール停止」と言った時に使う。"
        ),
        "input_schema": {
            "type": "object",
            "properties": {},
            "required": [],
        },
    },
    {
        "name": "getPatrolStatus",
        "description": (
            "パトロールの進行状態を取得する。"
            "ユーザーが「巡回の状況は？」「今どこまで進んだ？」と聞いた時に使う。"
        ),
        "input_schema": {
            "type": "object",
            "properties": {},
            "required": [],
        },
    },
    {
        "name": "askOperatorDecision",
        "description": (
            "オペレータに選択肢を提示して判断を求める。"
            "異常時や判断が必要な場面で使う。"
            "選択肢は最大5つ。各選択肢にはラベルと説明を付ける。"
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "question": {
                    "type": "string",
                    "description": "オペレータへの質問文",
                },
                "options": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "id": {"type": "string", "description": "選択肢ID (例: 'retry', 'skip')"},
                            "label": {"type": "string", "description": "選択肢のラベル"},
                            "description": {"type": "string", "description": "選択肢の説明"},
                        },
                        "required": ["id", "label"],
                    },
                    "description": "選択肢のリスト（2〜5個）",
                },
            },
            "required": ["question", "options"],
        },
    },
]


# ============================================================
# System Prompt（Role 定義）
# ============================================================
SCOUT_SYSTEM_PROMPT = """\
あなたは巡回ロボット「SCOUT」の対話型オペレーションアシスタントです。

## あなたの役割
- SCOUTの状態をオペレータに分かりやすく伝える
- オペレータの指示に基づいてロボットを操作する
- 異常時には状況を診断し、選択肢を提示する
- すべてのやりとりを通じて、オペレータが「何が起きているか」を理解できるようにする

## 使えるツール
1. **getScoutStatus**: 状態取得（画像・位置・バッテリー等）
2. **moveCorrection**: 前後移動（algo_move）
3. **rotateCorrection**: 回転（algo_action）
4. **stopScout**: 停止（nav_cancel）
5. **showPatrolRoutes**: 登録済みパトロールルート一覧を表示
6. **startPatrol**: 指定ルートでパトロール開始
7. **stopPatrol**: パトロール停止
8. **getPatrolStatus**: パトロール進行状態を取得
9. **askOperatorDecision**: オペレータに選択肢を提示

## 重要なルール
- 移動指示が曖昧な場合は、距離を確認してから実行する
- 大きな移動（0.5m以上）の前には確認を取る
- パトロール開始前は showPatrolRoutes でルートを確認する
- パトロール中は定期的に状態確認できることを伝える
- 操作後は結果を簡潔に報告する
- エラーが発生した場合は、何が起きたかを分かりやすく説明する
- 日本語で応答する
"""
