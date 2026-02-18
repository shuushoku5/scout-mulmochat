#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
llm_client.py — LLM通信クライアント（Claude API tool_use対応）

tool_use ループ:
  1. ユーザーメッセージ + tools を Claude に送信
  2. Claude が tool_use を返したら → ToolExecutor で実行
  3. tool_result を Claude に返す → 繰り返し
  4. Claude がテキストだけ返したら → 完了
"""

import os
from typing import List, Dict, Any, Tuple, Optional
from dataclasses import dataclass, field


@dataclass
class ChatTurn:
    """1ターンの会話データ"""
    role: str             # "user" | "assistant"
    text: str             # 表示用テキスト
    gui_data: Optional[Dict] = None   # GUI表示用データ（あれば）
    tool_calls: List[Dict] = field(default_factory=list)  # デバッグ用


class LLMClient:
    """Claude API を使った対話管理"""

    def __init__(
        self,
        tool_executor,
        system_prompt: str,
        tools: List[Dict],
        model: str = "claude-sonnet-4-20250514",
        max_tokens: int = 1024,
    ):
        self.tool_executor = tool_executor
        self.system_prompt = system_prompt
        self.tools = tools
        self.model = model
        self.max_tokens = max_tokens

        # 会話履歴（Claude API messages 形式）
        self._messages: List[Dict] = []

        # Anthropic client
        try:
            from anthropic import Anthropic
            api_key = os.environ.get("ANTHROPIC_API_KEY")
            if not api_key:
                raise ValueError("ANTHROPIC_API_KEY environment variable not set")
            self.client = Anthropic(api_key=api_key)
        except ImportError:
            raise ImportError("pip install anthropic が必要です")

    def chat(self, user_message: str) -> Tuple[str, List[Dict]]:
        """
        ユーザーメッセージを送り、最終応答テキストと全GUI dataのリストを返す。

        tool_use ループを自動的に処理する。

        Returns:
            (assistant_text, gui_data_list)
            - assistant_text: 最終的なアシスタントの応答テキスト
            - gui_data_list: ツール実行で生成されたGUIデータのリスト
        """
        # ユーザーメッセージを履歴に追加
        self._messages.append({
            "role": "user",
            "content": user_message,
        })

        gui_data_list = []

        # tool_use ループ
        while True:
            # Claude API 呼び出し
            response = self.client.messages.create(
                model=self.model,
                max_tokens=self.max_tokens,
                system=self.system_prompt,
                tools=self.tools,
                messages=self._messages,
            )

            # レスポンスのcontent blockを処理
            assistant_text_parts = []
            tool_uses = []

            for block in response.content:
                if block.type == "text":
                    assistant_text_parts.append(block.text)
                elif block.type == "tool_use":
                    tool_uses.append({
                        "id": block.id,
                        "name": block.name,
                        "input": block.input,
                    })

            # アシスタントの応答を履歴に追加（content blocks をそのまま保持）
            self._messages.append({
                "role": "assistant",
                "content": [b.model_dump() for b in response.content],
            })

            # ツール呼び出しがなければ完了
            if not tool_uses:
                final_text = "\n".join(assistant_text_parts)
                return final_text, gui_data_list

            # ツールを実行し、結果を返す
            tool_results = []
            for tu in tool_uses:
                result = self.tool_executor.execute(tu["name"], tu["input"])
                gui_data_list.append(result["gui_data"])

                tool_results.append({
                    "type": "tool_result",
                    "tool_use_id": tu["id"],
                    "content": result["llm_text"],
                })

            # tool_result を履歴に追加
            self._messages.append({
                "role": "user",
                "content": tool_results,
            })

            # ループ継続（Claude が次の応答を返す）

    def reset_history(self):
        """会話履歴をクリア"""
        self._messages = []

    def get_message_count(self) -> int:
        return len(self._messages)
