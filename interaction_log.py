#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
interaction_log.py — 構造化インタラクションログ

GUI Chat Protocol の設計原則③「すべてログに残す」の実装。
各やりとりを JSON Lines 形式で記録し、論文の評価データに直結させる。
"""

import json
import os
import time
from typing import Any, Dict, List, Optional


class InteractionLogger:
    """構造化インタラクションログ記録"""

    def __init__(self, log_dir: str = "~/scout_mulmochat_logs"):
        self.log_dir = os.path.expanduser(log_dir)
        os.makedirs(self.log_dir, exist_ok=True)

        ts = time.strftime("%Y%m%d_%H%M%S")
        self.session_id = f"session_{ts}"
        self.log_path = os.path.join(self.log_dir, f"{self.session_id}.jsonl")

        self._fp = open(self.log_path, "a")
        self._event_id = 0

        # 評価用カウンタ
        self.stats = {
            "total_turns": 0,
            "tool_calls": 0,
            "errors": 0,
            "decisions_asked": 0,
            "decisions_answered": 0,
            "move_commands": 0,
            "rotate_commands": 0,
            "session_start": time.time(),
        }

        self._log_event("session_start", {"session_id": self.session_id})

    def _log_event(self, event_type: str, data: Dict[str, Any]):
        """1イベントを JSON Lines で記録"""
        entry = {
            "event_id": self._event_id,
            "timestamp": time.time(),
            "timestamp_str": time.strftime("%Y-%m-%d %H:%M:%S"),
            "session_id": self.session_id,
            "event_type": event_type,
            "data": data,
        }
        self._fp.write(json.dumps(entry, ensure_ascii=False) + "\n")
        self._fp.flush()
        self._event_id += 1

    def log_user_message(self, message: str):
        self.stats["total_turns"] += 1
        self._log_event("user_message", {"text": message})

    def log_assistant_response(self, text: str, gui_data_list: List[Dict]):
        # 画像データはログには含めない（巨大になるため）
        sanitized = []
        for gd in gui_data_list:
            gd_copy = {k: v for k, v in gd.items()
                       if not k.endswith("_b64")}  # base64画像を除外
            sanitized.append(gd_copy)

        self._log_event("assistant_response", {
            "text": text,
            "gui_data_count": len(gui_data_list),
            "gui_data_types": [gd.get("type") for gd in gui_data_list],
            "gui_data": sanitized,
        })

    def log_tool_call(self, tool_name: str, tool_input: Dict, result: Dict):
        self.stats["tool_calls"] += 1

        if tool_name == "moveCorrection":
            self.stats["move_commands"] += 1
        elif tool_name == "rotateCorrection":
            self.stats["rotate_commands"] += 1
        elif tool_name == "askOperatorDecision":
            self.stats["decisions_asked"] += 1

        # 画像を除外
        result_sanitized = {k: v for k, v in result.items() if not k.endswith("_b64")}

        self._log_event("tool_call", {
            "tool_name": tool_name,
            "input": tool_input,
            "result_summary": result_sanitized,
        })

    def log_operator_decision(self, question: str, chosen_option: str):
        self.stats["decisions_answered"] += 1
        self._log_event("operator_decision", {
            "question": question,
            "chosen": chosen_option,
        })

    def log_error(self, error_type: str, message: str):
        self.stats["errors"] += 1
        self._log_event("error", {"error_type": error_type, "message": message})

    def get_session_summary(self) -> Dict:
        self.stats["session_duration_sec"] = time.time() - self.stats["session_start"]
        return {**self.stats, "log_path": self.log_path}

    def close(self):
        summary = self.get_session_summary()
        self._log_event("session_end", summary)
        self._fp.close()
        return summary
