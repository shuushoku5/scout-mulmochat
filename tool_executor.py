#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tool_executor.py — ツール実行エンジン

GUI Chat Protocol の核心実装:
  LLM が tool_use で呼んだツールを実行し、
  「LLM返却テキスト」と「UI表示用データ」を分離して返す。

返り値の形式:
  {
    "llm_text": "...",          # LLMに返すテキスト（次の思考材料）
    "gui_data": {               # チャットUIに表示するリッチデータ
        "type": "status_panel" | "action_result" | "decision_form" | ...,
        ...
    }
  }
"""

import time
from typing import Any, Dict, Optional


class ToolExecutor:
    """ツール実行を管理するクラス"""

    def __init__(self, ros_client):
        """
        Args:
            ros_client: ScoutROSClient インスタンス
        """
        self.ros = ros_client
        self._pending_decision: Optional[Dict] = None  # askOperatorDecision の未回答状態

    def execute(self, tool_name: str, tool_input: Dict[str, Any]) -> Dict[str, Any]:
        """
        ツールを実行し、LLMテキスト + GUIデータ を返す。

        Args:
            tool_name: ツール名
            tool_input: LLMからの入力パラメータ

        Returns:
            {"llm_text": str, "gui_data": dict}
        """
        handler = getattr(self, f"_exec_{tool_name}", None)
        if handler is None:
            return {
                "llm_text": f"Error: unknown tool '{tool_name}'",
                "gui_data": {"type": "error", "message": f"Unknown tool: {tool_name}"},
            }

        try:
            return handler(tool_input)
        except Exception as e:
            return {
                "llm_text": f"Error executing {tool_name}: {str(e)}",
                "gui_data": {"type": "error", "tool": tool_name, "message": str(e)},
            }

    # ================================================================
    # 個別ツール実装
    # ================================================================

    def _exec_getScoutStatus(self, _input: Dict) -> Dict[str, Any]:
        """SCOUT状態取得 → ステータスパネルをUIに表示"""
        status = self.ros.get_status()

        # LLM 用テキスト（画像は含めない、テキスト情報のみ）
        odom_str = "unavailable"
        if status.odom:
            o = status.odom
            odom_str = f"x={o['x']:.2f} y={o['y']:.2f} yaw={o['yaw']:.1f}rad"

        nav_str = {0: "idle", 1: "running", 2: "paused", None: "unknown"}.get(
            status.nav_status, f"code={status.nav_status}"
        )

        llm_text = (
            f"SCOUT status at {time.strftime('%H:%M:%S')}:\n"
            f"  position: {odom_str}\n"
            f"  moving: {status.is_moving}\n"
            f"  nav_status: {nav_str}\n"
            f"  camera: {'available' if status.camera_image_b64 else 'no image'}\n"
        )

        # GUI 用データ（画像含む）
        gui_data = {
            "type": "status_panel",
            "timestamp": status.timestamp,
            "odom": status.odom,
            "is_moving": status.is_moving,
            "nav_status": status.nav_status,
            "nav_status_label": nav_str,
            "camera_image_b64": status.camera_image_b64,  # base64 JPEG
            "camera_image_shape": status.camera_image_shape,
        }

        return {"llm_text": llm_text, "gui_data": gui_data}

    def _exec_moveCorrection(self, inp: Dict) -> Dict[str, Any]:
        """前後移動 → 結果パネルをUIに表示"""
        distance = float(inp.get("distance_m", 0.0))
        speed = float(inp.get("speed", 0.15))

        # 実行前の画像
        before_img = self.ros.get_camera_image_b64()

        # algo_move 実行
        result = self.ros.algo_move(y_distance=distance, speed=speed)

        # 停止待ち
        stopped = False
        if result["success"]:
            stopped = self.ros.wait_stop(timeout=5.0)

        # 実行後の画像
        after_img = self.ros.get_camera_image_b64()

        direction = "前進" if distance > 0 else "後退"
        llm_text = (
            f"moveCorrection result:\n"
            f"  direction: {direction} {abs(distance):.2f}m at {speed:.2f}m/s\n"
            f"  success: {result['success']}\n"
            f"  ret: {result['ret']}\n"
            f"  stopped: {stopped}\n"
        )
        if result["error"]:
            llm_text += f"  error: {result['error']}\n"

        gui_data = {
            "type": "action_result",
            "action": "moveCorrection",
            "params": {"distance_m": distance, "speed": speed},
            "success": result["success"],
            "ret": result["ret"],
            "stopped": stopped,
            "error": result["error"],
            "before_image_b64": before_img,
            "after_image_b64": after_img,
            "direction_label": f"{direction} {abs(distance):.2f}m",
        }

        return {"llm_text": llm_text, "gui_data": gui_data}

    def _exec_rotateCorrection(self, inp: Dict) -> Dict[str, Any]:
        """回転（algo_action使用）→ 結果パネルをUIに表示"""
        angle = float(inp.get("angle_deg", 0.0))
        speed = float(inp.get("speed", 0.4))

        before_img = self.ros.get_camera_image_b64()
        result = self.ros.algo_roll(angle_deg=angle, rotated_speed=speed)

        stopped = False
        if result["success"]:
            # algo_action は時間で終わるので、少し待ってから停止確認
            import time as _time
            _time.sleep((result.get("time_ms", 800) / 1000.0) + 0.5)
            stopped = self.ros.wait_stop(timeout=3.0)

        after_img = self.ros.get_camera_image_b64()

        direction = "左回転" if angle > 0 else "右回転"
        llm_text = (
            f"rotateCorrection result:\n"
            f"  direction: {direction} {abs(angle):.1f}°\n"
            f"  success: {result['success']}\n"
            f"  ret: {result['ret']}\n"
            f"  time_ms: {result.get('time_ms', '?')}\n"
            f"  stopped: {stopped}\n"
        )
        if result.get("error"):
            llm_text += f"  error: {result['error']}\n"

        gui_data = {
            "type": "action_result",
            "action": "rotateCorrection",
            "params": {"angle_deg": angle, "speed": speed},
            "success": result["success"],
            "ret": result["ret"],
            "stopped": stopped,
            "error": result["error"],
            "before_image_b64": before_img,
            "after_image_b64": after_img,
            "direction_label": f"{direction} {abs(angle):.1f}°",
        }

        return {"llm_text": llm_text, "gui_data": gui_data}

    def _exec_stopScout(self, _input: Dict) -> Dict[str, Any]:
        """停止"""
        result = self.ros.nav_cancel()

        llm_text = f"stopScout: success={result['success']}"
        if result["error"]:
            llm_text += f", error={result['error']}"

        gui_data = {
            "type": "action_result",
            "action": "stopScout",
            "success": result["success"],
            "error": result["error"],
        }

        return {"llm_text": llm_text, "gui_data": gui_data}

    # ── Patrol Tools（Phase 2）──

    def _exec_showPatrolRoutes(self, _input: Dict) -> Dict[str, Any]:
        """登録済みパトロールルート一覧"""
        result = self.ros.nav_list_path()

        routes = result.get("routes", [])
        if routes:
            route_lines = []
            for i, r in enumerate(routes):
                route_lines.append(
                    f"  {i+1}. \"{r['name']}\" (size={r['size']}, created={r['created']})"
                )
            llm_text = f"Available patrol routes ({len(routes)}):\n" + "\n".join(route_lines)
        else:
            llm_text = "No patrol routes registered."

        gui_data = {
            "type": "route_list",
            "routes": routes,
            "count": len(routes),
            "error": result.get("error"),
        }

        return {"llm_text": llm_text, "gui_data": gui_data}

    def _exec_startPatrol(self, inp: Dict) -> Dict[str, Any]:
        """パトロール開始"""
        route_name = inp.get("route_name", "")
        from_outside = inp.get("from_outside", False)

        if not route_name:
            return {
                "llm_text": "Error: route_name is required",
                "gui_data": {"type": "error", "message": "ルート名が指定されていません"},
            }

        result = self.ros.nav_path_start(
            name=route_name,
            is_from_out_start=1 if from_outside else 0,
        )

        llm_text = (
            f"startPatrol: route=\"{route_name}\", "
            f"from_outside={from_outside}, "
            f"success={result['success']}"
        )
        if result.get("error"):
            llm_text += f", error={result['error']}"

        gui_data = {
            "type": "patrol_started",
            "route_name": route_name,
            "from_outside": from_outside,
            "success": result["success"],
            "error": result.get("error"),
        }

        return {"llm_text": llm_text, "gui_data": gui_data}

    def _exec_stopPatrol(self, _input: Dict) -> Dict[str, Any]:
        """パトロール停止"""
        result = self.ros.nav_patrol_stop()

        llm_text = f"stopPatrol: success={result['success']}"
        if result.get("error"):
            llm_text += f", error={result['error']}"

        gui_data = {
            "type": "action_result",
            "action": "stopPatrol",
            "success": result["success"],
            "error": result.get("error"),
            "direction_label": "パトロール停止",
        }

        return {"llm_text": llm_text, "gui_data": gui_data}

    def _exec_getPatrolStatus(self, _input: Dict) -> Dict[str, Any]:
        """パトロール状態取得"""
        nav = self.ros.nav_get_status()
        img_b64 = self.ros.get_camera_image_b64()
        odom = self.ros.get_odom()

        status_code = nav.get("status")
        status_map = {
            0: "idle（待機中）",
            1: "running（巡回中）",
            2: "paused（一時停止）",
            3: "returning（帰還中）",
        }
        status_label = status_map.get(status_code, f"unknown (code={status_code})")

        odom_str = ""
        if odom:
            odom_str = f"x={odom.x:.2f} y={odom.y:.2f} yaw={odom.yaw:.1f}rad"

        llm_text = (
            f"Patrol status:\n"
            f"  nav_status: {status_label}\n"
            f"  position: {odom_str or 'unavailable'}\n"
        )

        gui_data = {
            "type": "patrol_status",
            "nav_status": status_code,
            "nav_status_label": status_label,
            "odom": {"x": odom.x, "y": odom.y, "yaw": odom.yaw} if odom else None,
            "camera_image_b64": img_b64,
        }

        return {"llm_text": llm_text, "gui_data": gui_data}

    def _exec_askOperatorDecision(self, inp: Dict) -> Dict[str, Any]:
        """
        オペレータへの選択肢提示。
        UIが選択肢をレンダリングし、ユーザーの選択を待つ。
        """
        question = inp.get("question", "どうしますか？")
        options = inp.get("options", [])

        # 未回答状態を記録（UI側で回収される）
        self._pending_decision = {
            "question": question,
            "options": options,
            "asked_at": time.time(),
        }

        option_labels = ", ".join([f"{o['id']}({o['label']})" for o in options])
        llm_text = f"Waiting for operator decision: {question}\nOptions: {option_labels}"

        gui_data = {
            "type": "decision_form",
            "question": question,
            "options": options,
        }

        return {"llm_text": llm_text, "gui_data": gui_data}

    # ================================================================
    # ユーティリティ
    # ================================================================

    def get_pending_decision(self) -> Optional[Dict]:
        return self._pending_decision

    def clear_pending_decision(self):
        self._pending_decision = None
