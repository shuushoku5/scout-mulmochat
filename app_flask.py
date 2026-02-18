#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
app_flask.py 窶・SCOUT ﾃ・MulmoChat 繝√Ε繝・ヨUI・・lask迚医・霆ｽ驥擾ｼ・
Gradio荳崎ｦ√１ython 3.8 + Flask 縺ｧ蜍穂ｽ懊・襍ｷ蜍・ ANTHROPIC_API_KEY=sk-... python3 app_flask.py --mock
繝悶Λ繧ｦ繧ｶ: http://localhost:7860
"""

import argparse
import json
import os
import sys
import time
import traceback

# Flask
try:
    from flask import Flask, request, jsonify, render_template_string
except ImportError:
    print("Flask 縺悟ｿ・ｦ√〒縺・ pip3 install flask")
    sys.exit(1)

# ============================================================
# HTML 繝・Φ繝励Ξ繝ｼ繝茨ｼ・繝輔ぃ繧､繝ｫ縺ｫ蝓九ａ霎ｼ縺ｿ・・# ============================================================
HTML_TEMPLATE = r"""
<!DOCTYPE html>
<html lang="ja">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>SCOUT ﾃ・MulmoChat</title>
<style>
  * { margin:0; padding:0; box-sizing:border-box; }
  body {
    background: #0a0e17; color: #e2e8f0;
    font-family: -apple-system, 'Segoe UI', sans-serif;
    height: 100vh; display: flex; flex-direction: column;
  }
  header {
    background: #111827; border-bottom: 1px solid #1e2d3d;
    padding: 12px 20px; text-align: center;
  }
  header h1 { font-size: 1.1rem; color: #f8fafc; }
  header h1 span.b { color: #38bdf8; }
  header h1 span.p { color: #818cf8; }
  header p { font-size: 0.75rem; color: #64748b; margin-top: 2px; }

  #chat-area {
    flex: 1; overflow-y: auto; padding: 16px 20px;
  }
  .msg {
    margin: 8px 0; display: flex; gap: 10px;
    max-width: 800px; margin-left: auto; margin-right: auto;
  }
  .msg.user { justify-content: flex-end; }
  .msg.assistant { justify-content: flex-start; }
  .bubble {
    padding: 10px 14px; border-radius: 12px;
    font-size: 0.88rem; line-height: 1.6;
    max-width: 75%;
    word-wrap: break-word;
  }
  .msg.user .bubble {
    background: #1e3a5f; color: #e2e8f0;
    border-bottom-right-radius: 4px;
  }
  .msg.assistant .bubble {
    background: #1a2332; color: #e2e8f0;
    border: 1px solid #1e2d3d;
    border-bottom-left-radius: 4px;
  }

  /* GUI 繝代ロ繝ｫ */
  .gui-panel {
    border: 1px solid #2d3748; border-radius: 8px;
    padding: 12px; margin: 8px 0; background: #0d1117;
  }
  .gui-panel.status { border-color: #38bdf8; }
  .gui-panel.action { border-color: #34d399; }
  .gui-panel.action.fail { border-color: #ef4444; }
  .gui-panel.decision { border-color: #818cf8; }
  .gui-panel .panel-title {
    font-weight: bold; font-size: 0.85rem; margin-bottom: 8px;
  }
  .gui-panel.status .panel-title { color: #38bdf8; }
  .gui-panel.action .panel-title { color: #34d399; }
  .gui-panel.action.fail .panel-title { color: #ef4444; }
  .gui-panel.decision .panel-title { color: #818cf8; }
  .gui-panel img {
    max-width: 100%; border-radius: 4px; margin: 4px 0;
  }
  .gui-panel table {
    width: 100%; font-size: 0.8rem; margin-top: 6px;
  }
  .gui-panel td { padding: 3px 6px; }
  .gui-panel td:first-child { color: #64748b; }

  .img-row { display: flex; gap: 8px; }
  .img-row > div { flex:1; text-align: center; }
  .img-row .cap { font-size: 0.7rem; color: #64748b; }

  .decision-opt {
    background: #1a2332; border: 1px solid #2d3748;
    border-radius: 6px; padding: 8px 12px; margin: 4px 0;
    cursor: pointer; font-size: 0.82rem;
    transition: border-color 0.2s;
  }
  .decision-opt:hover { border-color: #818cf8; }
  .decision-opt .opt-label { color: #38bdf8; font-weight: bold; }
  .decision-opt .opt-desc { color: #94a3b8; font-size: 0.78rem; }

  /* 蜈･蜉帶ｬ・*/
  #input-area {
    background: #111827; border-top: 1px solid #1e2d3d;
    padding: 12px 20px; display: flex; gap: 8px;
    max-width: 840px; margin: 0 auto; width: 100%;
  }
  #msg-input {
    flex: 1; padding: 10px 14px; border-radius: 8px;
    border: 1px solid #2d3748; background: #1a2332;
    color: #e2e8f0; font-size: 0.9rem; outline: none;
  }
  #msg-input:focus { border-color: #38bdf8; }
  #send-btn {
    padding: 10px 20px; border-radius: 8px;
    border: none; background: #2563eb; color: white;
    font-size: 0.9rem; cursor: pointer;
    transition: background 0.2s;
  }
  #send-btn:hover { background: #1d4ed8; }
  #send-btn:disabled { background: #374151; cursor: not-allowed; }

  .loading { color: #64748b; font-style: italic; font-size: 0.85rem; }
</style>
</head>
<body>

<header>
  <h1><span class="b">SCOUT</span> ﾃ・<span class="p">MulmoChat</span></h1>
  <p>GUI Chat Protocol 窶・蟇ｾ隧ｱ蝙九Ο繝懊ャ繝磯°逕ｨ繧､繝ｳ繧ｿ繝輔ぉ繝ｼ繧ｹ</p>
</header>

<div id="chat-area"></div>

<div id="input-area">
  <input id="msg-input" type="text" placeholder="SCOUT縺ｸ縺ｮ謖・､ｺ繧貞・蜉・..・井ｾ・ 迥ｶ諷九ｒ隕九○縺ｦ・・ autocomplete="off" />
  <button id="send-btn" onclick="sendMessage()">騾∽ｿ｡</button>
</div>

<script>
const chatArea = document.getElementById('chat-area');
const msgInput = document.getElementById('msg-input');
const sendBtn = document.getElementById('send-btn');

msgInput.addEventListener('keydown', (e) => {
  if (e.key === 'Enter' && !e.shiftKey) { e.preventDefault(); sendMessage(); }
});

function addMessage(role, html) {
  const div = document.createElement('div');
  div.className = 'msg ' + role;
  div.innerHTML = '<div class="bubble">' + html + '</div>';
  chatArea.appendChild(div);
  chatArea.scrollTop = chatArea.scrollHeight;
}

function renderGuiData(gd) {
  if (!gd || !gd.type) return '';
  if (gd.type === 'status_panel') return renderStatusPanel(gd);
  if (gd.type === 'action_result') return renderActionResult(gd);
  if (gd.type === 'decision_form') return renderDecisionForm(gd);
  if (gd.type === 'route_list') return renderRouteList(gd);
  if (gd.type === 'patrol_started') return renderPatrolStarted(gd);
  if (gd.type === 'patrol_status') return renderPatrolStatus(gd);
  if (gd.type === 'error') return '<div class="gui-panel" style="border-color:#ef4444;color:#ef4444;">笞・・' + (gd.message||'Error') + '</div>';
  return '';
}

function renderStatusPanel(d) {
  let h = '<div class="gui-panel status"><div class="panel-title">藤 SCOUT Status</div>';
  if (d.camera_image_b64) {
    h += '<img src="data:image/jpeg;base64,' + d.camera_image_b64 + '" />';
  }
  h += '<table>';
  if (d.odom) {
    h += '<tr><td>Position</td><td>x=' + d.odom.x.toFixed(2) + ' y=' + d.odom.y.toFixed(2) + '</td></tr>';
    h += '<tr><td>Yaw</td><td>' + d.odom.yaw.toFixed(2) + ' rad</td></tr>';
    h += '<tr><td>Speed</td><td>' + d.odom.v_linear.toFixed(3) + ' m/s</td></tr>';
  }
  const moving = d.is_moving;
  h += '<tr><td>State</td><td style="color:' + (moving ? '#ef4444' : '#34d399') + ';">' + (moving ? '遘ｻ蜍穂ｸｭ' : '蛛懈ｭ｢荳ｭ') + '</td></tr>';
  h += '<tr><td>Nav</td><td>' + (d.nav_status_label || 'unknown') + '</td></tr>';
  h += '</table></div>';
  return h;
}

function renderActionResult(d) {
  const ok = d.success;
  const cls = ok ? 'action' : 'action fail';
  const icon = ok ? '笨・ : '笶・;
  let h = '<div class="gui-panel ' + cls + '"><div class="panel-title">' + icon + ' ' + (d.direction_label || d.action) + '</div>';
  if (d.before_image_b64 || d.after_image_b64) {
    h += '<div class="img-row">';
    if (d.before_image_b64) h += '<div><div class="cap">Before</div><img src="data:image/jpeg;base64,' + d.before_image_b64 + '" /></div>';
    if (d.after_image_b64) h += '<div><div class="cap">After</div><img src="data:image/jpeg;base64,' + d.after_image_b64 + '" /></div>';
    h += '</div>';
  }
  if (!ok && d.error) h += '<div style="color:#ef4444;font-size:0.82rem;">Error: ' + d.error + '</div>';
  h += '</div>';
  return h;
}

function renderDecisionForm(d) {
  let h = '<div class="gui-panel decision"><div class="panel-title">､・' + d.question + '</div>';
  (d.options||[]).forEach(o => {
    h += '<div class="decision-opt" onclick="selectOption(\'' + o.id + '\')">';
    h += '<span class="opt-label">' + o.label + '</span>';
    if (o.description) h += ' <span class="opt-desc">窶・' + o.description + '</span>';
    h += '</div>';
  });
  h += '</div>';
  return h;
}

function renderRouteList(d) {
  const routes = d.routes || [];
  let h = '<div class="gui-panel" style="border-color:#818cf8;"><div class="panel-title" style="color:#818cf8;">搭 繝代ヨ繝ｭ繝ｼ繝ｫ繝ｫ繝ｼ繝井ｸ隕ｧ</div>';
  if (routes.length === 0) {
    h += '<div style="color:#94a3b8;font-size:0.82rem;">逋ｻ骭ｲ貂医∩繝ｫ繝ｼ繝医′縺ゅｊ縺ｾ縺帙ｓ</div>';
  } else {
    routes.forEach((r,i) => {
      h += '<div class="decision-opt" onclick="selectOption(\'' + r.name + '\')" style="cursor:pointer;">';
      h += '<span class="opt-label">' + r.name + '</span>';
      h += ' <span class="opt-desc">窶・size: ' + r.size + '</span>';
      if (r.created) h += ' <span class="opt-desc">/ ' + r.created + '</span>';
      h += '</div>';
    });
  }
  h += '<div style="color:#64748b;font-size:0.7rem;margin-top:6px;">繝ｫ繝ｼ繝亥錐繧偵け繝ｪ繝・け縺ｾ縺溘・縲娯雷笳九〒蟾｡蝗槭＠縺ｦ縲阪→蜈･蜉・/div>';
  h += '</div>';
  return h;
}

function renderPatrolStarted(d) {
  const ok = d.success;
  const color = ok ? '#34d399' : '#ef4444';
  const icon = ok ? '垳' : '笶・;
  let h = '<div class="gui-panel" style="border-color:' + color + ';"><div class="panel-title" style="color:' + color + ';">' + icon + ' 繝代ヨ繝ｭ繝ｼ繝ｫ: ' + d.route_name + '</div>';
  if (ok) {
    h += '<div style="font-size:0.82rem;">繝代ヨ繝ｭ繝ｼ繝ｫ繧帝幕蟋九＠縺ｾ縺励◆縲ゅ悟ｷ｡蝗槭・迥ｶ豕√・・溘阪〒騾ｲ謐励ｒ遒ｺ隱阪〒縺阪∪縺吶・/div>';
  } else {
    h += '<div style="color:#ef4444;font-size:0.82rem;">髢句ｧ九↓螟ｱ謨励＠縺ｾ縺励◆: ' + (d.error || 'unknown') + '</div>';
  }
  h += '</div>';
  return h;
}

function renderPatrolStatus(d) {
  let h = '<div class="gui-panel" style="border-color:#38bdf8;"><div class="panel-title" style="color:#38bdf8;">垳 繝代ヨ繝ｭ繝ｼ繝ｫ迥ｶ諷・/div>';
  if (d.camera_image_b64) {
    h += '<img src="data:image/jpeg;base64,' + d.camera_image_b64 + '" style="max-width:100%;border-radius:4px;margin:4px 0;" />';
  }
  h += '<table style="width:100%;font-size:0.8rem;color:#e2e8f0;">';
  const statusColor = d.nav_status === 1 ? '#34d399' : (d.nav_status === 0 ? '#94a3b8' : '#fb923c');
  h += '<tr><td style="color:#94a3b8;">Status</td><td style="color:' + statusColor + ';">' + (d.nav_status_label || 'unknown') + '</td></tr>';
  if (d.odom) {
    h += '<tr><td style="color:#94a3b8;">Position</td><td>x=' + d.odom.x.toFixed(2) + ' y=' + d.odom.y.toFixed(2) + '</td></tr>';
  }
  h += '</table></div>';
  return h;
}

function selectOption(optId) {
  msgInput.value = optId;
  sendMessage();
}

async function sendMessage() {
  const text = msgInput.value.trim();
  if (!text) return;

  addMessage('user', escapeHtml(text));
  msgInput.value = '';
  sendBtn.disabled = true;

  // loading
  const loadDiv = document.createElement('div');
  loadDiv.className = 'msg assistant';
  loadDiv.innerHTML = '<div class="bubble loading">閠・∴荳ｭ...</div>';
  chatArea.appendChild(loadDiv);
  chatArea.scrollTop = chatArea.scrollHeight;

  try {
    const res = await fetch('/api/chat', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({message: text}),
    });
    const data = await res.json();

    chatArea.removeChild(loadDiv);

    if (data.error) {
      addMessage('assistant', '<span style="color:#ef4444;">笞・・' + escapeHtml(data.error) + '</span>');
    } else {
      let html = '';
      (data.gui_data_list || []).forEach(gd => { html += renderGuiData(gd); });
      if (data.text) html += '<div>' + escapeHtml(data.text).replace(/\n/g, '<br>') + '</div>';
      addMessage('assistant', html || '(蠢懃ｭ斐↑縺・');
    }
  } catch (e) {
    chatArea.removeChild(loadDiv);
    addMessage('assistant', '<span style="color:#ef4444;">笞・・騾壻ｿ｡繧ｨ繝ｩ繝ｼ: ' + e.message + '</span>');
  }

  sendBtn.disabled = false;
  msgInput.focus();
}

function escapeHtml(s) {
  const d = document.createElement('div');
  d.textContent = s;
  return d.innerHTML;
}
</script>

</body>
</html>
"""

# ============================================================
# Mock ROS Client
# ============================================================
class MockROSClient:
    def get_status(self):
        class S:
            timestamp = time.time()
            odom = {"timestamp": time.time(), "x": 1.23, "y": 4.56, "yaw": 0.78, "v_linear": 0.0, "v_angular": 0.0}
            camera_image_b64 = None
            camera_image_shape = None
            is_moving = False
            nav_status = 0
            error = None
        return S()

    def get_camera_image_b64(self):
        return None

    def algo_move(self, y_distance, speed=0.15):
        return {"success": True, "ret": 0, "error": None}

    def algo_roll(self, angle_deg, rotated_speed=0.4):
        angle_rad = abs(angle_deg) * 3.14159 / 180.0
        time_ms = int(angle_rad / rotated_speed * 1000)
        return {"success": True, "ret": 0, "error": None, "time_ms": time_ms, "rotated_speed": rotated_speed}

    def nav_cancel(self):
        return {"success": True, "error": None}

    def nav_list_path(self):
        return {"success": True, "routes": [
            {"name": "hallway_A", "path": "/path/a", "size": 120, "created": "2026-02-10 14:30"},
            {"name": "office_B", "path": "/path/b", "size": 85, "created": "2026-02-12 09:15"},
            {"name": "warehouse_C", "path": "/path/c", "size": 200, "created": "2026-02-13 11:00"},
        ], "error": None}

    def nav_path_start(self, name, is_from_out_start=0):
        return {"success": True, "error": None}

    def nav_patrol_stop(self):
        return {"success": True, "error": None}

    def nav_get_status(self):
        return {"status": 1, "error": None}

    def wait_stop(self, **kwargs):
        return True


# ============================================================
# Flask App
# ============================================================
def create_app(use_mock=False):
    app = Flask(__name__)

    # -- import project modules --
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from tool_definitions import SCOUT_TOOLS, SCOUT_SYSTEM_PROMPT
    from tool_executor import ToolExecutor
    from llm_client import LLMClient
    from interaction_log import InteractionLogger

    if use_mock:
        print("[APP] Mock mode")
        ros = MockROSClient()
    else:
        print("[APP] ROS mode")
        import rospy
        from ros_client import ScoutROSClient
        rospy.init_node("scout_mulmochat", anonymous=True)
        ros = ScoutROSClient()

    executor = ToolExecutor(ros)
    llm = LLMClient(
        tool_executor=executor,
        system_prompt=SCOUT_SYSTEM_PROMPT,
        tools=SCOUT_TOOLS,
    )
    logger = InteractionLogger()

    @app.route("/")
    def index():
        return render_template_string(HTML_TEMPLATE)

    @app.route("/api/chat", methods=["POST"])
    def api_chat():
        data = request.get_json()
        message = data.get("message", "").strip()
        if not message:
            return jsonify({"error": "empty message"})

        logger.log_user_message(message)

        try:
            text, gui_data_list = llm.chat(message)
            logger.log_assistant_response(text, gui_data_list)
            return jsonify({
                "text": text,
                "gui_data_list": gui_data_list,
            })
        except Exception as e:
            traceback.print_exc()
            logger.log_error("chat_error", str(e))
            return jsonify({"error": str(e)})

    @app.route("/api/reset", methods=["POST"])
    def api_reset():
        llm.reset_history()
        return jsonify({"ok": True})

    return app


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--port", type=int, default=7860)
    args = parser.parse_args()

    if not os.environ.get("ANTHROPIC_API_KEY"):
        print("=" * 50)
        print("ERROR: ANTHROPIC_API_KEY 縺梧悴險ｭ螳壹〒縺・)
        print("  export ANTHROPIC_API_KEY=<YOUR_ANTHROPIC_API_KEY>")
        print("=" * 50)
        return

    app = create_app(use_mock=args.mock)
    print(f"\n  竊・繝悶Λ繧ｦ繧ｶ縺ｧ http://localhost:{args.port} 繧帝幕縺・※縺上□縺輔＞\n")
    app.run(host="0.0.0.0", port=args.port, debug=False)


if __name__ == "__main__":
    main()
