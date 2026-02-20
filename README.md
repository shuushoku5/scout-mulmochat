# SCOUT x MulmoChat

**Conversational Robot Operation Interface -- GUI Chat Protocol for Robot Operation**

[日本語版はこちら / Japanese version](docs/README_ja.md)

A novel robot operation UI where camera images, status panels, and decision forms appear inline within LLM chat conversations.

<p align="center">
  <img src="docs/images/screenshot_status.png" width="400" alt="Status Panel" />
  <img src="docs/images/screenshot_action.png" width="400" alt="Action Result" />
</p>

## Overview

An interactive system for operating and monitoring a patrol robot ([Moorebot Scout](https://www.moorebot.com/)) through natural language conversation.

**Problems with conventional robot operation UIs:**
- Dashboard-based: When anomalies occur, operators must read logs and switch between screens, slowing situation awareness
- LLM text-only: Conversation works, but there's no structured UI -- no image comparison, no clickable choices

**Our approach: GUI Chat Protocol**

Inspired by the GUI Chat Protocol concept from [MulmoChat](https://github.com/receptron/MulmoChat), we extend LLM function calls so that each tool returns **two types of data simultaneously**: text for the LLM's reasoning and rich GUI data for the UI. This allows camera images, status panels, and action buttons to appear inline without breaking the conversation flow.

```
User: "Show me the status"
    |
LLM -> function call: getScoutStatus()
    |
Tool execution -> Returns 2 types of data:
  |-- llm_text: "status=idle, position=x:1.2 y:4.5"  -> LLM's next reasoning input
  |-- gui_data: {type:"status_panel", image:..., odom:...}  -> Rendered as UI panel
    |
Camera image + status table appears in chat + LLM explains in natural language
```

## Key Features

- **Unified conversation and UI** -- Camera images, status panels, and decision forms appear inline in chat
- **LLM decides, ROS executes** -- All control goes through ROS Services. Even if the LLM hallucinates, the robot stays safe
- **LLM-agnostic** -- Swap Claude / GPT / Ollama by changing only `llm_client.py`
- **Lightweight** -- Python + Flask, runs on Python 3.8+
- **Structured logging** -- All interactions recorded in JSON Lines format for research evaluation
- **Easily extensible** -- Add new capabilities just by defining new tools

## Architecture

```
+----------------------------------------------+
|  Operator (Browser)                          |
|  Chat input -> GUI panel rendering           |
+--------------+-------------------------------+
               | HTTP
+--------------v-------------------------------+
|  app_flask.py  (Flask Server)                |
|  HTML/JS renders gui_data inline             |
+--------------+-------------------------------+
               |
+--------------v-------------------------------+
|  llm_client.py  (Claude API / tool_use loop) |
|  User message -> LLM -> function call -> loop|
+--------------+-------------------------------+
               | function call
+--------------v-------------------------------+
|  tool_executor.py  * GUI Chat Protocol core  |
|  Execute tool -> { llm_text + gui_data }     |
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

## Demo

| Input | Action |
|---|---|
| "Show me the status" | Camera image + position + speed + Nav state panel |
| "Move forward 0.2m" | Before/After image comparison with move result |
| "Turn right 30 degrees" | algo_action execution + result panel |
| "Show patrol routes" | Clickable list of registered patrol routes |
| "Start patrol" | Patrol start + progress panel |
| On anomaly | LLM automatically generates and presents decision form |

## Setup

### Requirements

- Moorebot Scout (ROS-connected)
- PC (Ubuntu, ROS Noetic, Python 3.8+)
- Anthropic API key ([get one here](https://console.anthropic.com/))

### Installation

```bash
git clone https://github.com/YOUR_USERNAME/scout-mulmochat.git
cd scout-mulmochat

pip3 install -r requirements.txt
```

### Mock mode (no ROS required)

```bash
export ANTHROPIC_API_KEY=sk-ant-your-key
python3 app_flask.py --mock
# Open http://localhost:7860 in browser
```

### Connect to real robot

```bash
# In a terminal with ROS environment configured
export ROS_MASTER_URI=http://<SCOUT_IP>:11311
export ANTHROPIC_API_KEY=sk-ant-your-key
python3 app_flask.py
```

## File Structure

```
scout-mulmochat/
├── app_flask.py          # Flask chat UI + GUI rendering (main entry)
├── llm_client.py         # LLM communication + tool_use loop
├── tool_definitions.py   # Tool definitions + System Prompt
├── tool_executor.py      # Tool execution (GUI Chat Protocol core)
├── ros_client.py         # SCOUT ROS communication client
├── interaction_log.py    # Structured interaction logging
├── docs/
│   ├── images/           # Screenshots
│   ├── api-reference.md  # SCOUT API reference
│   └── README_ja.md      # Japanese README
├── requirements.txt
├── LICENSE
└── README.md
```

## GUI Chat Protocol Design Pattern

The core of this project is in `tool_executor.py`. Each tool returns this data structure:

```python
def execute_tool(tool_name, tool_input):
    # ... execute tool ...
    return {
        "llm_text": "status=idle, battery=85%",    # Input for LLM reasoning
        "gui_data": {                               # Rendered as UI panel
            "type": "status_panel",                 # Panel type
            "camera_image_b64": "...",              # base64 image
            "odom": {"x": 1.2, "y": 4.5, ...},    # Structured data
        }
    }
```

The `gui_data.type` determines which frontend renderer is used:

| type | Display |
|---|---|
| `status_panel` | Camera image + status table |
| `action_result` | Before/After image comparison + success/failure |
| `decision_form` | Clickable decision form |
| `route_list` | Patrol route list |
| `patrol_started` | Patrol start notification |
| `patrol_status` | Patrol progress + camera image |
| `error` | Error display |

**Adding a new tool (3 steps):**
1. Add tool definition to `tool_definitions.py`
2. Add `_exec_toolname` method to `tool_executor.py`
3. Add `render` function to JavaScript in `app_flask.py`

No changes needed on the LLM side.

## Adapting to Other Robots

This architecture is not SCOUT-specific. Replace `ros_client.py` to adapt to other ROS robots:

```python
# Example: TurtleBot3
class TurtleBot3ROSClient:
    def algo_move(self, y_distance, speed=0.15):
        # Convert to cmd_vel and publish
        ...
    def get_camera_image_b64(self):
        # Subscribe to /camera/rgb/image_raw
        ...
```

Non-ROS robots (HTTP API, serial, etc.) also work -- just implement the same interface.

## Research Context

This system addresses the following research challenges:

- **Mixed-Initiative Robot Operation**: Robot, LLM, and operator collaborate to recover from anomalies
- **Explainable Robot Operation**: Explanation through UI presentation, not just text generation
- **Structured Interaction Logging**: Human interventions are automatically recorded in JSON format for evaluation

For more details on GUI Chat Protocol, see [MulmoChat](https://github.com/receptron/MulmoChat).

## License

MIT License

## Acknowledgments

- [MulmoChat / GUI Chat Protocol](https://github.com/receptron/MulmoChat) -- Inspiration for the UI presentation protocol
- [Moorebot Scout](https://www.moorebot.com/) -- Target robot platform
- [Anthropic Claude API](https://docs.anthropic.com/) -- LLM backend
