#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ros_client.py — SCOUT ROS通信クライアント
auto_step_and_check.py のROS接続パターンを再利用。
Bridge Server から呼ばれる唯一のROS窓口。
"""

import math
import time
import base64
import threading
from dataclasses import dataclass, asdict
from typing import Optional

import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np


def load_srv_class(pkg: str, srv_name: str):
    """auto_step_and_check.py と同じサービスクラスロード"""
    mod = __import__(f"{pkg}.srv", fromlist=["*"])
    if hasattr(mod, srv_name):
        return getattr(mod, srv_name)
    camel = "".join([w.capitalize() for w in srv_name.split("_")])
    if hasattr(mod, camel):
        return getattr(mod, camel)
    raise ImportError(f"Service class not found: {pkg}.srv.{srv_name}")


@dataclass
class OdomSnapshot:
    timestamp: float
    x: float
    y: float
    yaw: float
    v_linear: float
    v_angular: float


@dataclass
class ScoutStatus:
    """SCOUTの現在状態をまとめた構造体"""
    timestamp: float
    odom: Optional[dict]
    camera_image_b64: Optional[str]  # base64 encoded JPEG
    camera_image_shape: Optional[list]  # [H, W]
    is_moving: bool
    nav_status: Optional[int]  # nav_get_status の返り値（あれば）
    error: Optional[str]


class ScoutROSClient:
    """
    ROS通信を一手に引き受けるクライアント。
    rospy.init_node は外側（main）で1回だけ呼ぶ。
    """

    def __init__(
        self,
        image_topic: str = "/scout/camera/image_raw",
        odom_topic: str = "/MotorNode/baselink_odom_relative",
        srv_move: str = "/UtilNode/algo_move",
        srv_action: str = "/UtilNode/algo_action",
        srv_nav_cancel: str = "/NavPathNode/nav_cancel",
        srv_nav_status: str = "/NavPathNode/nav_get_status",
        srv_nav_list_path: str = "/NavPathNode/nav_list_path",
        srv_nav_path_start: str = "/NavPathNode/nav_path_start",
        srv_nav_patrol_stop: str = "/NavPathNode/nav_patrol_stop",
    ):
        self.bridge = CvBridge()
        self._lock = threading.Lock()

        # 最新データ
        self._latest_img: Optional[Image] = None
        self._latest_img_time: float = 0.0
        self._latest_odom: Optional[Odometry] = None

        # Subscribers
        rospy.Subscriber(image_topic, Image, self._cb_img, queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self._cb_odom, queue_size=30)

        # Service clients
        self._move_cli = None
        self._action_cli = None
        self._nav_cancel_cli = None
        self._nav_status_cli = None
        self._nav_list_path_cli = None
        self._nav_path_start_cli = None
        self._nav_patrol_stop_cli = None

        self._init_service(srv_move, "roller_eye", "algo_move", "_move_cli")
        self._init_service(srv_action, "roller_eye", "algo_action", "_action_cli")
        self._init_service(srv_nav_cancel, "roller_eye", "nav_cancel", "_nav_cancel_cli", required=False)
        self._init_service(srv_nav_status, "roller_eye", "nav_get_status", "_nav_status_cli", required=False)
        self._init_service(srv_nav_list_path, "roller_eye", "nav_list_path", "_nav_list_path_cli", required=False)
        self._init_service(srv_nav_path_start, "roller_eye", "nav_path_start", "_nav_path_start_cli", required=False)
        self._init_service(srv_nav_patrol_stop, "roller_eye", "nav_patrol_stop", "_nav_patrol_stop_cli", required=False)

        rospy.loginfo("[ScoutROSClient] initialized")

    def _init_service(self, srv_path: str, pkg: str, srv_name: str, attr: str, required: bool = True):
        try:
            SrvClass = load_srv_class(pkg, srv_name)
            rospy.loginfo("Waiting for service: %s (timeout=5s)", srv_path)
            rospy.wait_for_service(srv_path, timeout=5.0)
            setattr(self, attr, rospy.ServiceProxy(srv_path, SrvClass))
            rospy.loginfo("  -> OK: %s", srv_path)
        except Exception as e:
            if required:
                rospy.logerr("Required service failed: %s -> %s", srv_path, e)
                raise
            else:
                rospy.logwarn("Optional service not available: %s -> %s", srv_path, e)
                setattr(self, attr, None)

    # ── Callbacks ──

    def _cb_img(self, msg: Image):
        with self._lock:
            self._latest_img = msg
            self._latest_img_time = time.time()

    def _cb_odom(self, msg: Odometry):
        with self._lock:
            self._latest_odom = msg

    # ── Odom ──

    def get_odom(self) -> Optional[OdomSnapshot]:
        with self._lock:
            od = self._latest_odom
        if od is None:
            return None

        x = od.pose.pose.position.x
        y = od.pose.pose.position.y
        q = od.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        vx = od.twist.twist.linear.x
        vy = od.twist.twist.linear.y
        v = math.sqrt(vx * vx + vy * vy)
        wz = abs(od.twist.twist.angular.z)

        return OdomSnapshot(
            timestamp=time.time(), x=x, y=y,
            yaw=yaw, v_linear=v, v_angular=wz
        )

    # ── Camera ──

    def get_camera_image_b64(self, max_width: int = 640) -> Optional[str]:
        """最新カメラ画像をbase64 JPEGで返す（チャットUI表示用）"""
        with self._lock:
            msg = self._latest_img
        if msg is None:
            return None

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = img.shape[:2]
        if w > max_width:
            scale = max_width / w
            img = cv2.resize(img, (max_width, int(h * scale)))

        _, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return base64.b64encode(buf).decode("ascii")

    def get_camera_image_shape(self) -> Optional[list]:
        with self._lock:
            msg = self._latest_img
        if msg is None:
            return None
        return [msg.height, msg.width]

    # ── Status（複合） ──

    def get_status(self) -> ScoutStatus:
        odom = self.get_odom()
        img_b64 = self.get_camera_image_b64()
        img_shape = self.get_camera_image_shape()

        is_moving = False
        if odom and (odom.v_linear > 0.02 or odom.v_angular > 0.05):
            is_moving = True

        nav_status = None
        if self._nav_status_cli is not None:
            try:
                res = self._nav_status_cli()
                nav_status = int(getattr(res, "status", -1))
            except Exception:
                pass

        return ScoutStatus(
            timestamp=time.time(),
            odom=asdict(odom) if odom else None,
            camera_image_b64=img_b64,
            camera_image_shape=img_shape,
            is_moving=is_moving,
            nav_status=nav_status,
            error=None,
        )

    # ── Actions ──

    def algo_move(self, y_distance: float, speed: float = 0.15) -> dict:
        """
        algo_move 実行。auto_step_and_check.py の call_algo_move と同等。
        yDistance > 0 → 前進, yDistance < 0 → 後退
        """
        if self._move_cli is None:
            return {"success": False, "ret": -1, "error": "algo_move service not available"}

        # 安全制約: 距離上限
        MAX_DIST = 1.0  # 1m以上は拒否
        if abs(y_distance) > MAX_DIST:
            return {"success": False, "ret": -1, "error": f"distance {y_distance}m exceeds limit {MAX_DIST}m"}

        try:
            res = self._move_cli(
                xDistance=0.0,
                yDistance=float(y_distance),
                speed=float(speed)
            )
            ret = int(getattr(res, "ret", 0))
            return {"success": ret >= 0, "ret": ret, "error": None}
        except rospy.ServiceException as e:
            return {"success": False, "ret": -1, "error": str(e)}

    def algo_roll(self, angle_deg: float, rotated_speed: float = 0.4) -> dict:
        """
        algo_action で旋回。SCOUTでは algo_roll ではなく algo_action を使う。
        angle_deg: 正=左旋回、負=右旋回
        内部で角度→時間に変換: time_ms = abs(angle_deg) / (rotated_speed * 57.3) * 1000
        """
        if self._action_cli is None:
            return {"success": False, "ret": -1, "error": "algo_action service not available"}

        # 安全制約
        MAX_ANGLE = 180.0
        if abs(angle_deg) > MAX_ANGLE:
            return {"success": False, "ret": -1, "error": f"angle {angle_deg}° exceeds limit {MAX_ANGLE}°"}

        # 角度 → 時間(ms) 変換
        # angle(rad) = rotated_speed * time(s)
        # time(ms) = angle(rad) / rotated_speed * 1000
        angle_rad = abs(angle_deg) * 3.14159 / 180.0
        time_ms = int(angle_rad / rotated_speed * 1000)
        time_ms = max(100, min(time_ms, 5000))  # 100ms〜5s に制限

        # 符号: 正=左旋回、負=右旋回
        speed_sign = rotated_speed if angle_deg > 0 else -rotated_speed

        try:
            res = self._action_cli(
                xSpeed=0.0,
                ySpeed=0.0,
                rotatedSpeed=float(speed_sign),
                time=int(time_ms),
            )
            ret = int(getattr(res, "ret", 0))
            return {
                "success": ret >= 0, "ret": ret, "error": None,
                "time_ms": time_ms, "rotated_speed": speed_sign,
            }
        except rospy.ServiceException as e:
            return {"success": False, "ret": -1, "error": str(e)}

    def nav_cancel(self) -> dict:
        """巡回キャンセル"""
        if self._nav_cancel_cli is None:
            return {"success": False, "error": "nav_cancel not available"}
        try:
            self._nav_cancel_cli()
            return {"success": True, "error": None}
        except Exception as e:
            return {"success": False, "error": str(e)}

    # ── Patrol（Phase 2） ──

    def nav_list_path(self) -> dict:
        """登録済みパトロールルート一覧を取得"""
        if self._nav_list_path_cli is None:
            return {"success": False, "routes": [], "error": "nav_list_path not available"}
        try:
            res = self._nav_list_path_cli()
            routes = []
            names = list(getattr(res, "name_list", []))
            paths = list(getattr(res, "path_list", []))
            sizes = list(getattr(res, "size_list", []))
            times = list(getattr(res, "create_time_list", []))
            for i in range(len(names)):
                routes.append({
                    "name": names[i] if i < len(names) else "",
                    "path": paths[i] if i < len(paths) else "",
                    "size": sizes[i] if i < len(sizes) else 0,
                    "created": times[i] if i < len(times) else "",
                })
            return {"success": True, "routes": routes, "error": None}
        except Exception as e:
            return {"success": False, "routes": [], "error": str(e)}

    def nav_path_start(self, name: str, is_from_out_start: int = 0) -> dict:
        """パトロール開始
        name: ルート名
        is_from_out_start: 0=充電台から開始, 1=途中から開始
        """
        if self._nav_path_start_cli is None:
            return {"success": False, "error": "nav_path_start not available"}
        try:
            res = self._nav_path_start_cli(
                isFromOutStart=int(is_from_out_start),
                name=str(name),
            )
            # nav_path_start は返り値なし（空のresponse）
            return {"success": True, "error": None}
        except Exception as e:
            return {"success": False, "error": str(e)}

    def nav_patrol_stop(self) -> dict:
        """パトロール停止"""
        if self._nav_patrol_stop_cli is None:
            return {"success": False, "error": "nav_patrol_stop not available"}
        try:
            self._nav_patrol_stop_cli()
            return {"success": True, "error": None}
        except Exception as e:
            return {"success": False, "error": str(e)}

    def nav_get_status(self) -> dict:
        """ナビ状態取得"""
        if self._nav_status_cli is None:
            return {"status": None, "error": "nav_get_status not available"}
        try:
            res = self._nav_status_cli()
            return {"status": int(getattr(res, "status", -1)), "error": None}
        except Exception as e:
            return {"status": None, "error": str(e)}

    # ── Utility ──

    def wait_stop(self, v_eps=0.02, w_eps=0.05, need_n=5, timeout=3.0) -> bool:
        """auto_step_and_check.py の wait_stop_confirmed と同等"""
        ok = 0
        t0 = time.time()
        while not rospy.is_shutdown():
            odom = self.get_odom()
            if odom is not None:
                if odom.v_linear < v_eps and odom.v_angular < w_eps:
                    ok += 1
                else:
                    ok = 0
                if ok >= need_n:
                    return True
            if time.time() - t0 > timeout:
                return False
            rospy.sleep(0.05)
        return False
