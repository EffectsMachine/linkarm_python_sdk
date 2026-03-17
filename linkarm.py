from __future__ import annotations

import json
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Sequence
import math
import argparse
import shlex

import serial
import serial.tools.list_ports

try:
    from scservo_sdk import PortHandler, scscl, COMM_SUCCESS
except Exception:
    PortHandler = None
    scscl = None
    COMM_SUCCESS = 0


class RobotController:
    """
    机器人控制器

    设计目标：
    1. 单 I/O 线程独占总线，避免读写冲突
    2. 高频关节命令：只保留最新值，可丢弃旧值
    3. 灯光/开关/PWM 等低频但重要命令：进入可靠队列，不能丢
    4. 插值线程不直接访问总线，只生成目标点并交给高频关节槽
    5. 新插值目标到来时，立即打断旧轨迹，转向新目标
    """

    def __init__(
        self,
        config_path: str = "arm_config.json",
        communication_mode: str = "direct_servo",
        serial_port: Optional[str] = None,
        baudrate: Optional[int] = None,
        device_keyword: Optional[str] = None,
        timeout: float = 0.001,
        write_timeout: float = 0.001,
        auto_connect: bool = True,
        direct_read_period: float = 0.02,
        json_read_chunk: int = 256,
        planner_period: float = 0.001,   # 插值线程步进周期
    ):
        self.config_path = str(Path(config_path).expanduser().resolve())
        self.communication_mode = communication_mode.strip().lower()
        if self.communication_mode not in ["direct_servo", "json"]:
            raise ValueError("communication_mode must be 'direct_servo' or 'json'")

        self.timeout = timeout
        self.write_timeout = write_timeout
        self.direct_read_period = direct_read_period
        self.json_read_chunk = json_read_chunk
        self.planner_period = planner_period

        self.config = self._load_config(self.config_path)
        self.linkarm_cfg = self.config.get("linkarm", {})
        self.joint_cfg = self.config.get("joint", {})

        self.device_keyword = device_keyword or self.linkarm_cfg.get("device_info_keyword", "CH343")
        self.serial_port_name = (
            serial_port
            or self.linkarm_cfg.get("default_device_serial_ports")
        )
        self.baudrate = int(baudrate or self.linkarm_cfg.get("serial_baudrate", 500000))
        self.gripper_torque_limit = int(self.linkarm_cfg.get("gripper_torque_limit", 200))
        self.node_id = int(self.linkarm_cfg.get("node_id", 40))

        self.joint_type = self.linkarm_cfg.get("joint_type", "scs")
        self.joint_info = self.joint_cfg.get(self.joint_type, {})
        self.joint_range_rad = float(self.joint_info.get("joint_range_rad", 3.8397))
        self.joint_range_steps = int(self.joint_info.get("joint_range_steps", 1024))
        self.rad_to_step_coefficient = self.joint_range_steps / self.joint_range_rad
        self.id_address = int(self.joint_info.get("id_address", 5))
        self.torque_limit_address = int(self.joint_info.get("torque_limit_address", 16))
        self.torque_lock_address = int(self.joint_info.get("torque_lock_address", 40))

        self.joint_ids = list(self.linkarm_cfg.get("joint_id", [31, 32, 33, 34]))
        self.servo_middle = list(self.linkarm_cfg.get("servo_middle", [512, 512, 512, 512]))
        self.joint_direction = list(self.linkarm_cfg.get("joint_direction", [1, 1, 1, 1]))
        self.joint_limit = list(self.linkarm_cfg.get("joint_limit", [
                                                                        [-1.5708, 1.5708],
                                                                        [-1.5708, 1.5708],
                                                                        [-0.8, 2],
                                                                        [-1.5, 0]
                                                                    ]))
        self.joint_count = len(self.joint_ids)

        self.serial_connection: Optional[serial.Serial] = None
        self.port_handler = None
        self.packet_handler = None

        # =============================
        # IK pre-rendering
        # =============================
        self.l_ab = self.linkarm_cfg.get("link_ab")
        self.l_bc = self.linkarm_cfg.get("link_bc")
        self.l_ac = self.l_ab + self.l_bc
        self.l_cd = math.sqrt(pow(self.linkarm_cfg.get("link_cd_1"), 2)+pow(self.linkarm_cfg.get("link_cd_2"), 2))
        self.l_de = self.linkarm_cfg.get("link_de")
        self.l_ef = self.linkarm_cfg.get("link_ef")
        self.l_bf = math.sqrt(pow(self.linkarm_cfg.get("link_bf_1"), 2)+pow(self.linkarm_cfg.get("link_bf_2"), 2))
        self.l_bf_rad = math.atan2(self.linkarm_cfg.get("link_bf_1"), self.linkarm_cfg.get("link_bf_2"))

        self.ik_status = False
        self.current_xyzg = [self.l_ab + self.linkarm_cfg.get("link_bf_1") + (self.l_ef/2), 
                             0, 
                             self.linkarm_cfg.get("link_bf_2"), 
                             0
                             ]

        # =============================
        # 总线 / 线程控制
        # =============================
        self.stop_event = threading.Event()
        self.wakeup_event = threading.Event()
        self.io_thread: Optional[threading.Thread] = None
        self.bus_lock = threading.RLock()

        # =============================
        # 分槽命令设计
        # =============================
        # 高频槽：关节/底盘控制，只保留最新
        self.pending_joint_motion: Optional[Dict[str, Any]] = None

        # 可靠队列：灯光、开关、PWM 等，不能丢
        self.pending_reliable_commands: Deque[Dict[str, Any]] = deque()

        # json 模式写入：高频槽
        self.pending_json_write: Optional[Dict[str, Any]] = None

        self.pending_lock = threading.Lock()

        # =============================
        # 反馈缓存
        # =============================
        self.latest_feedback_lock = threading.Lock()
        self.latest_feedback: Dict[str, Any] = {
            "timestamp": 0.0,
            "mode": self.communication_mode,
            "joints": {}
        }

        self.last_joint_radians_lock = threading.Lock()
        self.last_joint_radians: List[float] = [0.0] * self.joint_count

        # JSON 接收缓存
        self.json_rx_buffer = bytearray()
        self.json_rx_queue: List[Dict[str, Any]] = []
        self.json_rx_lock = threading.Lock()

        self.last_direct_read_time = 0.0

        # =============================
        # 插值 / 规划线程
        # =============================
        self.planner_wakeup_event = threading.Event()
        self.planner_thread: Optional[threading.Thread] = None
        self.planner_lock = threading.Lock()
        self.planner_target: Optional[Dict[str, Any]] = None
        self.planner_generation: int = 0

        if auto_connect:
            try:
                self.connect()
            except:
                self.disconnect()
                time.sleep(0.1)
                self.connect()

        self.torque_limit(self.joint_ids[0], 1000)
        self.torque_limit(self.joint_ids[1], 1000)
        self.torque_limit(self.joint_ids[2], 1000)
        self.torque_limit(self.joint_ids[3], self.gripper_torque_limit)

    # =====================================================
    # 基础
    # =====================================================
    def _load_config(self, path: str) -> Dict[str, Any]:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)

    def guess_serial_device(self, keyword: str = "CH343") -> Optional[str]:
        keyword = keyword.lower()
        matched = []

        for port in serial.tools.list_ports.comports():
            text = " ".join([
                str(port.device or ""),
                str(port.description or ""),
                str(port.manufacturer or ""),
                str(port.product or ""),
                str(port.hwid or ""),
            ]).lower()
            if keyword in text:
                matched.append(port.device)

        if not matched:
            return None
        if len(matched) == 1:
            return matched[0]
        return matched[-1]

    def resolve_serial_port(self) -> str:
        if self.serial_port_name:
            return self.serial_port_name

        guessed = self.guess_serial_device(self.device_keyword)
        if guessed:
            self.serial_port_name = guessed
            return guessed

        raise RuntimeError(f"No Keyword: {self.device_keyword} device found")

    def rad_to_step(self, joint_index: int, rad: float) -> int:
        return int(round(self.servo_middle[joint_index] + rad * self.rad_to_step_coefficient))

    def clamp_joint_rad(self, joint_index: int, rad: float) -> float:
        if not (0 <= joint_index < self.joint_count):
            raise IndexError(f"joint_index out of range: {joint_index}")

        limit_cfg = self.joint_limit[joint_index] if joint_index < len(self.joint_limit) else None
        if (
            isinstance(limit_cfg, (list, tuple))
            and len(limit_cfg) >= 2
            and limit_cfg[0] is not None
            and limit_cfg[1] is not None
        ):
            low = float(limit_cfg[0])
            high = float(limit_cfg[1])
            if low > high:
                low, high = high, low
            return max(low, min(high, float(rad)))

        return float(rad)


    def clamp_joint_radians(self, joint_radians: Sequence[float]) -> List[float]:
        result: List[float] = []
        for i, rad in enumerate(joint_radians):
            if i >= self.joint_count:
                break
            result.append(self.clamp_joint_rad(i, rad))
        return result


    @property
    def is_connected(self) -> bool:
        if self.communication_mode == "json":
            return self.serial_connection is not None and self.serial_connection.is_open
        return self.port_handler is not None and self.packet_handler is not None

    # =====================================================
    # 连接
    # =====================================================
    def connect(self):
        if self.communication_mode == "direct_servo":
            self._connect_direct_servo()
        else:
            self._connect_json()

        self._start_io_thread()
        self._start_planner_thread()

    def disconnect(self):
        self._stop_planner_thread()
        self._stop_io_thread()

        with self.bus_lock:
            if self.serial_connection is not None:
                try:
                    if self.serial_connection.is_open:
                        self.serial_connection.close()
                except Exception:
                    pass
                self.serial_connection = None

            if self.port_handler is not None:
                try:
                    self.port_handler.closePort()
                except Exception:
                    pass
                self.port_handler = None
                self.packet_handler = None

    def _connect_direct_servo(self):
        if PortHandler is None or scscl is None:
            raise ImportError("没有找到 scservo_sdk")

        port = self.resolve_serial_port()
        self.port_handler = PortHandler(port)
        self.packet_handler = scscl(self.port_handler)

        if not self.port_handler.openPort():
            raise RuntimeError(f"打开串口失败: {port}")
        if not self.port_handler.setBaudRate(self.baudrate):
            raise RuntimeError(f"设置波特率失败: {self.baudrate}")

    def _connect_json(self):
        port = self.resolve_serial_port()
        self.serial_connection = serial.Serial(
            port=port,
            baudrate=self.baudrate,
            timeout=self.timeout,
            write_timeout=self.write_timeout,
        )

        try:
            self.serial_connection.reset_input_buffer()
            self.serial_connection.reset_output_buffer()
        except Exception:
            pass

    # =====================================================
    # I/O 线程
    # =====================================================
    def _start_io_thread(self):
        if self.io_thread is not None and self.io_thread.is_alive():
            return

        self.stop_event.clear()
        self.wakeup_event.clear()
        self.io_thread = threading.Thread(target=self._io_loop, daemon=True)
        self.io_thread.start()

    def _stop_io_thread(self):
        self.stop_event.set()
        self.wakeup_event.set()

        if self.io_thread is not None and self.io_thread.is_alive():
            self.io_thread.join(timeout=0.5)
        self.io_thread = None

    def _io_loop(self):
        """
        优先级：
        1. reliable queue（不能丢的控制）
        2. joint motion slot（高频、可丢）
        3. 读取反馈
        """
        idle_wait = 0.002

        while not self.stop_event.is_set():
            did_work = False

            reliable_cmd, joint_cmd, json_cmd = self._take_pending_writes()

            if reliable_cmd is not None:
                try:
                    if self.communication_mode == "direct_servo":
                        self._execute_direct_command_now(reliable_cmd)
                    else:
                        self._execute_json_write_now(reliable_cmd)
                except Exception as e:
                    print("[io_loop] reliable write failed:", e)
                did_work = True
                        

            elif joint_cmd is not None:
                try:
                    if self.communication_mode == "direct_servo":
                        self._execute_direct_command_now(joint_cmd)
                    else:
                        self._execute_json_write_now(joint_cmd)
                except Exception as e:
                    print("[io_loop] joint write failed:", e)
                did_work = True

            elif json_cmd is not None:
                try:
                    self._execute_json_write_now(json_cmd)
                except Exception as e:
                    print("[io_loop] json write failed:", e)
                did_work = True

            if not did_work:
                try:
                    if self.communication_mode == "direct_servo":
                        now = time.perf_counter()
                        if now - self.last_direct_read_time >= self.direct_read_period:
                            self._read_direct_feedback_once()
                            self.last_direct_read_time = now
                            did_work = True
                    else:
                        if self._read_json_available_once():
                            did_work = True
                except Exception as e:
                    print("[io_loop] read failed:", e)

            if not did_work:
                self.wakeup_event.wait(idle_wait)
                self.wakeup_event.clear()

    def _take_pending_writes(self):
        with self.pending_lock:
            reliable_cmd = None
            if self.pending_reliable_commands:
                reliable_cmd = self.pending_reliable_commands.popleft()

            joint_cmd = self.pending_joint_motion
            self.pending_joint_motion = None

            json_cmd = self.pending_json_write
            self.pending_json_write = None

            return reliable_cmd, joint_cmd, json_cmd

    # =====================================================
    # 对外写接口
    # =====================================================
    def move_joints_rad_sync(
        self,
        joint_radians: Sequence[float],
        speed,
        acc,
        blocking: bool = False,
    ):
        joint_radians = self.clamp_joint_radians(list(joint_radians))
        joint_count = len(joint_radians)

        if isinstance(speed, int):
            speed = [speed] * joint_count
        else:
            speed = list(speed)

        if isinstance(acc, int):
            acc = [acc] * joint_count
        else:
            acc = list(acc)

        if len(speed) < joint_count:
            speed += [0] * (joint_count - len(speed))
        if len(acc) < joint_count:
            acc += [0] * (joint_count - len(acc))

        cmd = {
            "type": "move_joints",
            "joint_radians": joint_radians,
            "speed": speed,
            "acc": acc,
        }

        if blocking:
            if self.communication_mode == "direct_servo":
                return self._execute_direct_command_now(cmd)
            return self._execute_json_write_now({
                "cmd": "move_joints",
                "joints_rad": list(joint_radians),
                "speed": speed,
            })

        if self.communication_mode == "direct_servo":
            with self.pending_lock:
                self.pending_joint_motion = cmd
            self.wakeup_event.set()
            return None

        payload = {
            "cmd": "move_joints",
            "joints_rad": list(joint_radians),
            "speed": speed,
        }
        with self.pending_lock:
            self.pending_json_write = payload
        self.wakeup_event.set()
        return None

    def move_joint_rad(
        self,
        joint_index: int,
        rad: float,
        speed: int = 0,
        acc: int = 0,
        blocking: bool = False,
    ):
        joints = self.get_last_joint_radians()
        if 0 <= joint_index < len(joints):
            joints[joint_index] = self.clamp_joint_rad(joint_index, rad)
            speed_list = [0] * len(joints)
            acc_list = [0] * len(joints)
            speed_list[joint_index] = speed
            acc_list[joint_index] = acc
            return self.move_joints_rad_sync(
                        joints,
                        speed=speed_list,
                        acc=acc_list,
                        blocking=blocking,
                    )
        else:
            return None



    def set_bus_address_1byte_value(self, device_id: int, address: int, value: int):
        '''
        Reliable cmd
        '''
        cmd = {
            "type":"set_1byte_value",
            "id":device_id,
            "address": address,
            "value": value
        }
        self._enqueue_reliable_command(cmd)


    def set_bus_address_2byte_value(self, device_id: int, address: int, value: int):
        '''
        Reliable cmd
        '''
        cmd = {
            "type":"set_2byte_value",
            "id":device_id,
            "address": address,
            "value": value
        }
        self._enqueue_reliable_command(cmd)


    def set_led_async(self, r: int, g: int, b: int):
        """
        Reliable cmd
        """
        r = max(0, min(8, r))
        g = max(0, min(8, g))
        b = max(0, min(8, b))

        r2 = int(r * 3 / 8)   # 0~3
        g3 = int(g * 7 / 8)   # 0~7
        b2 = int(b * 3 / 8)   # 0~3

        rgb232 = (((g3 << 5) | (r2 << 2) | b2) << 1) & 0xFF

        self.set_bus_address_1byte_value(self.node_id, 43, rgb232)
        self.set_bus_address_1byte_value(self.node_id, 44, rgb232)
        self.set_bus_address_1byte_value(self.node_id, 42, 2)


    def set_pwm_async(self, channel: int, pwm: int):
        if channel == 0:
            self.set_bus_address_2byte_value(self.node_id, 34, pwm)
        elif channel == 1:
            self.set_bus_address_2byte_value(self.node_id, 36, pwm)


    def _enqueue_reliable_command(self, cmd: Dict[str, Any]):
        with self.pending_lock:
            self.pending_reliable_commands.append(cmd)
        self.wakeup_event.set()
    

    def send_json(self, payload: Dict[str, Any], blocking: bool = False):
        if self.communication_mode != "json":
            raise RuntimeError("Not in the json mode")

        if blocking:
            return self._execute_json_write_now(payload)

        with self.pending_lock:
            self.pending_json_write = payload
        self.wakeup_event.set()
        return None

    # =====================================================
    # direct_servo 执行
    # =====================================================
    def _execute_direct_command_now(self, cmd: Dict[str, Any]):
        if self.packet_handler is None:
            raise RuntimeError("direct_servo not initialized yet")

        cmd_type = cmd.get("type")
        if not cmd_type:
            raise ValueError("direct command missing 'type'")

        with self.bus_lock:
            if cmd_type == "move_joints":
                return self._direct_write_move_joints(cmd)
            elif cmd_type == "set_1byte_value":
                return self.packet_handler.write1ByteTxRx(cmd["id"], cmd["address"], cmd["value"])
            elif cmd_type == "set_2byte_value":
                return self.packet_handler.write2ByteTxRx(cmd["id"], cmd["address"], cmd["value"])
            elif cmd_type == "set_led":
                return self._direct_write_set_led(cmd)
            elif cmd_type == "set_pwm":
                return self._direct_write_set_pwm(cmd)
            elif cmd_type == "set_switch":
                return self._direct_write_set_switch(cmd)
            else:
                raise ValueError(f"unsupported direct command type: {cmd_type}")

    def _direct_write_move_joints(self, cmd: Dict[str, Any]):
        joint_radians = cmd["joint_radians"]
        speed = cmd["speed"]
        acc = cmd["acc"]

        goal_positions = []
        for i, rad in enumerate(joint_radians):
            if i >= self.joint_count:
                break

            goal_pos = self.rad_to_step(i, rad)
            servo_id = self.joint_ids[i]

            ok = self.packet_handler.SyncWritePos(servo_id, goal_pos, acc[i], speed[i])
            if ok is not True:
                raise RuntimeError(f"SyncWritePos failed, id={servo_id}")

            goal_positions.append(goal_pos)

        comm_result = self.packet_handler.groupSyncWrite.txPacket()
        if comm_result != COMM_SUCCESS:
            err = self.packet_handler.getTxRxResult(comm_result)
            self.packet_handler.groupSyncWrite.clearParam()
            raise RuntimeError(err)

        self.packet_handler.groupSyncWrite.clearParam()

        with self.last_joint_radians_lock:
            for i in range(min(len(joint_radians), self.joint_count)):
                self.last_joint_radians[i] = float(joint_radians[i])

        return goal_positions


    # =====================================================
    # json 写入
    # =====================================================
    def _execute_json_write_now(self, payload: Dict[str, Any]):
        if self.serial_connection is None or not self.serial_connection.is_open:
            raise RuntimeError("json uart not open")

        raw = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8") + b"\n"

        with self.bus_lock:
            return self.serial_connection.write(raw)

    # =====================================================
    # direct_servo 读取
    # =====================================================
    def _read_direct_feedback_once(self):
        if self.packet_handler is None:
            return

        joints = {}
        with self.bus_lock:
            for index in range(len(self.joint_ids)):
                snapshot = self._ft_read_snapshot_one(index)
                joints[index] = snapshot

        with self.latest_feedback_lock:
            self.latest_feedback = {
                "timestamp": time.time(),
                "mode": "direct_servo",
                "joints": joints,
            }

    def _ft_read_snapshot_one(self, joint_index: int) -> Dict[str, Any]:
        if self.packet_handler is None:
            raise RuntimeError("direct_servo not initialized yet")

        result: Dict[str, Any] = {"id": self.joint_ids[joint_index]}
        scs_present_position, scs_present_speed, scs_comm_result, scs_error = self.packet_handler.ReadPosSpeed(
            self.joint_ids[joint_index]
        )
        if scs_comm_result == COMM_SUCCESS:
            result["position"] = scs_present_position
            result["speed"] = scs_present_speed
            result["error"] = scs_error
        else:
            result["comm"] = "failed"
        return result

    def get_latest_feedback(self) -> Dict[str, Any]:
        with self.latest_feedback_lock:
            return dict(self.latest_feedback)

    def get_last_joint_radians(self) -> List[float]:
        with self.last_joint_radians_lock:
            return list(self.last_joint_radians)

    # =====================================================
    # json 读取
    # =====================================================
    def _read_json_available_once(self) -> bool:
        if self.serial_connection is None or not self.serial_connection.is_open:
            return False

        did_read = False

        with self.bus_lock:
            waiting = self.serial_connection.in_waiting
            if waiting <= 0:
                return False
            data = self.serial_connection.read(min(waiting, self.json_read_chunk))

        if not data:
            return False

        self.json_rx_buffer.extend(data)
        did_read = True

        while b"\n" in self.json_rx_buffer:
            line, _, remain = self.json_rx_buffer.partition(b"\n")
            self.json_rx_buffer = bytearray(remain)

            text = line.decode("utf-8", errors="ignore").strip()
            if not text:
                continue

            try:
                obj = json.loads(text)
            except Exception:
                obj = {"raw": text}

            with self.json_rx_lock:
                self.json_rx_queue.append(obj)
                if len(self.json_rx_queue) > 200:
                    self.json_rx_queue = self.json_rx_queue[-200:]

            with self.latest_feedback_lock:
                self.latest_feedback = {
                    "timestamp": time.time(),
                    "mode": "json",
                    "data": obj,
                }

        return did_read

    def read_json_message(self) -> Optional[Dict[str, Any]]:
        with self.json_rx_lock:
            if not self.json_rx_queue:
                return None
            return self.json_rx_queue.pop(0)

    # =====================================================
    # 插值 / 规划线程
    # =====================================================
    def _start_planner_thread(self):
        if self.planner_thread is not None and self.planner_thread.is_alive():
            return

        self.planner_wakeup_event.clear()
        self.planner_thread = threading.Thread(target=self._planner_loop, daemon=True)
        self.planner_thread.start()

    def _stop_planner_thread(self):
        self.planner_wakeup_event.set()
        if self.planner_thread is not None and self.planner_thread.is_alive():
            self.planner_thread.join(timeout=0.5)
        self.planner_thread = None

    def ik_ctrl(self, goal_position: Sequence[float], speed: float = 880):
        """
        用户接口：
            ik_ctrl([x, y, z, g], speed)

        非阻塞：
        - 只写入新的规划目标
        - 规划线程被唤醒
        - 旧轨迹自动失效，转向新目标
        """
        if len(goal_position) != 4:
            raise ValueError("goal_position must be [x, y, z, g]")

        with self.planner_lock:
            self.planner_generation += 1
            self.planner_target = {
                "goal_position": list(goal_position),
                "speed": float(speed),
                "generation": self.planner_generation,
            }

        self.planner_wakeup_event.set()

    def _planner_loop(self):
        """
        规划线程不直接访问串口。
        它只负责：
        1. 取当前目标
        2. 计算当前末端到目标末端的插值点
        3. 每一步调用 ik_generate() -> joints
        4. 再调用 move_joints_rad_sync(..., blocking=False)

        新目标到来时，通过 generation 机制让旧轨迹立刻失效。
        """
        while not self.stop_event.is_set():
            self.planner_wakeup_event.wait()
            self.planner_wakeup_event.clear()

            while not self.stop_event.is_set():
                with self.planner_lock:
                    task = self.planner_target

                if task is None:
                    break

                my_generation = int(task["generation"])
                goal_position = list(task["goal_position"])
                speed = float(task["speed"])

                current_pose = self.current_xyzg

                delta = [goal_position[i] - current_pose[i] for i in range(3)]
                distance = max(abs(v) for v in delta)

                if distance < 1e-6:
                    with self.planner_lock:
                        if self.planner_target and self.planner_target["generation"] == my_generation:
                            self.planner_target = None
                    break

                step_distance = max(speed * self.planner_period, 1e-4)
                steps = max(int(distance / step_distance), 1)

                interrupted = False

                for step_idx in range(1, steps + 1):
                    if self.stop_event.is_set():
                        return

                    with self.planner_lock:
                        latest_generation = self.planner_target["generation"] if self.planner_target else -1

                    if latest_generation != my_generation:
                        interrupted = True
                        break

                    t = step_idx / steps
                    interp_pose = [
                        current_pose[i] + delta[i] * t
                        for i in range(3)
                    ]

                    joints = self.ik_generate(
                        interp_pose[0],
                        interp_pose[1],
                        interp_pose[2]
                    )

                    if joints == False:
                        interrupted = True
                        break

                    self.move_joints_rad_sync(
                        joints,
                        speed=[0] * len(joints),
                        acc=[0] * len(joints),
                        blocking=False,
                    )

                    time.sleep(self.planner_period)

                if interrupted:
                    continue

                with self.planner_lock:
                    if self.planner_target and self.planner_target["generation"] == my_generation:
                        self.planner_target = None
                break
    

    def ik_ctrl_immediate(self, goal_position: Sequence[float], speed: Sequence[int] = [0,0,0,0]):
        self.cancel_ik_ctrl()
        joints = self.ik_generate(
            goal_position[0],
            goal_position[1],
            goal_position[2]
        )
        if joints == False:
            return False
        self.move_joints_rad_sync(
            joints,
            speed,
            acc=[0] * len(joints),
            blocking=True,
        )
        for i in range(len(self.current_xyzg) - 1):
            self.current_xyzg[i] = goal_position[i]
        return True


    def fpv_ctrl_immediate(self, goal_position: Sequence[float], speed: Sequence[int] = [0,0,0,0]):
        '''
        goal_position = [base(rad), reach, z]
        '''
        self.cancel_ik_ctrl()
        input_xyzg_buffer = [math.cos(goal_position[0]) * goal_position[1], 
                             math.sin(goal_position[0]) * goal_position[1], 
                             goal_position[2]]
        joints = self.ik_generate(input_xyzg_buffer[0],
                                  input_xyzg_buffer[1],
                                  input_xyzg_buffer[2])
        if joints == False:
            return False
        self.move_joints_rad_sync(
            joints,
            speed,
            acc=[0] * len(joints),
            blocking=True,
        )
        for i in range(len(self.current_xyzg) - 1):
            self.current_xyzg[i] = input_xyzg_buffer[i]
        return True


    def cancel_ik_ctrl(self):
        with self.planner_lock:
            self.planner_generation += 1
            self.planner_target = None
        self.planner_wakeup_event.set()


    def move_joint_rad_reliable(
        self,
        joint_index: int,
        rad: float,
        speed: int = 0,
        acc: int = 0,
    ):
        joints = self.get_last_joint_radians()
        if not (0 <= joint_index < len(joints)):
            raise IndexError(f"joint_index out of range: {joint_index}")

        joints[joint_index] = self.clamp_joint_rad(joint_index, rad)

        speed_list = [0] * len(joints)
        acc_list = [0] * len(joints)
        speed_list[joint_index] = speed
        acc_list[joint_index] = acc

        cmd = {
            "type": "move_joints",
            "joint_radians": joints,
            "speed": speed_list,
            "acc": acc_list,
        }

        self._enqueue_reliable_command(cmd)


    def gripper_ctrl(self, rad: float, speed: int = 0, acc: int = 0):
        self.torque_limit(self.joint_ids[3], self.gripper_torque_limit)
        return self.move_joint_rad_reliable(3, rad, speed, acc)
    

    def torque_off_all_joint(self):
        for i in range(len(self.joint_ids)):
            self.torque_lock_ctrl(self.joint_ids[i], 0)

    def torque_lock_ctrl(self, id_input, value_input):
        self.set_bus_address_1byte_value(id_input, self.torque_lock_address, value_input)

    def torque_limit(self, id_input, value_input):
        self.set_bus_address_2byte_value(id_input, self.torque_limit_address, value_input)

    def set_arm_middle_as_current_pos(self):
        feedback = self.get_latest_feedback()
        print(f"Raw feedback: {feedback}")
        joints = feedback["joints"]
        result = []
        for i in range(len(joints)):
            result.append(joints[i]["position"])
        self.servo_middle = result
        return result


    def save_joint_middle(self):
        with open(self.config_path, "r", encoding="utf-8") as f:
            config = json.load(f)
        config["linkarm"]["servo_middle"] = self.servo_middle
        with open(self.config_path, "w", encoding="utf-8") as f:
            json.dump(config, f, indent=2, ensure_ascii=False)


    # =====================================================
    # IK / FK
    # =====================================================
    def ik_generate(self, x: float, y: float, z: float) -> Optional[List[float]]:
        """
        linkarm - ik
        """
        try:
            input_xyzg_buffer = [x, y, z]
            rad_0 = math.atan2(-y, x)
            # linkArmPlaneIK(sqrt(pow(x, 2) + pow(y, 2)) - (l_ef / 2), z);
            x = math.sqrt(x*x + y*y) - (self.l_ef/2)
            l_af = math.sqrt(x*x + z*z)
            theta = math.acos(-((self.l_ab*self.l_ab) - (l_af*l_af) - (self.l_bf*self.l_bf))/(2*l_af*self.l_bf))
            lambd = math.atan2(z, x)
            # output: the angle of the shoulder-front joint
            alpha = 1.570796326794897 - theta - lambd

            omega = math.acos(-(self.l_bf*self.l_bf - l_af*l_af - self.l_ab*self.l_ab)/(2*l_af*self.l_ab))
            delta = math.atan2(x, z)
            # output: the radius of the eoat pitch
            mu = delta + omega - 1.570796326794897

            l_ch = math.sin(mu) * self.l_ac
            l_ci = l_ch + z
            l_ah = math.cos(mu) * self.l_ac
            l_ei = x + self.l_ef - l_ah
            l_ce = math.sqrt(l_ei*l_ei + l_ci*l_ci)
            psi = math.acos(-(self.l_cd*self.l_cd - l_ce*l_ce - self.l_de*self.l_de)/(2*l_ce*self.l_de))
            epsilon = math.atan2(l_ci, l_ei)
            # output: the angle of the shoulder-rear joint
            beta = epsilon + psi - 1.570796326794897
        except:
            self.ik_status = False
            return False
        
        if math.isnan(alpha) or math.isnan(beta) or math.isnan(theta):
            self.ik_status = False
            return False
        
        alpha = alpha - self.l_bf_rad
        beta = 1.570796326794897 - beta
        self.ik_status = True
        self.current_xyzg = input_xyzg_buffer

        return [rad_0, alpha, beta]
    

    def _angle_diff(self, a: float, b: float) -> float:
        d = a - b
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        return d
    

    def _ik_planar_angles(self, r: float, z: float):
        """
        输入:
            r: 末端在水平面的半径距离 sqrt(x^2 + y^2)
            z: 末端高度
        输出:
            (alpha, beta)  —— 与 ik_generate() 返回的第2、3个角一致
        """
        x = r - (self.l_ef / 2.0)

        l_af = math.sqrt(x * x + z * z)
        if l_af < 1e-9:
            raise ValueError("l_af too small")

        theta = math.acos(
            -((self.l_ab * self.l_ab) - (l_af * l_af) - (self.l_bf * self.l_bf))
            / (2.0 * l_af * self.l_bf)
        )
        lambd = math.atan2(z, x)

        alpha = math.pi / 2.0 - theta - lambd

        omega = math.acos(
            -(self.l_bf * self.l_bf - l_af * l_af - self.l_ab * self.l_ab)
            / (2.0 * l_af * self.l_ab)
        )
        delta = math.atan2(x, z)
        mu = delta + omega - math.pi / 2.0

        l_ch = math.sin(mu) * self.l_ac
        l_ci = l_ch + z
        l_ah = math.cos(mu) * self.l_ac
        l_ei = x + self.l_ef - l_ah

        l_ce = math.sqrt(l_ei * l_ei + l_ci * l_ci)
        if l_ce < 1e-9:
            raise ValueError("l_ce too small")

        psi = math.acos(
            -(self.l_cd * self.l_cd - l_ce * l_ce - self.l_de * self.l_de)
            / (2.0 * l_ce * self.l_de)
        )
        epsilon = math.atan2(l_ci, l_ei)
        beta = epsilon + psi - math.pi / 2.0

        if math.isnan(alpha) or math.isnan(beta):
            raise ValueError("planar ik result is nan")

        alpha = alpha - self.l_bf_rad
        beta = math.pi / 2.0 - beta

        return alpha, beta


    def fk_generate(
        self,
        rad_0: float,
        alpha_target: float,
        beta_target: float,
        max_iter: int = 50,
        tol: float = 1e-6,
    ):
        """
        输入:
            rad_0, alpha_target, beta_target
            这三个角度应与 ik_generate() 返回值的定义保持一致

        输出:
            [x, y, z]
        """

        # ----------------------------
        # 先做一个粗搜索，找一个比较好的初值
        # ----------------------------
        reach = self.l_ab + self.l_bf
        r_min = max(1e-6, (self.l_ef / 2.0) - reach)
        r_max = (self.l_ef / 2.0) + reach
        z_min = -reach
        z_max = reach

        best_r = None
        best_z = None
        best_err = float("inf")

        # 优先尝试 current_xyzg 作为初值来源
        if hasattr(self, "current_xyzg") and self.current_xyzg and len(self.current_xyzg) >= 3:
            cx, cy, cz = self.current_xyzg[:3]
            guess_r = math.sqrt(cx * cx + cy * cy)
            try:
                a0, b0 = self._ik_planar_angles(guess_r, cz)
                err0 = abs(self._angle_diff(a0, alpha_target)) + abs(self._angle_diff(b0, beta_target))
                best_r, best_z, best_err = guess_r, cz, err0
            except:
                pass

        # 粗网格搜索
        grid_n = 21
        for i in range(grid_n):
            r = r_min + (r_max - r_min) * i / (grid_n - 1)
            for j in range(grid_n):
                z = z_min + (z_max - z_min) * j / (grid_n - 1)
                try:
                    a, b = self._ik_planar_angles(r, z)
                    err = abs(self._angle_diff(a, alpha_target)) + abs(self._angle_diff(b, beta_target))
                    if err < best_err:
                        best_r, best_z, best_err = r, z, err
                except:
                    continue

        if best_r is None or best_z is None:
            return False

        r = best_r
        z = best_z

        # ----------------------------
        # 牛顿迭代精修
        # ----------------------------
        for _ in range(max_iter):
            try:
                a, b = self._ik_planar_angles(r, z)
            except:
                return False

            e1 = self._angle_diff(a, alpha_target)
            e2 = self._angle_diff(b, beta_target)

            if abs(e1) < tol and abs(e2) < tol:
                x = r * math.cos(rad_0)
                y = -r * math.sin(rad_0)
                return [x, y, z]

            h = 1e-4

            try:
                a_r, b_r = self._ik_planar_angles(r + h, z)
                a_z, b_z = self._ik_planar_angles(r, z + h)
            except:
                return False

            # Jacobian
            j11 = self._angle_diff(a_r, a) / h
            j12 = self._angle_diff(a_z, a) / h
            j21 = self._angle_diff(b_r, b) / h
            j22 = self._angle_diff(b_z, b) / h

            det = j11 * j22 - j12 * j21
            if abs(det) < 1e-10:
                return False

            # 解 J * [dr, dz]^T = -[e1, e2]^T
            dr = (-e1 * j22 + e2 * j12) / det
            dz = (-j11 * e2 + j21 * e1) / det

            # 可加一点步长限制，防止发散
            step_limit = 20.0
            dr = max(-step_limit, min(step_limit, dr))
            dz = max(-step_limit, min(step_limit, dz))

            r += dr
            z += dz

            if r < 0:
                r = 0.0

        return False
    
    def get_fk_result(self):
        feedback = self.get_latest_feedback()
        joints_info = feedback["joints"]

        joints_step = []
        for i in range(len(joints_info)):
            joints_step.append(joints_info[i]["position"])

        result = self.fk_generate(
            (joints_step[0] - self.servo_middle[0])/self.rad_to_step_coefficient, 
            (joints_step[1] - self.servo_middle[1])/self.rad_to_step_coefficient, 
            (joints_step[2] - self.servo_middle[2])/self.rad_to_step_coefficient
        )

        print("[FK RESULT]", result)
        return result


    # =====================================================
    # with
    # =====================================================
    def __enter__(self):
        if not self.is_connected:
            self.connect()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.disconnect()








# =============================================================================================================
# LinkArm CLI SDK
# =============================================================================================================
# Command Line Interface for controlling the LinkArm robotic arm.

# This CLI tool allows users, scripts, or AI agents to control the robotic arm through simple commands.

# It supports:
#   Direct joint control
#   Cartesian inverse kinematics
#   Gripper control
#   Torque configuration
#   Interactive shell mode
#   Script / automation integration

# The CLI is designed for:
#   robotics beginners
#   makers
#   developers
#   AI systems controlling robots



def _format_feedback_positions(feedback: Dict[str, Any]) -> List[Optional[int]]:
    """
    Extract joint positions from the latest feedback dictionary.

    Returns:
        A list like [513, 508, 327, 632]. If a joint is missing, None is used.
    """
    joints = feedback.get("joints", {})
    if not isinstance(joints, dict):
        return []

    result: List[Optional[int]] = []
    for i in range(len(joints)):
        joint_info = joints.get(i, {})
        result.append(joint_info.get("position"))
    return result


def _print_status(arm: RobotController):
    """
    Print a compact runtime status summary.
    """
    feedback = arm.get_latest_feedback()
    positions = _format_feedback_positions(feedback)
    print("Connected:", arm.is_connected)
    print("Mode:", feedback.get("mode"))
    print("Timestamp:", feedback.get("timestamp"))
    print("Joint positions:", positions)
    print("Last joint radians:", arm.get_last_joint_radians())
    print("Current XYZG:", arm.current_xyzg)


def _json_print(obj: Dict[str, Any]):
    """
    Print one JSON object in a stable machine-friendly format.
    """
    print(json.dumps(obj, ensure_ascii=False, separators=(",", ":")))


def _sleep_after_nonblocking_command(cmd: Optional[str], seconds: float = 0.05):
    """
    Give the I/O thread a short window to flush queued commands before process exit.

    This is mainly useful for one-shot CLI mode when a command uses
    the async queue instead of direct blocking execution.
    """
    if cmd in {
        "joint",
        "joints",
        "gripper",
        "ik",
        "torque-lock",
        "torque-limit",
        "torque-off-all",
        "set-middle",
        "save-middle",
        "cancel-ik",
        "led",
        "pwm",
    }:
        time.sleep(seconds)


def _split_exec_commands(text: str) -> List[str]:
    """
    Split a semicolon-separated command string.

    Example:
        'joint 3 0 --reliable; sleep 0.2; status'
    """
    parts = [item.strip() for item in text.split(";")]
    return [item for item in parts if item]


def _run_shell_line(arm: RobotController, line: str, json_output: bool = False) -> Dict[str, Any]:
    """
    Execute one shell-style command line and return a structured result.

    Returned object example:
        {"ok": True, "command": "status", "result": {...}}
    """
    try:
        parts = shlex.split(line)
    except ValueError as exc:
        return {
            "ok": False,
            "command": None,
            "error": f"Parse error: {exc}",
        }

    if not parts:
        return {
            "ok": True,
            "command": None,
            "result": None,
        }

    cmd = parts[0].lower()

    try:
        if cmd == "status":
            time.sleep(0.05)
            feedback = arm.get_latest_feedback()
            result = {
                "connected": arm.is_connected,
                "mode": feedback.get("mode"),
                "timestamp": feedback.get("timestamp"),
                "joint_positions": _format_feedback_positions(feedback),
                "last_joint_radians": arm.get_last_joint_radians(),
                "current_xyzg": arm.current_xyzg,
            }
            return {"ok": True, "command": cmd, "result": result}

        elif cmd == "joints":
            if len(parts) < 4:
                raise ValueError("Usage: joints j1 j2 j3 [j4] [speed]")

            numeric_values = [float(x) for x in parts[1:]]
            speed = 0

            if len(numeric_values) in [3, 4]:
                rad_values = numeric_values
            elif len(numeric_values) == 5:
                rad_values = numeric_values[:4]
                speed = int(numeric_values[4])
            else:
                rad_values = numeric_values[:-1]
                speed = int(numeric_values[-1])

            arm.move_joints_rad_sync(rad_values, speed=speed, acc=0, blocking=False)
            return {"ok": True, "command": cmd, "result": "OK"}

        elif cmd == "joint":
            if len(parts) < 3:
                raise ValueError("Usage: joint INDEX RAD [speed] [acc] [--reliable]")

            reliable = "--reliable" in parts
            filtered = [p for p in parts[1:] if p != "--reliable"]

            index = int(filtered[0])
            rad = float(filtered[1])
            speed = int(filtered[2]) if len(filtered) > 2 else 0
            acc = int(filtered[3]) if len(filtered) > 3 else 0

            if reliable:
                arm.move_joint_rad_reliable(index, rad, speed=speed, acc=acc)
            else:
                arm.move_joint_rad(index, rad, speed=speed, acc=acc, blocking=False)

            return {"ok": True, "command": cmd, "result": "OK"}

        elif cmd == "jointr":
            if len(parts) < 3:
                raise ValueError("Usage: jointr INDEX RAD [speed] [acc]")

            index = int(parts[1])
            rad = float(parts[2])
            speed = int(parts[3]) if len(parts) > 3 else 0
            acc = int(parts[4]) if len(parts) > 4 else 0
            arm.move_joint_rad_reliable(index, rad, speed=speed, acc=acc)
            return {"ok": True, "command": cmd, "result": "OK"}

        elif cmd == "gripper":
            if len(parts) < 2:
                raise ValueError("Usage: gripper RAD [speed] [acc]")

            rad = float(parts[1])
            speed = int(parts[2]) if len(parts) > 2 else 0
            acc = int(parts[3]) if len(parts) > 3 else 0
            arm.gripper_ctrl(rad, speed=speed, acc=acc)
            return {"ok": True, "command": cmd, "result": "OK"}

        elif cmd == "ik":
            if len(parts) < 4:
                raise ValueError("Usage: ik X Y Z [speed]")

            x = float(parts[1])
            y = float(parts[2])
            z = float(parts[3])
            speed = float(parts[4]) if len(parts) > 4 else 880.0
            arm.ik_ctrl([x, y, z, 0.0], speed=speed)
            return {"ok": True, "command": cmd, "result": "OK"}

        elif cmd in ["iknow", "ik-now"]:
            if len(parts) < 4:
                raise ValueError("Usage: ik-now X Y Z")

            x = float(parts[1])
            y = float(parts[2])
            z = float(parts[3])
            ok = arm.ik_ctrl_immediate([x, y, z, 0.0])
            return {
                "ok": bool(ok),
                "command": "ik-now",
                "result": "OK" if ok else "IK_FAILED",
            }

        elif cmd == "fpv":
            if len(parts) < 4:
                raise ValueError("Usage: fpv BASE_RAD REACH Z")

            base = float(parts[1])
            reach = float(parts[2])
            z = float(parts[3])
            ok = arm.fpv_ctrl_immediate([base, reach, z])
            return {
                "ok": bool(ok),
                "command": cmd,
                "result": "OK" if ok else "IK_FAILED",
            }

        elif cmd == "torque_lock":
            if len(parts) != 3:
                raise ValueError("Usage: torque_lock SERVO_ID 0|1")

            servo_id = int(parts[1])
            value = int(parts[2])
            arm.torque_lock_ctrl(servo_id, value)
            return {"ok": True, "command": cmd, "result": "OK"}

        elif cmd == "torque_limit":
            if len(parts) != 3:
                raise ValueError("Usage: torque_limit SERVO_ID VALUE")

            servo_id = int(parts[1])
            value = int(parts[2])
            arm.torque_limit(servo_id, value)
            return {"ok": True, "command": cmd, "result": "OK"}

        elif cmd == "torque_off_all":
            arm.torque_off_all_joint()
            return {"ok": True, "command": cmd, "result": "OK"}

        elif cmd == "set_middle":
            result = arm.set_arm_middle_as_current_pos()
            return {"ok": True, "command": cmd, "result": result}

        elif cmd == "save_middle":
            arm.save_joint_middle()
            return {"ok": True, "command": cmd, "result": "OK"}

        elif cmd in ["cancel_ik", "cancel-ik"]:
            arm.cancel_ik_ctrl()
            return {"ok": True, "command": "cancel-ik", "result": "OK"}

        elif cmd == "sleep":
            if len(parts) != 2:
                raise ValueError("Usage: sleep SECONDS")
            seconds = float(parts[1])
            time.sleep(seconds)
            return {"ok": True, "command": cmd, "result": "OK"}

        elif cmd == "fk":
            result = arm.get_fk_result()
            if result is False or result is None:
                return {"ok": False, "command": cmd, "result": "FK failed"}
            else:
                x, y, z = result
                return {"ok": True, "command": cmd, "result": result}

        elif cmd == "led":
            if len(parts) != 4:
                raise ValueError("Usage: led R G B")

            r = int(parts[1])
            g = int(parts[2])
            b = int(parts[3])

            arm.set_led_async(r, g, b)
            return {"ok": True, "command": cmd, "result": "OK"}

        elif cmd == "pwm":
            if len(parts) != 3:
                raise ValueError("Usage: pwm CHANNEL VALUE")

            channel = int(parts[1])
            pwm = int(parts[2])

            if channel not in [0, 1]:
                raise ValueError("PWM channel must be 0 or 1")

            arm.set_pwm_async(channel, pwm)
            return {"ok": True, "command": cmd, "result": "OK"}

        elif cmd in ["exit", "quit"]:
            return {"ok": True, "command": cmd, "result": "EXIT"}

        elif cmd == "help":
            return {"ok": True, "command": cmd, "result": "HELP"}

        raise ValueError(f"Unknown command: {cmd}")

    except Exception as exc:
        return {
            "ok": False,
            "command": cmd,
            "error": str(exc),
        }


def _build_arg_parser() -> argparse.ArgumentParser:
    """
    Build the command line parser.

    This CLI supports both:
    1. One-shot command mode
    2. Interactive shell mode
    """
    parser = argparse.ArgumentParser(
        description="LinkArm CLI for scripting, AI control, and manual interaction."
    )
    parser.add_argument(
        "--config",
        default="arm_config.json",
        help="Path to the robot config JSON file."
    )
    parser.add_argument(
        "--mode",
        default="direct_servo",
        choices=["direct_servo", "json"],
        help="Communication mode."
    )
    parser.add_argument(
        "--port",
        default=None,
        help="Serial port override, e.g. COM42 or /dev/ttyUSB0."
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=None,
        help="Serial baudrate override."
    )
    parser.add_argument(
        "--json-output",
        action="store_true",
        help="Print machine-friendly JSON results."
    )

    subparsers = parser.add_subparsers(dest="command")

    subparsers.add_parser("status", help="Read and print current robot status.")

    p_exec = subparsers.add_parser(
        "exec",
        help='Execute multiple shell commands, e.g. exec "cmd1; cmd2; cmd3"'
    )
    p_exec.add_argument(
        "script",
        type=str,
        help='Semicolon-separated command string.'
    )

    p_joints = subparsers.add_parser("joints", help="Move all joints in radians.")
    p_joints.add_argument("radians", nargs="+", type=float, help="Joint radians.")
    p_joints.add_argument("--speed", type=int, default=0, help="Uniform speed.")
    p_joints.add_argument("--acc", type=int, default=0, help="Uniform acceleration.")
    p_joints.add_argument(
        "--blocking",
        action="store_true",
        help="Execute immediately in blocking mode."
    )

    p_joint = subparsers.add_parser("joint", help="Move one joint in radians.")
    p_joint.add_argument("index", type=int, help="Joint index.")
    p_joint.add_argument("rad", type=float, help="Target joint angle in radians.")
    p_joint.add_argument("--speed", type=int, default=0, help="Joint speed.")
    p_joint.add_argument("--acc", type=int, default=0, help="Joint acceleration.")
    p_joint.add_argument(
        "--blocking",
        action="store_true",
        help="Execute immediately in blocking mode."
    )
    p_joint.add_argument(
        "--reliable",
        action="store_true",
        help="Push this motion into the reliable queue."
    )

    p_gripper = subparsers.add_parser("gripper", help="Control the gripper joint.")
    p_gripper.add_argument("rad", type=float, help="Gripper target in radians.")
    p_gripper.add_argument("--speed", type=int, default=0, help="Gripper speed.")
    p_gripper.add_argument("--acc", type=int, default=0, help="Gripper acceleration.")

    subparsers.add_parser("fk", help="Get current FK xyz.")

    p_led = subparsers.add_parser("led", help="Set onboard LED color.")
    p_led.add_argument("r", type=int, help="Red level, 0~8")
    p_led.add_argument("g", type=int, help="Green level, 0~8")
    p_led.add_argument("b", type=int, help="Blue level, 0~8")

    p_pwm = subparsers.add_parser("pwm", help="Set PWM output.")
    p_pwm.add_argument("channel", type=int, choices=[0, 1], help="PWM channel: 0 or 1")
    p_pwm.add_argument("value", type=int, help="PWM value")

    p_ik = subparsers.add_parser("ik", help="Start interpolated IK movement.")
    p_ik.add_argument("x", type=float)
    p_ik.add_argument("y", type=float)
    p_ik.add_argument("z", type=float)
    p_ik.add_argument("--g", type=float, default=0.0, help="Reserved gripper value.")
    p_ik.add_argument("--speed", type=float, default=880.0, help="IK interpolation speed.")

    p_ik_now = subparsers.add_parser("ik-now", help="Immediate IK move without interpolation.")
    p_ik_now.add_argument("x", type=float)
    p_ik_now.add_argument("y", type=float)
    p_ik_now.add_argument("z", type=float)
    p_ik_now.add_argument("--g", type=float, default=0.0, help="Reserved gripper value.")

    p_fpv = subparsers.add_parser("fpv", help="Immediate FPV control: base(rad), reach, z.")
    p_fpv.add_argument("base", type=float, help="Base angle in radians.")
    p_fpv.add_argument("reach", type=float, help="Planar reach.")
    p_fpv.add_argument("z", type=float, help="Z axis target.")

    p_torque_lock = subparsers.add_parser("torque-lock", help="Set servo torque enable/disable.")
    p_torque_lock.add_argument("servo_id", type=int, help="Servo ID.")
    p_torque_lock.add_argument("value", type=int, choices=[0, 1], help="0=off, 1=on.")

    p_torque_limit = subparsers.add_parser("torque-limit", help="Set servo torque limit.")
    p_torque_limit.add_argument("servo_id", type=int, help="Servo ID.")
    p_torque_limit.add_argument("value", type=int, help="Torque limit value.")

    subparsers.add_parser("torque-off-all", help="Disable torque on all joints.")
    subparsers.add_parser("set-middle", help="Capture current positions as servo_middle in memory.")
    subparsers.add_parser("save-middle", help="Save current servo_middle to config JSON.")
    subparsers.add_parser("cancel-ik", help="Cancel current IK interpolation.")
    subparsers.add_parser("shell", help="Start interactive shell mode.")

    return parser


def _execute_cli_command(arm: RobotController, args: argparse.Namespace):
    """
    Execute one parsed CLI command.
    """
    cmd = args.command
    json_output = bool(getattr(args, "json_output", False))

    def emit_ok(command: str, result: Any = "OK"):
        if json_output:
            _json_print({
                "ok": True,
                "command": command,
                "result": result,
            })
        else:
            if isinstance(result, str):
                print(result)
            else:
                print(result)

    def emit_error(command: Optional[str], error: str):
        if json_output:
            _json_print({
                "ok": False,
                "command": command,
                "error": error,
            })
        else:
            print(f"ERROR: {error}")

    try:
        if cmd == "status":
            time.sleep(0.05)
            feedback = arm.get_latest_feedback()
            result = {
                "connected": arm.is_connected,
                "mode": feedback.get("mode"),
                "timestamp": feedback.get("timestamp"),
                "joint_positions": _format_feedback_positions(feedback),
                "last_joint_radians": arm.get_last_joint_radians(),
                "current_xyzg": arm.current_xyzg,
            }
            emit_ok("status", result)
            return
        
        if cmd == "fk":
            result = arm.get_fk_result()
            if result is False or result is None:
                emit_error("fk", "FK failed")
            else:
                x, y, z = result
                emit_ok("fk", {"x": x, "y": y, "z": z})
            return

        if cmd == "led":
            arm.set_led_async(args.r, args.g, args.b)
            _sleep_after_nonblocking_command("led")
            emit_ok("led")
            return

        if cmd == "pwm":
            arm.set_pwm_async(args.channel, args.value)
            _sleep_after_nonblocking_command("pwm")
            emit_ok("pwm")
            return

        if cmd == "joints":
            arm.move_joints_rad_sync(
                args.radians,
                speed=args.speed,
                acc=args.acc,
                blocking=args.blocking,
            )
            _sleep_after_nonblocking_command("joints")
            emit_ok("joints")
            return

        if cmd == "joint":
            if args.reliable:
                arm.move_joint_rad_reliable(
                    joint_index=args.index,
                    rad=args.rad,
                    speed=args.speed,
                    acc=args.acc,
                )
            else:
                arm.move_joint_rad(
                    joint_index=args.index,
                    rad=args.rad,
                    speed=args.speed,
                    acc=args.acc,
                    blocking=args.blocking,
                )
            _sleep_after_nonblocking_command("joint")
            emit_ok("joint")
            return

        if cmd == "gripper":
            arm.gripper_ctrl(args.rad, speed=args.speed, acc=args.acc)
            _sleep_after_nonblocking_command("gripper")
            emit_ok("gripper")
            return

        if cmd == "ik":
            arm.ik_ctrl([args.x, args.y, args.z, args.g], speed=args.speed)
            _sleep_after_nonblocking_command("ik")
            emit_ok("ik")
            return

        if cmd == "ik-now":
            ok = arm.ik_ctrl_immediate([args.x, args.y, args.z, args.g])
            emit_ok("ik-now", "OK" if ok else "IK_FAILED")
            return

        if cmd == "fpv":
            ok = arm.fpv_ctrl_immediate([args.base, args.reach, args.z])
            emit_ok("fpv", "OK" if ok else "IK_FAILED")
            return

        if cmd == "torque-lock":
            arm.torque_lock_ctrl(args.servo_id, args.value)
            _sleep_after_nonblocking_command("torque-lock")
            emit_ok("torque-lock")
            return

        if cmd == "torque-limit":
            arm.torque_limit(args.servo_id, args.value)
            _sleep_after_nonblocking_command("torque-limit")
            emit_ok("torque-limit")
            return

        if cmd == "torque-off-all":
            arm.torque_off_all_joint()
            _sleep_after_nonblocking_command("torque-off-all")
            emit_ok("torque-off-all")
            return

        if cmd == "set-middle":
            result = arm.set_arm_middle_as_current_pos()
            emit_ok("set-middle", result)
            return

        if cmd == "save-middle":
            arm.save_joint_middle()
            emit_ok("save-middle")
            return

        if cmd == "cancel-ik":
            arm.cancel_ik_ctrl()
            _sleep_after_nonblocking_command("cancel-ik")
            emit_ok("cancel-ik")
            return

        if cmd == "exec":
            commands = _split_exec_commands(args.script)
            results = []

            for line in commands:
                item = _run_shell_line(arm, line, json_output=json_output)
                results.append(item)

                shell_cmd = item.get("command")
                if item.get("ok") and shell_cmd not in ["status", "sleep", "help", "exit", "quit"]:
                    _sleep_after_nonblocking_command(shell_cmd)

                if not item.get("ok"):
                    break

            if json_output:
                _json_print({
                    "ok": all(item.get("ok", False) for item in results),
                    "command": "exec",
                    "results": results,
                })
            else:
                for item in results:
                    if item.get("ok"):
                        result = item.get("result")
                        if item.get("command") == "status":
                            print(result)
                        else:
                            print(result)
                    else:
                        print(f"ERROR: {item.get('error')}")
            return

        if cmd == "shell":
            _run_interactive_shell(arm, json_output=json_output)
            return

        _run_interactive_shell(arm, json_output=json_output)

    except Exception as exc:
        emit_error(cmd, str(exc))


def _run_interactive_shell(arm: RobotController, json_output: bool = False):
    """
    Run an interactive shell for manual control.
    """
    if not json_output:
        print("LinkArm interactive shell")
        print("Type 'help' to see commands. Type 'exit' or 'quit' to leave.")

    help_text = """
Available commands:
  status
  joints j0 j1 j2 j3 [speed]
  joint INDEX RAD [speed] [acc]
  joint INDEX RAD --reliable
  jointr INDEX RAD [speed] [acc]
  gripper RAD [speed] [acc]
  fk
  ik X Y Z [speed]
  iknow X Y Z
  fpv BASE_RAD REACH Z
  led R G B
  pwm CHANNEL VALUE
  torque_lock SERVO_ID 0|1
  torque_limit SERVO_ID VALUE
  torque_off_all
  set_middle
  save_middle
  cancel_ik
  sleep SECONDS
  exit
"""

    while True:
        try:
            line = input("linkarm> ").strip()
        except (EOFError, KeyboardInterrupt):
            if not json_output:
                print()
            break

        if not line:
            continue

        result = _run_shell_line(arm, line, json_output=json_output)

        cmd = result.get("command")
        if result.get("ok") and cmd not in ["status", "sleep", "help", "exit", "quit"]:
            _sleep_after_nonblocking_command(cmd)

        if cmd in ["exit", "quit"]:
            break

        if cmd == "help":
            if json_output:
                _json_print({
                    "ok": True,
                    "command": "help",
                    "result": help_text.strip(),
                })
            else:
                print(help_text)
            continue

        if json_output:
            _json_print(result)
        else:
            if result.get("ok"):
                value = result.get("result")
                if isinstance(value, dict):
                    print(value)
                else:
                    print(value)
            else:
                print(f"ERROR: {result.get('error')}")



if __name__ == "__main__":
    parser = _build_arg_parser()
    args = parser.parse_args()

    with RobotController(
        config_path=args.config,
        communication_mode=args.mode,
        serial_port=args.port,
        baudrate=args.baudrate,
    ) as arm:
        _execute_cli_command(arm, args)