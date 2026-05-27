# Linux系统下 PySide6 图形界面控制程序
# 功能：
# 1. 打开、初始化 CANFD，并使能 1~6 号电机
# 2. 启动 CANFD 连续收发线程
# 3. 启动 MIT 重力补偿线程
# 4. 在线切换 MIT / PV / PVT 模式
# 5. 切换模式前不主动失能，不停止重力补偿线程
# 6. PV 模式按界面输入的 DH 关节角目标运动
# 7. PVT 模式保持当前位置
# 8. 实时显示 1~6 号电机状态和当前 DH 关节角
import sys
import time
import math
import threading
from types import MethodType
from typing import Optional, List
from PySide6.QtGui import QTextCursor
from PySide6.QtCore import QObject, Signal, QTimer, Qt
from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QGroupBox,
    QLabel,
    QPushButton,
    QTextEdit,
    QTableWidget,
    QTableWidgetItem,
    QDoubleSpinBox,
    QCheckBox,
    QMessageBox,
    QSplitter,
)

from USBCANFD import USBCANFD
from DMMotor import DMMotor
from Robot import Robot

MODE_MIT = 1
MODE_PV = 2
MODE_PVT = 4

MODE_NAME = {
    MODE_MIT: "MIT",
    MODE_PV: "PV",
    MODE_PVT: "PVT",
}

MODE_SWITCH_TIMEOUT_S = 3.0
GRAVITY_COMP_PERIOD_S = 0.001

# 与 test_control.py 保持一致：默认补偿 2~5 轴
GRAVITY_TORQUE_SCALE = [0.0, 1.15, 1.1, 1.0, 1.0, 0.0]

DEFAULT_PV_TARGET_DH_Q = [
    3.10,
    -3.13,
    3.10,
    0.05,
    -1.40,
    -0.07,
]
DEFAULT_PV_MOVE_VEL_LIM = 0.3

PVT_HOLD_VEL_LIM = 0
PVT_HOLD_TORQUE_LIM = 0
POWER_ON_WAIT_S = 1.0

class ArmController(QObject):
    log_signal = Signal(str)

    def __init__(self):
        super().__init__()
        self.can: Optional[USBCANFD] = None
        self.robot: Optional[Robot] = None

        self.initialized = False
        self.current_mode: Optional[int] = None

        self.gravity_stop_event = threading.Event()
        self.gravity_thread: Optional[threading.Thread] = None

        self.command_lock = threading.RLock()
        self.data_lock = threading.RLock()

        self.disable_on_exit = True
        self.pv_fallback_to_hold = True

    def log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        self.log_signal.emit(f"[{ts}] {msg}")

    def get_status_snapshot(self) -> Optional[dict]:
        if self.can is None or self.robot is None:
            return None

        try:
            with self.data_lock:
                motors = []
                for m in self.can.motors:
                    motors.append({
                        "id": m.ID,
                        "mode": m.ModeName,
                        "enable": m.Enable,
                        "err": m.ERRCODE,
                        "pos": float(m.Position),
                        "vel": float(m.Velocity),
                        "tau": float(m.Torque),
                        "recv": int(m.recv_num),
                    })

                q_now = self.robot.motor2dh(self.can.motors)
                q_rad = [float(x) for x in q_now]
                q_deg = [float(x * 180.0 / math.pi) for x in q_now]

                return {
                    "initialized": self.initialized,
                    "is_updating": bool(self.can.IsUpdating),
                    "current_mode": self.current_mode,
                    "motor_modes": [m.Mode for m in self.can.motors],
                    "motors": motors,
                    "dh_rad": q_rad,
                    "dh_deg": q_deg,
                    "can_param": self.can.CanParam,
                }
        except Exception as e:
            self.log(f"[WARN] 状态读取失败: {e}")
            return None

    def patch_can_methods(self):
        """
        针对 Linux 版 USBCANFD_fix.py 做运行时补丁：
        1. 让 mode_switch_flag 支持 MIT / PV / PVT 三种模式；
        2. 连续队列发送线程只发送 1~6 号关节电机，不发送第 7 个工具电机；
        3. 适配 Linux 版 _new_canfd_frame(can_id, data, queue=True) 接口；
        4. 将发送 API 从 Windows zlgcan 的 _zcan.TransmitFD 改为 Linux VCI_TransmitFD。
        """
        assert self.can is not None

        def _fill_send_queue_patched(this) -> int:
            if this.mode_switch_flag != 0:
                for i, motor in enumerate(this.motors):
                    if this.mode_switch_flag == MODE_MIT:
                        cmd = motor.set_mit_command
                    elif this.mode_switch_flag == MODE_PV:
                        cmd = motor.set_pv_command
                    elif this.mode_switch_flag == MODE_PVT:
                        cmd = motor.set_pvt_command
                    else:
                        cmd = motor.set_pv_command

                    this.canfd_queue[i] = this._new_canfd_frame(motor.PARAM_SET_ID, cmd, queue=True)
            else:
                for i, motor in enumerate(this.motors):
                    this.canfd_queue[i] = this._new_canfd_frame(motor.ID_OFFSET, motor.Command, queue=True)

            # 只发送 1~6 号电机，不发送第 7 个工具电机
            return this.MOTOR_NUM

        def _canfd_queue_send_thread_6motors(this) -> None:
            """Linux连续发送线程：只发送前6个关节电机。"""
            this.setQueueSend()
            this.clearQueueSend()
            frame = 0
            this.system_update = 0

            FrameArray = type(this.canfd_queue[0]) * this.MOTOR_NUM

            while this.is_updating:
                frame += 1
                this._fill_send_queue()

                frames = FrameArray(*this.canfd_queue[:this.MOTOR_NUM])
                count = this.MOTOR_NUM

                try:
                    ret = this.lib.VCI_TransmitFD(
                        this.kDeviceType,
                        this.device_index_,
                        this.channel_index_,
                        frames,
                        count,
                    )
                except Exception:
                    ret = 0

                with this._lock:
                    this.send_err_num += max(0, count - int(ret))
                    this.send_suc_num += int(ret)
                    this.system_update = frame

                time.sleep(0)

        self.can._fill_send_queue = MethodType(_fill_send_queue_patched, self.can)
        self.can._canfd_queue_send_thread = MethodType(_canfd_queue_send_thread_6motors, self.can)

    def enable_motors_only_before_thread(self) -> bool:
        assert self.can is not None

        self.can.stop_can()
        self.can.clearRecvBuffer()

        for motor in self.can.motors:
            data = self.can.send_wait(1, motor.ID, DMMotor.clear_error_command, 100)
            if not motor.read_motor(data):
                self.log(f"[ERR] 电机 {motor.ID} 清错无有效回复")
                return False

            data = self.can.send_wait(1, motor.ID, DMMotor.enable_command, 100)
            if not motor.read_motor(data):
                self.log(f"[ERR] 电机 {motor.ID} 使能无有效回复")
                return False

            if not motor.Enable:
                self.log(f"[ERR] 电机 {motor.ID} 使能失败，ERR={motor.ERRCODE}")
                return False

            self.log(f"[OK] 电机 {motor.ID} 已使能")

        return True

    def disable_motors_only_at_exit(self):
        if self.can is None:
            return

        self.can.stop_can()

        for motor in self.can.motors:
            try:
                data = self.can.send_wait(1, motor.ID, DMMotor.disable_command, 50)
                motor.read_motor(data)
                self.log(f"[EXIT] 电机 {motor.ID} 已发送失能命令，Enable={motor.Enable}, ERR={motor.ERRCODE}")
            except Exception as e:
                self.log(f"[WARN] 电机 {motor.ID} 失能异常: {e}")

    def set_all_mit_zero_torque(self):
        if self.can is None:
            return

        with self.data_lock:
            for motor in self.can.motors:
                motor.MIT.position_set = 0.0
                motor.MIT.velocity_set = 0.0
                motor.MIT.kp_set = 0.0
                motor.MIT.kd_set = 0.0
                motor.MIT.torque_set = 0.0
                motor.set()

    def set_pv_hold_current_position(self, velocity_lim: float):
        assert self.can is not None

        with self.data_lock:
            for motor in self.can.motors:
                motor.PV.position_set = float(motor.Position)
                motor.PV.velocity_lim = float(velocity_lim)
                motor.set()

        self.log("[PV] 已设置为当前位置保持")

    def set_pv_target_motor_position(self, target_motor_q: List[float], velocity_lim: float) -> bool:
        assert self.can is not None

        if len(target_motor_q) != 6:
            self.log("[ERR] target_motor_q 必须是 6 个数")
            return False

        with self.data_lock:
            for i, motor in enumerate(self.can.motors):
                target = float(target_motor_q[i])
                if not (motor.angle_lim[0] <= target <= motor.angle_lim[1]):
                    self.log(
                        f"[ERR] 电机 {motor.ID} 目标角超限: "
                        f"target={target:.4f}, limit=[{motor.angle_lim[0]:.4f}, {motor.angle_lim[1]:.4f}]"
                    )
                    return False

            for i, motor in enumerate(self.can.motors):
                motor.PV.position_set = float(target_motor_q[i])
                motor.PV.velocity_lim = float(velocity_lim)
                motor.set()

        self.log("[PV] 已写入目标电机角 rad: " + str(["{:.4f}".format(float(x)) for x in target_motor_q]))
        return True

    def set_pv_target_dh_position(self, target_dh_q: List[float], velocity_lim: float) -> bool:
        assert self.can is not None
        assert self.robot is not None

        if len(target_dh_q) != 6:
            self.log("[ERR] target_dh_q 必须是 6 个数")
            return False

        self.log("[PV] 目标 DH 关节角 rad: " + str(["{:.4f}".format(float(x)) for x in target_dh_q]))
        self.log("[PV] 目标 DH 关节角 deg: " + str(["{:.2f}".format(float(x * 180.0 / math.pi)) for x in target_dh_q]))

        with self.data_lock:
            ok, target_motor_q, in_range = self.robot.dh2motor(self.can.motors, target_dh_q)

        if not ok:
            self.log("[ERR] 目标 DH 关节角无法转换为合法电机角")
            self.log(f"in_range={in_range}")
            self.log("target_dh_q rad = " + str(["{:.4f}".format(float(x)) for x in target_dh_q]))
            return False

        return self.set_pv_target_motor_position(target_motor_q, velocity_lim)

    def set_pvt_hold_current_position(self):
        assert self.can is not None

        with self.data_lock:
            for motor in self.can.motors:
                motor.PVT.position_set = float(motor.Position)
                motor.PVT.velocity_lim = int(PVT_HOLD_VEL_LIM)
                motor.PVT.torque_lim = int(PVT_HOLD_TORQUE_LIM)
                motor.set()

        self.log("[PVT] 已设置为当前位置保持")

    def prepare_command_for_target_mode(self, target_mode: int, target_dh_q: Optional[List[float]], pv_velocity_lim: float):
        if target_mode == MODE_MIT:
            self.log("[PREPARE] 目标为 MIT：保持重力补偿线程写入的 MIT 命令")

        elif target_mode == MODE_PV:
            self.log("[PREPARE] 目标为 PV：设置为界面输入的固定 DH 关节角目标")

            if target_dh_q is None:
                self.log("[WARN] 未提供 PV 目标 DH 角，退回当前位置保持")
                self.set_pv_hold_current_position(pv_velocity_lim)
                return

            ok = self.set_pv_target_dh_position(target_dh_q, pv_velocity_lim)

            if not ok:
                if self.pv_fallback_to_hold:
                    self.log("[WARN] 固定 DH 目标设置失败，退回当前位置保持")
                    self.set_pv_hold_current_position(pv_velocity_lim)
                else:
                    raise RuntimeError("PV 固定 DH 目标设置失败")

        elif target_mode == MODE_PVT:
            self.log("[PREPARE] 目标为 PVT：设置为当前位置保持")
            self.set_pvt_hold_current_position()

    def wait_motor_feedback(self, timeout_s: float = 2.0) -> bool:
        assert self.can is not None

        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if all(m.recv_num > 0 for m in self.can.motors):
                return True
            time.sleep(0.005)

        self.log(f"[WARN] 等待反馈超时，recv_num={[m.recv_num for m in self.can.motors]}")
        return False

    def initialize_system(self) -> bool:
        with self.command_lock:
            if self.initialized:
                self.log("[INFO] 系统已经初始化，无需重复初始化")
                return True

            self.can = USBCANFD(device_index=0, channel_index=0, canfd_extended=False, canfd_brs=True)
            self.robot = Robot()
            self.patch_can_methods()

            self.log("[1] 打开 CANFD 设备...")
            if not self.can.open_device():
                self.log("[ERR] 打开 CANFD 设备失败")
                return False

            self.log("[2] 初始化 CANFD 设备...")
            if not self.can.init_device():
                self.log("[ERR] 初始化 CANFD 设备失败")
                self.can.close_device()
                return False

            self.log("[3] 启动 CANFD 通道...")
            if not self.can.start_device():
                self.log("[ERR] 启动 CANFD 通道失败")
                self.can.close_device()
                return False

            time.sleep(POWER_ON_WAIT_S)
            self.can.clearRecvBuffer()

            self.log("[4] 使能前 6 个电机...")
            if not self.enable_motors_only_before_thread():
                self.log("[ERR] 前 6 个电机使能失败")
                self.can.close_device()
                return False

            self.log("[5] 初始化 MIT 零力矩缓存...")
            self.set_all_mit_zero_torque()

            self.log("[6] 启动 CANFD 连续收发线程...")
            self.can.start_can_thread(1)

            self.log("[7] 等待电机反馈...")
            self.wait_motor_feedback(timeout_s=2.0)

            self.log("[8] 启动重力补偿线程...")
            self.gravity_stop_event.clear()
            self.gravity_thread = threading.Thread(
                target=self.gravity_comp_loop,
                name="gravity_comp_loop",
                daemon=True,
            )
            self.gravity_thread.start()

            self.initialized = True
            self.current_mode = None

            self.log("[OK] 初始化完成")
            self.log("[INFO] 后续模式切换不会主动失能，也不会停止重力补偿线程")
            self.log("[INFO] Linux版：本程序只操作 1~6 号电机，不操作第 7 个工具电机")

            return True

    def switch_mode(self, target_mode: int, target_dh_q: Optional[List[float]], pv_velocity_lim: float) -> bool:
        with self.command_lock:
            if not self.initialized or self.can is None:
                self.log("[ERR] 系统未初始化，无法切换模式")
                return False

            if target_mode not in (MODE_MIT, MODE_PV, MODE_PVT):
                self.log(f"[ERR] 不支持的模式: {target_mode}")
                return False

            target_name = MODE_NAME[target_mode]

            if self.current_mode == target_mode:
                self.log(f"[INFO] 当前目标模式已经是 {target_name}，不重复切换")
                return True

            if not self.can.IsUpdating:
                self.log("[ERR] CANFD 连续收发线程未启动，无法在线切换模式")
                return False

            self.prepare_command_for_target_mode(target_mode, target_dh_q, pv_velocity_lim)

            for i in range(self.can.MOTOR_NUM):
                self.can.motor_mode[i] = 0

            self.log(f"[SWITCH] 在线切换到 {target_name} 模式：不失能、不停止重力补偿")

            self.can.mode_switch_flag = target_mode
            deadline = time.time() + MODE_SWITCH_TIMEOUT_S

            while time.time() < deadline:
                modes = [m.Mode for m in self.can.motors]

                if self.can.mode_switch_flag == 0 and all(m == target_mode for m in modes):
                    self.current_mode = target_mode
                    self.log(f"[OK] 已在线切换到 {target_name} 模式")
                    return True

                time.sleep(0.01)

            self.log(
                f"[ERR] 在线切换到 {target_name} 模式超时，"
                f"当前 modes={[m.Mode for m in self.can.motors]}, flag={self.can.mode_switch_flag}"
            )
            self.can.mode_switch_flag = 0
            return False

    def disable_and_stop(self) -> bool:
        with self.command_lock:
            self.log("[SAFE] 准备停止重力补偿、清零 MIT 力矩并失能电机")

            self.gravity_stop_event.set()
            if self.gravity_thread is not None and self.gravity_thread.is_alive():
                self.gravity_thread.join(timeout=1.0)

            try:
                self.set_all_mit_zero_torque()
                time.sleep(0.05)
            except Exception as e:
                self.log(f"[WARN] 清零 MIT 力矩异常: {e}")

            try:
                self.disable_motors_only_at_exit()
            except Exception as e:
                self.log(f"[WARN] 失能电机异常: {e}")

            try:
                if self.can is not None:
                    self.can.stop_can()
            except Exception as e:
                self.log(f"[WARN] stop_can 异常: {e}")

            self.initialized = False
            self.current_mode = None
            self.log("[OK] 已停止并失能")
            return True

    def cleanup(self):
        with self.command_lock:
            self.log("[CLEANUP] 程序退出清理...")

            self.gravity_stop_event.set()
            if self.gravity_thread is not None and self.gravity_thread.is_alive():
                self.gravity_thread.join(timeout=1.0)

            try:
                self.set_all_mit_zero_torque()
                time.sleep(0.05)
            except Exception as e:
                self.log(f"[WARN] 退出时清零 MIT 力矩异常: {e}")

            if self.disable_on_exit:
                try:
                    self.disable_motors_only_at_exit()
                except Exception as e:
                    self.log(f"[WARN] 退出失能异常: {e}")

            try:
                if self.can is not None:
                    self.can.stop_can()
                    self.can.close_device()
            except Exception as e:
                self.log(f"[WARN] 关闭 CANFD 设备异常: {e}")

            self.initialized = False
            self.current_mode = None
            self.log("[END] 清理完成")

    def gravity_comp_loop(self):
        assert self.can is not None
        assert self.robot is not None

        self.log("[GRAVITY] 重力补偿线程已启动，模式切换时不会停止")

        last_print_time = time.time()
        loop_count = 0

        while not self.gravity_stop_event.is_set():
            try:
                with self.data_lock:
                    self.robot.Angle = self.robot.motor2dh(self.can.motors)

                    if not self.robot.set_robot():
                        time.sleep(GRAVITY_COMP_PERIOD_S)
                        continue

                    tau_g_motor = self.robot.Tau_G_Motor

                    for i, motor in enumerate(self.can.motors):
                        motor.MIT.position_set = 0.0
                        motor.MIT.velocity_set = 0.0
                        motor.MIT.kp_set = 0.0
                        motor.MIT.kd_set = 0.0
                        motor.MIT.torque_set = float(tau_g_motor[i] * GRAVITY_TORQUE_SCALE[i])
                        motor.set()

                loop_count += 1

                now = time.time()
                if now - last_print_time >= 1.0:
                    loop_count = 0
                    last_print_time = now

                time.sleep(GRAVITY_COMP_PERIOD_S)

            except Exception as e:
                self.log(f"[ERR] 重力补偿线程异常: {e}")
                time.sleep(0.01)

        self.log("[GRAVITY] 重力补偿线程退出")
class MainWindow(QMainWindow):
    command_done_signal = Signal(str, bool)

    def __init__(self):
        super().__init__()

        self.setWindowTitle("六轴电机模式切换与重力补偿控制界面")
        self.resize(1250, 760)

        self.controller = ArmController()
        self.controller.log_signal.connect(self.append_log)
        self.command_done_signal.connect(self.on_command_done)

        self.command_running = False

        self._build_ui()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_status)
        self.timer.start(200)
    def _build_ui(self):
        central = QWidget()
        main_layout = QVBoxLayout(central)

        splitter = QSplitter(Qt.Vertical)

        top_widget = QWidget()
        top_layout = QHBoxLayout(top_widget)

        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)

        self._build_connection_group(left_layout)
        self._build_mode_group(left_layout)
        self._build_pv_target_group(left_layout)
        self._build_options_group(left_layout)

        left_layout.addStretch(1)

        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)

        self._build_status_group(right_layout)

        top_layout.addWidget(left_panel, 0)
        top_layout.addWidget(right_panel, 1)

        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)

        splitter.addWidget(top_widget)
        splitter.addWidget(self.log_box)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

        main_layout.addWidget(splitter)

        self.setCentralWidget(central)

    def _build_connection_group(self, parent_layout):
        group = QGroupBox("设备与安全")
        layout = QGridLayout(group)

        self.btn_init = QPushButton("初始化并启动控制")
        self.btn_disable = QPushButton("安全失能并停止")
        self.btn_status = QPushButton("刷新状态")
        self.btn_exit = QPushButton("退出程序")

        self.btn_init.clicked.connect(lambda: self.run_async("初始化", self.controller.initialize_system))
        self.btn_disable.clicked.connect(lambda: self.run_async("安全失能", self.controller.disable_and_stop))
        self.btn_status.clicked.connect(self.refresh_status)
        self.btn_exit.clicked.connect(self.close)

        layout.addWidget(self.btn_init, 0, 0, 1, 2)
        layout.addWidget(self.btn_disable, 1, 0, 1, 2)
        layout.addWidget(self.btn_status, 2, 0)
        layout.addWidget(self.btn_exit, 2, 1)

        parent_layout.addWidget(group)
    def _build_mode_group(self, parent_layout):
        group = QGroupBox("模式切换")
        layout = QGridLayout(group)

        self.btn_mit = QPushButton("切换到 MIT + 重力补偿")
        self.btn_pv = QPushButton("切换到 PV 并移动到目标")
        self.btn_pvt = QPushButton("切换到 PVT 当前位置保持")

        self.btn_mit.clicked.connect(lambda: self.switch_mode_async(MODE_MIT))
        self.btn_pv.clicked.connect(lambda: self.switch_mode_async(MODE_PV))
        self.btn_pvt.clicked.connect(lambda: self.switch_mode_async(MODE_PVT))

        layout.addWidget(self.btn_mit, 0, 0, 1, 2)
        layout.addWidget(self.btn_pv, 1, 0, 1, 2)
        layout.addWidget(self.btn_pvt, 2, 0, 1, 2)

        parent_layout.addWidget(group)
    def _build_pv_target_group(self, parent_layout):
        group = QGroupBox("PV 目标 DH 关节角")
        layout = QGridLayout(group)

        layout.addWidget(QLabel("关节"), 0, 0)
        layout.addWidget(QLabel("rad"), 0, 1)
        layout.addWidget(QLabel("deg 显示"), 0, 2)

        self.q_spin = []
        self.q_deg_label = []

        for i, value in enumerate(DEFAULT_PV_TARGET_DH_Q):
            label = QLabel(f"q{i + 1}")
            spin = QDoubleSpinBox()
            spin.setRange(-2.0 * math.pi, 2.0 * math.pi)
            spin.setDecimals(4)
            spin.setSingleStep(0.01)
            spin.setValue(float(value))
            spin.valueChanged.connect(self.update_target_deg_labels)

            deg_label = QLabel("0.00")

            self.q_spin.append(spin)
            self.q_deg_label.append(deg_label)

            layout.addWidget(label, i + 1, 0)
            layout.addWidget(spin, i + 1, 1)
            layout.addWidget(deg_label, i + 1, 2)

        layout.addWidget(QLabel("PV速度限制"), 7, 0)
        self.vel_spin = QDoubleSpinBox()
        self.vel_spin.setRange(0.01, 5.0)
        self.vel_spin.setDecimals(3)
        self.vel_spin.setSingleStep(0.05)
        self.vel_spin.setValue(DEFAULT_PV_MOVE_VEL_LIM)
        layout.addWidget(self.vel_spin, 7, 1)

        self.btn_use_current = QPushButton("读取当前 DH 角作为 PV 目标")
        self.btn_use_current.clicked.connect(self.use_current_dh_as_target)
        layout.addWidget(self.btn_use_current, 8, 0, 1, 3)

        self.update_target_deg_labels()

        parent_layout.addWidget(group)

    def _build_options_group(self, parent_layout):
        group = QGroupBox("选项")
        layout = QVBoxLayout(group)

        self.check_fallback = QCheckBox("PV目标非法时退回当前位置保持")
        self.check_fallback.setChecked(True)
        self.check_fallback.stateChanged.connect(self.update_controller_options)

        self.check_disable_exit = QCheckBox("退出程序时失能前6个电机")
        self.check_disable_exit.setChecked(True)
        self.check_disable_exit.stateChanged.connect(self.update_controller_options)

        layout.addWidget(self.check_fallback)
        layout.addWidget(self.check_disable_exit)

        parent_layout.addWidget(group)

    def _build_status_group(self, parent_layout):
        group = QGroupBox("实时状态")
        layout = QVBoxLayout(group)

        self.mode_label = QLabel("当前目标模式：未知")
        self.updating_label = QLabel("CANFD线程：未启动")
        layout.addWidget(self.mode_label)
        layout.addWidget(self.updating_label)

        self.table = QTableWidget(6, 10)
        self.table.setHorizontalHeaderLabels([
            "ID", "Mode", "Enable", "ERR", "Pos(rad)", "Vel", "Torque", "Recv", "DH(rad)", "DH(deg)"
        ])
        self.table.verticalHeader().setVisible(False)

        for r in range(6):
            for c in range(10):
                item = QTableWidgetItem("")
                item.setTextAlignment(Qt.AlignCenter)
                self.table.setItem(r, c, item)

        layout.addWidget(self.table)

        self.can_param_label = QLabel("CAN参数：")
        layout.addWidget(self.can_param_label)

        parent_layout.addWidget(group)

    def append_log(self, msg: str):
        self.log_box.append(msg)
        self.log_box.moveCursor(QTextCursor.End)   # 使用类属性而非实例属性

    def set_buttons_enabled(self, enabled: bool):
        self.btn_init.setEnabled(enabled)
        self.btn_disable.setEnabled(enabled)
        self.btn_mit.setEnabled(enabled)
        self.btn_pv.setEnabled(enabled)
        self.btn_pvt.setEnabled(enabled)
        self.btn_status.setEnabled(enabled)
        self.btn_exit.setEnabled(enabled)
        self.btn_use_current.setEnabled(enabled)

    def run_async(self, label: str, func):
        if self.command_running:
            self.append_log("[WARN] 上一个命令仍在执行，请稍后再操作")
            return

        self.command_running = True
        self.set_buttons_enabled(False)

        def worker():
            ok = False
            try:
                ok = bool(func())
            except Exception as e:
                self.controller.log(f"[ERR] {label}异常: {e}")
                ok = False
            finally:
                self.command_done_signal.emit(label, ok)

        threading.Thread(target=worker, name=f"cmd_{label}", daemon=True).start()

    def on_command_done(self, label: str, ok: bool):
        self.command_running = False
        self.set_buttons_enabled(True)
        self.append_log(f"[DONE] {label} {'成功' if ok else '失败'}")
        self.refresh_status()

    def update_controller_options(self):
        self.controller.pv_fallback_to_hold = self.check_fallback.isChecked()
        self.controller.disable_on_exit = self.check_disable_exit.isChecked()

    def update_target_deg_labels(self):
        for spin, label in zip(self.q_spin, self.q_deg_label):
            label.setText(f"{spin.value() * 180.0 / math.pi:.2f}")

    def get_target_dh_q_from_ui(self) -> List[float]:
        return [float(spin.value()) for spin in self.q_spin]

    def switch_mode_async(self, mode: int):
        self.update_controller_options()

        if mode == MODE_PV:
            target_dh_q = self.get_target_dh_q_from_ui()
            pv_vel = float(self.vel_spin.value())
        else:
            target_dh_q = None
            pv_vel = float(self.vel_spin.value())

        def command():
            return self.controller.switch_mode(mode, target_dh_q, pv_vel)

        self.run_async(f"切换到 {MODE_NAME[mode]}", command)

    def use_current_dh_as_target(self):
        snapshot = self.controller.get_status_snapshot()
        if snapshot is None:
            QMessageBox.warning(self, "提示", "当前无法读取 DH 关节角，请先初始化并等待反馈。")
            return

        q_rad = snapshot.get("dh_rad", None)
        if q_rad is None or len(q_rad) != 6:
            QMessageBox.warning(self, "提示", "当前 DH 关节角无效。")
            return

        for i in range(6):
            self.q_spin[i].setValue(float(q_rad[i]))

        self.update_target_deg_labels()
        self.append_log("[UI] 已将当前 DH 关节角填入 PV 目标输入框")

    def refresh_status(self):
        snapshot = self.controller.get_status_snapshot()
        if snapshot is None:
            return

        mode = snapshot.get("current_mode", None)
        if mode is None:
            self.mode_label.setText("当前目标模式：未知")
        else:
            self.mode_label.setText(f"当前目标模式：{MODE_NAME.get(mode, mode)}")

        self.updating_label.setText(f"CANFD线程：{'运行中' if snapshot.get('is_updating') else '未运行'}")

        motors = snapshot.get("motors", [])
        dh_rad = snapshot.get("dh_rad", [0.0] * 6)
        dh_deg = snapshot.get("dh_deg", [0.0] * 6)

        for r in range(min(6, len(motors))):
            m = motors[r]
            values = [
                str(m["id"]),
                str(m["mode"]),
                str(m["enable"]),
                str(m["err"]),
                f"{m['pos']:.4f}",
                f"{m['vel']:.4f}",
                f"{m['tau']:.4f}",
                str(m["recv"]),
                f"{dh_rad[r]:.4f}",
                f"{dh_deg[r]:.2f}",
            ]

            for c, text in enumerate(values):
                self.table.item(r, c).setText(text)

        can_param = snapshot.get("can_param", [])
        if can_param:
            self.can_param_label.setText(
                "CAN参数：" + " | ".join(f"{i}:{float(v):.2f}" for i, v in enumerate(can_param))
            )
    def closeEvent(self, event):
        if self.command_running:
            QMessageBox.warning(self, "提示", "当前命令仍在执行，请等待完成后再退出。")
            event.ignore()
            return

        reply = QMessageBox.question(
            self,
            "确认退出",
            "是否退出程序？\n如果勾选了“退出程序时失能前6个电机”，程序会先尝试失能电机。",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )

        if reply != QMessageBox.Yes:
            event.ignore()
            return

        self.timer.stop()

        try:
            self.controller.cleanup()
        except Exception as e:
            self.append_log(f"[WARN] 退出清理异常: {e}")

        event.accept()
def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
if __name__ == "__main__":
    main()