"""
teach_record_replay_linux.py
用途：
    1. 在 MIT 模式下启用重力补偿，使 1~6 号关节可以手拖示教；
    2. 记录示教过程中的电机角轨迹和 DH 关节角轨迹；
    3. 切换到 PV 模式后，按记录到的电机角轨迹进行复现；
    4. 本版本适配 Linux 下的 USBCANFD_fix.py / libusbcanfd.so。
建议：
    第一次测试时，请降低 PV_REPLAY_VEL_MAX 和 REPLAY_SPEED_SCALE，并确保急停可用。
"""
from __future__ import annotations
import csv
import math
import threading
import time
from types import MethodType
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional, Sequence
from DMMotor import DMMotor
from Robot import Robot
from USBCANFD import USBCANFD
# =============================================================================
# 基本模式和运行参数
# =============================================================================
MODE_MIT = 1
MODE_PV = 2

MODE_NAME = {
    MODE_MIT: "MIT",
    MODE_PV: "PV",
}

POWER_ON_WAIT_S = 1.0
MODE_SWITCH_TIMEOUT_S = 3.0

# 是否退出时失能前 6 个电机
DISABLE_ON_EXIT = True

# 记录参数
TRAJ_DIR = Path("trajectories")
RECORD_HZ = 50.0
RECORD_PERIOD_S = 1.0 / RECORD_HZ

# MIT 示教时的重力补偿周期
GRAVITY_COMP_PERIOD_S = 0.001

# 重力补偿比例：与之前 test_control.py 保持一致
# 1轴和6轴通常不补偿；2、3轴略放大；4、5轴按 1.0
GRAVITY_TORQUE_SCALE = [0.0, 1.15, 1.1, 1.0, 1.0, 0.0]

# PV 复现参数
REPLAY_SPEED_SCALE = 0.5       # 1.0 原速；0.5 半速；2.0 两倍速。第一次建议 <= 1.0
REPLAY_POINT_STRIDE = 1        # 1 表示不跳点；2 表示隔点播放
PV_RETURN_TO_START_VEL = 0.25  # 复现前回到轨迹起点的速度限制 rad/s
PV_RETURN_TIMEOUT_S = 30.0
PV_POSITION_TOL = 0.035        # 判断到达起点的电机角误差 rad
PV_LIMIT_SOFT_MARGIN = 0.02    # PV目标轻微越界容差 rad；小于该值会夹到限位边界
CLIP_WARNING_ONCE = True       # True：同一电机同一方向只提示一次，避免复现过程刷屏
_clip_warning_keys: set[tuple[int, str]] = set()

PV_REPLAY_VEL_MIN = 0.12       # 复现过程中每个关节最小速度限制
PV_REPLAY_VEL_MAX = 0.5       # 复现过程中每个关节最大速度限制，第一次测试建议 0.3~0.8
PV_VEL_MARGIN = 1.5            # 根据相邻轨迹点估计速度后的裕量

@dataclass
class Trajectory:
    path: Optional[Path]
    t: list[float]
    motor_q: list[list[float]]
    dh_q: list[list[float]]

    @property
    def size(self) -> int:
        return len(self.t)

    @property
    def duration(self) -> float:
        if len(self.t) < 2:
            return 0.0
        return self.t[-1] - self.t[0]

# =============================================================================
# CANFD 初始化、使能、失能
# =============================================================================

def patch_usbcanfd_for_linux_mit_pv_and_six_motors(can: USBCANFD) -> None:
    """
    针对 Linux 版 USBCANFD_fix.py 做运行时补丁：
    1. 在线切换时支持 MIT / PV 两种模式；
    2. 连续队列发送线程只发送 1~6 号关节电机，不周期性发送第 7 个工具电机；
    3. 适配 Linux 版 _new_canfd_frame(can_id, data, queue=True) 接口。

    Windows 版 USBCANFD.py 使用 zlgcan 风格的 _new_canfd_frame(..., flags=0x11)，
    Linux 版 USBCANFD_fix.py 使用 libusbcanfd.so / VCI_* API，因此这里改为 queue=True。
    """

    def _fill_send_queue_patched(self) -> int:
        if self.mode_switch_flag != 0:
            for i, motor in enumerate(self.motors):
                if self.mode_switch_flag == MODE_MIT:
                    cmd = motor.set_mit_command
                elif self.mode_switch_flag == MODE_PV:
                    cmd = motor.set_pv_command
                else:
                    cmd = motor.set_pv_command

                self.canfd_queue[i] = self._new_canfd_frame(motor.PARAM_SET_ID, cmd, queue=True)
        else:
            for i, motor in enumerate(self.motors):
                self.canfd_queue[i] = self._new_canfd_frame(motor.ID_OFFSET, motor.Command, queue=True)

        # 只操作前 6 个关节电机；第 7 个工具电机不放入连续发送队列。
        return self.MOTOR_NUM

    def _canfd_queue_send_thread_six_motors(self) -> None:
        """Linux 连续发送线程：只发送 self.motors[0:6]，不周期性发送工具电机。"""
        self.setQueueSend()
        self.clearQueueSend()

        frame = 0
        self.system_update = 0
        FrameArray = type(self.canfd_queue[0]) * self.MOTOR_NUM

        while self.is_updating:
            frame += 1
            self._fill_send_queue()
            frames = FrameArray(*self.canfd_queue[:self.MOTOR_NUM])

            try:
                ret = self.lib.VCI_TransmitFD(
                    self.kDeviceType,
                    self.device_index_,
                    self.channel_index_,
                    frames,
                    self.MOTOR_NUM,
                )
            except Exception:
                ret = 0

            with self._lock:
                self.send_err_num += max(0, self.MOTOR_NUM - int(ret))
                self.send_suc_num += int(ret)
                self.system_update = frame

            # 让出 GIL，避免发送线程长时间占用 Python 调度。
            time.sleep(0)

    can._fill_send_queue = MethodType(_fill_send_queue_patched, can)
    can._canfd_queue_send_thread = MethodType(_canfd_queue_send_thread_six_motors, can)


def enable_motors_only_before_thread(can: USBCANFD) -> bool:
    """
    只使能前 6 个关节电机，不操作第 7 个工具电机。
    注意：该函数内部会用 send_wait()，只能在 start_can_thread() 之前调用。
    """
    can.stop_can()
    can.clearRecvBuffer()

    for motor in can.motors:
        data = can.send_wait(1, motor.ID, DMMotor.clear_error_command, 100)
        if not motor.read_motor(data):
            print(f"[ERR] 电机 {motor.ID} 清错无有效回复")
            return False

        data = can.send_wait(1, motor.ID, DMMotor.enable_command, 100)
        if not motor.read_motor(data):
            print(f"[ERR] 电机 {motor.ID} 使能无有效回复")
            return False

        if not motor.Enable:
            print(f"[ERR] 电机 {motor.ID} 使能失败，ERR={motor.ERRCODE}")
            return False

        print(f"[OK] 电机 {motor.ID} 已使能")

    return True

def disable_motors_only_at_exit(can: USBCANFD) -> None:
    """
    退出程序时失能前 6 个电机。
    注意：该函数会 stop_can()，只在退出阶段使用。
    """
    can.stop_can()

    for motor in can.motors:
        data = can.send_wait(1, motor.ID, DMMotor.disable_command, 100)
        motor.read_motor(data)
        print(f"[EXIT] 电机 {motor.ID} 已发送失能命令，Enable={motor.Enable}, ERR={motor.ERRCODE}")


def init_canfd_system() -> tuple[USBCANFD, Robot]:
    # Linux 版 USBCANFD_fix.py 默认设备类型为 43，波特率为仲裁域 1 Mbps / 数据域 5 Mbps。
    # canfd_extended=False 表示标准帧；canfd_brs=True 表示 CANFD 开启 BRS 加速。
    can = USBCANFD(device_index=0, channel_index=0, canfd_extended=False, canfd_brs=True)
    robot = Robot()

    patch_usbcanfd_for_linux_mit_pv_and_six_motors(can)

    print("[1] 打开 CANFD 设备...")
    if not can.open_device():
        raise RuntimeError("打开 CANFD 设备失败")

    print("[2] 初始化 CANFD 设备...")
    if not can.init_device():
        raise RuntimeError("初始化 CANFD 设备失败")

    print("[3] 启动 CANFD 通道...")
    if not can.start_device():
        raise RuntimeError("启动 CANFD 通道失败")

    time.sleep(POWER_ON_WAIT_S)
    can.clearRecvBuffer()

    print("[4] 使能前 6 个关节电机...")
    if not enable_motors_only_before_thread(can):
        raise RuntimeError("前 6 个关节电机使能失败")

    print("[5] 初始化 MIT 零力矩命令缓存...")
    set_all_mit_zero_torque(can)

    print("[6] 启动 CANFD 三线程：接收线程、队列发送线程、总线参数更新线程...")
    can.start_can_thread(1)

    print("[7] 等待 1~6 号电机反馈...")
    wait_motor_feedback(can, timeout_s=2.0)

    return can, robot


# =============================================================================
# 模式切换与命令写入
# =============================================================================

def set_all_mit_zero_torque(can: USBCANFD) -> None:
    """清空 MIT 命令缓存，避免残留力矩。"""
    for motor in can.motors:
        motor.MIT.position_set = 0.0
        motor.MIT.velocity_set = 0.0
        motor.MIT.kp_set = 0.0
        motor.MIT.kd_set = 0.0
        motor.MIT.torque_set = 0.0
        motor.set_empty_command_MIT()
        if motor.Mode == MODE_MIT:
            motor.set()


def refresh_pv_command_cache(motor) -> None:
    """
    刷新单个电机的 PV 命令缓存。

    DMMotor.set() 会根据当前模式打包命令；如果当前还在 MIT 模式，
    直接调用 set() 不会更新 PV 缓存。因此从 MIT 切到 PV 之前，
    需要显式刷新 _pv_command，保证模式切换完成后的第一帧 PV 命令安全。
    """
    if hasattr(motor, "_convert_to_candata_PV") and hasattr(motor, "_pv_command"):
        motor._pv_command = bytearray(
            motor._convert_to_candata_PV(motor.PV.position_set, motor.PV.velocity_lim)
        )
    else:
        # 兼容未来如果 DMMotor 增加了公开接口的情况。
        if motor.Mode == MODE_PV:
            motor.set()


def set_pv_hold_current_position(can: USBCANFD, velocity_lim: float = PV_RETURN_TO_START_VEL) -> None:
    """PV 模式下保持当前电机位置。"""
    for motor in can.motors:
        motor.PV.position_set = float(motor.Position)
        motor.PV.velocity_lim = float(velocity_lim)
        refresh_pv_command_cache(motor)


def should_print_clip_warning(motor_id: int, side: str) -> bool:
    """
    控制轻微越界夹紧提示的打印频率。

    CLIP_WARNING_ONCE=True 时，同一电机、同一越界方向只打印一次，
    避免轨迹复现时每个采样点都刷一遍 WARN。
    """
    if not CLIP_WARNING_ONCE:
        return True

    key = (int(motor_id), str(side))
    if key in _clip_warning_keys:
        return False

    _clip_warning_keys.add(key)
    return True


def sanitize_motor_target_to_limit(
    can: USBCANFD,
    target_motor_q: Sequence[float],
    soft_margin: float = PV_LIMIT_SOFT_MARGIN,
) -> Optional[list[float]]:
    """
    检查并修正 PV 目标电机角。

    目的：
        记录轨迹时，电机反馈在零点附近可能出现极小负数。
        例如第 2 轴限位是 [0, 3.7176]，但记录值可能是 -0.0027。
        这种轻微越界不应该直接导致复现失败，而应该夹到限位边界 0。

    规则：
        1. 在限位内：直接使用；
        2. 超出限位但超出量 <= soft_margin：夹到最近限位；
        3. 超出量 > soft_margin：认为轨迹确实非法，返回 None。
    """
    if len(target_motor_q) != 6:
        print("[ERR] target_motor_q 必须是 6 个数")
        return None

    fixed_q: list[float] = []

    for i, motor in enumerate(can.motors):
        q = float(target_motor_q[i])
        lo, hi = float(motor.angle_lim[0]), float(motor.angle_lim[1])

        if lo <= q <= hi:
            fixed_q.append(q)
            continue

        if q < lo and (lo - q) <= soft_margin:
            if should_print_clip_warning(motor.ID, "low"):
                print(
                    f"[WARN] 电机 {motor.ID} 目标角轻微低于下限，已夹到下限: "
                    f"raw={q:.4f}, fixed={lo:.4f}, limit=[{lo:.4f}, {hi:.4f}]"
                )
            fixed_q.append(lo)
            continue

        if q > hi and (q - hi) <= soft_margin:
            if should_print_clip_warning(motor.ID, "high"):
                print(
                    f"[WARN] 电机 {motor.ID} 目标角轻微高于上限，已夹到上限: "
                    f"raw={q:.4f}, fixed={hi:.4f}, limit=[{lo:.4f}, {hi:.4f}]"
                )
            fixed_q.append(hi)
            continue

        print(
            f"[ERR] 电机 {motor.ID} 目标角明显超限: "
            f"target={q:.4f}, limit=[{lo:.4f}, {hi:.4f}], "
            f"soft_margin={soft_margin:.4f}"
        )
        return None

    return fixed_q


def set_pv_target_motor_position(
    can: USBCANFD,
    target_motor_q: Sequence[float],
    velocity_lim: float | Sequence[float] = PV_RETURN_TO_START_VEL,
) -> bool:
    """
    PV 模式下设置目标电机角。
    velocity_lim 可以是一个标量，也可以是 6 个关节分别对应的速度限制。
    """
    fixed_target_q = sanitize_motor_target_to_limit(can, target_motor_q)
    if fixed_target_q is None:
        return False

    if isinstance(velocity_lim, (list, tuple)):
        vel_list = [float(v) for v in velocity_lim]
        if len(vel_list) != 6:
            print("[ERR] velocity_lim 如果是列表，必须是 6 个数")
            return False
    else:
        vel_list = [float(velocity_lim)] * 6

    for i, motor in enumerate(can.motors):
        motor.PV.position_set = float(fixed_target_q[i])
        motor.PV.velocity_lim = float(vel_list[i])
        refresh_pv_command_cache(motor)

    return True


def wait_motor_feedback(can: USBCANFD, timeout_s: float = 2.0) -> bool:
    deadline = time.time() + timeout_s

    while time.time() < deadline:
        if all(m.recv_num > 0 for m in can.motors):
            return True
        time.sleep(0.005)

    print(f"[WARN] 等待反馈超时，recv_num={[m.recv_num for m in can.motors]}")
    return False


def online_switch_mode_keep_enable(can: USBCANFD, target_mode: int) -> bool:
    """
    在线切换 MIT / PV：
    - 不主动失能；
    - 不停止 CANFD 三线程；
    - 通过 mode_switch_flag 让队列发送线程持续发送模式切换命令。
    """
    if target_mode not in (MODE_MIT, MODE_PV):
        print(f"[ERR] 不支持的模式: {target_mode}")
        return False

    if not can.IsUpdating:
        print("[ERR] CANFD 连续收发线程未启动，无法在线切换模式")
        return False

    target_name = MODE_NAME[target_mode]

    if target_mode == MODE_MIT:
        print("[PREPARE] 切换 MIT：先清零 MIT 力矩缓存，随后由重力补偿线程写入 MIT 力矩")
        set_all_mit_zero_torque(can)

    elif target_mode == MODE_PV:
        print("[PREPARE] 切换 PV：先准备当前位置保持命令")
        set_pv_hold_current_position(can)

    # 重置模式记录，避免旧状态误判
    for i in range(can.MOTOR_NUM):
        can.motor_mode[i] = 0

    print(f"[SWITCH] 在线切换到 {target_name} 模式，不失能、不停止三线程")
    can.mode_switch_flag = target_mode

    deadline = time.time() + MODE_SWITCH_TIMEOUT_S

    while time.time() < deadline:
        modes = [m.Mode for m in can.motors]
        if can.mode_switch_flag == 0 and all(m == target_mode for m in modes):
            print(f"[OK] 已切换到 {target_name} 模式")
            return True
        time.sleep(0.01)

    print(
        f"[ERR] 切换到 {target_name} 模式超时，"
        f"modes={[m.Mode for m in can.motors]}, flag={can.mode_switch_flag}"
    )
    can.mode_switch_flag = 0
    return False

# =============================================================================
# MIT 重力补偿示教线程
# =============================================================================

def gravity_comp_loop(can: USBCANFD, robot: Robot, stop_event: threading.Event) -> None:
    """
    重力补偿线程：
    - 在 MIT 模式下写入 MIT 力矩命令；
    - 在 PV 模式下不覆盖 PV 轨迹命令；
    - 线程一直运行，方便 MIT/PV 在线切换。
    """
    print("[GRAVITY] 重力补偿线程已启动")

    while not stop_event.is_set():
        try:
            robot.Angle = robot.motor2dh(can.motors)

            if not robot.set_robot():
                time.sleep(GRAVITY_COMP_PERIOD_S)
                continue

            tau_g_motor = robot.Tau_G_Motor

            # 只有在 MIT 模式下才打包 MIT 命令，避免 PV 复现时被覆盖
            if all(m.Mode == MODE_MIT for m in can.motors):
                for i, motor in enumerate(can.motors):
                    motor.MIT.position_set = 0.0
                    motor.MIT.velocity_set = 0.0
                    motor.MIT.kp_set = 0.0
                    motor.MIT.kd_set = 0.0
                    motor.MIT.torque_set = float(tau_g_motor[i] * GRAVITY_TORQUE_SCALE[i])
                    motor.set()

            time.sleep(GRAVITY_COMP_PERIOD_S)

        except Exception as exc:
            print(f"[ERR] 重力补偿线程异常: {exc}")
            time.sleep(0.01)

    print("[GRAVITY] 重力补偿线程退出")

# =============================================================================
# 轨迹记录与保存
# =============================================================================
def record_worker(
    can: USBCANFD,
    robot: Robot,
    rows: list[dict[str, float]],
    stop_event: threading.Event,
) -> None:
    print("[RECORD] 开始记录轨迹")
    t0 = time.perf_counter()
    next_t = t0
    sample_idx = 0

    while not stop_event.is_set():
        now = time.perf_counter()

        if now < next_t:
            time.sleep(min(0.001, next_t - now))
            continue

        t_rel = now - t0
        motor_q = [float(m.Position) for m in can.motors]
        dh_q = [float(x) for x in robot.motor2dh(can.motors)]

        row: dict[str, float] = {
            "sample": float(sample_idx),
            "t": float(t_rel),
        }

        for i in range(6):
            row[f"motor_{i + 1}"] = motor_q[i]
        for i in range(6):
            row[f"dh_{i + 1}"] = dh_q[i]

        rows.append(row)

        sample_idx += 1
        next_t += RECORD_PERIOD_S

    print(f"[RECORD] 停止记录，共 {len(rows)} 个采样点")


def save_trajectory(rows: list[dict[str, float]], path: Optional[Path] = None) -> Optional[Path]:
    if not rows:
        print("[WARN] 没有轨迹点，不保存")
        return None

    TRAJ_DIR.mkdir(parents=True, exist_ok=True)

    if path is None:
        path = TRAJ_DIR / f"teach_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

    fieldnames = ["sample", "t"]
    fieldnames += [f"motor_{i + 1}" for i in range(6)]
    fieldnames += [f"dh_{i + 1}" for i in range(6)]

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    duration = rows[-1]["t"] - rows[0]["t"] if len(rows) >= 2 else 0.0
    print(f"[SAVE] 已保存轨迹: {path}")
    print(f"[SAVE] 点数={len(rows)}, 时长={duration:.3f}s, 采样频率约={len(rows) / max(duration, 1e-6):.1f}Hz")
    return path


def record_trajectory_until_enter(can: USBCANFD, robot: Robot) -> Optional[Path]:
    """
    启动记录线程，按回车停止并保存 CSV。
    """
    if not all(m.Mode == MODE_MIT for m in can.motors):
        print("[WARN] 当前不是 MIT 模式，建议先切换到 MIT 示教模式")
        ok = online_switch_mode_keep_enable(can, MODE_MIT)
        if not ok:
            return None

    rows: list[dict[str, float]] = []
    stop_event = threading.Event()

    th = threading.Thread(
        target=record_worker,
        args=(can, robot, rows, stop_event),
        name="record_worker",
        daemon=True,
    )
    th.start()

    input("[RECORD] 现在可以手拖机械臂示教；按回车停止记录并保存轨迹...\n")
    stop_event.set()
    th.join(timeout=2.0)

    return save_trajectory(rows)


def load_trajectory(path: str | Path) -> Trajectory:
    path = Path(path)

    t: list[float] = []
    motor_q: list[list[float]] = []
    dh_q: list[list[float]] = []

    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)

        for row in reader:
            t.append(float(row["t"]))
            motor_q.append([float(row[f"motor_{i + 1}"]) for i in range(6)])

            # 兼容只保存电机角的文件
            if all(f"dh_{i + 1}" in row for i in range(6)):
                dh_q.append([float(row[f"dh_{i + 1}"]) for i in range(6)])
            else:
                dh_q.append([0.0] * 6)

    if len(t) < 2:
        raise ValueError("轨迹点数不足，无法复现")

    traj = Trajectory(path=path, t=t, motor_q=motor_q, dh_q=dh_q)
    print(f"[LOAD] 已加载轨迹: {path}")
    print(f"[LOAD] 点数={traj.size}, 时长={traj.duration:.3f}s")
    return traj


def get_latest_trajectory_file() -> Optional[Path]:
    if not TRAJ_DIR.exists():
        return None
    files = sorted(TRAJ_DIR.glob("teach_*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)
    return files[0] if files else None


# =============================================================================
# PV 复现
# =============================================================================

def current_motor_q(can: USBCANFD) -> list[float]:
    return [float(m.Position) for m in can.motors]


def max_abs_error(a: Sequence[float], b: Sequence[float]) -> float:
    return max(abs(float(x) - float(y)) for x, y in zip(a, b))


def wait_until_motor_close(
    can: USBCANFD,
    target_motor_q: Sequence[float],
    tol: float = PV_POSITION_TOL,
    timeout_s: float = PV_RETURN_TIMEOUT_S,
) -> bool:
    deadline = time.time() + timeout_s

    while time.time() < deadline:
        err = max_abs_error(current_motor_q(can), target_motor_q)
        if err <= tol:
            print(f"[PV] 已接近目标，max_err={err:.4f} rad")
            return True
        time.sleep(0.02)

    err = max_abs_error(current_motor_q(can), target_motor_q)
    print(f"[WARN] 等待到位超时，max_err={err:.4f} rad")
    return False


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def compute_segment_velocity_limits(
    can: USBCANFD,
    q_prev: Sequence[float],
    q_next: Sequence[float],
    dt: float,
) -> list[float]:
    dt = max(float(dt), 0.005)
    vel_list: list[float] = []

    for i, motor in enumerate(can.motors):
        required = abs(float(q_next[i]) - float(q_prev[i])) / dt * PV_VEL_MARGIN
        motor_limit = min(PV_REPLAY_VEL_MAX, float(motor.max_velocity) * 0.5)
        vel_list.append(clamp(required, PV_REPLAY_VEL_MIN, motor_limit))

    return vel_list


def print_trajectory_limit_summary(can: USBCANFD, points: Sequence[Sequence[float]]) -> None:
    """
    复现前统计轨迹中有多少点会触及/超过软件限位。

    这样可以知道是单个点偶然越界，还是整段轨迹都在限位边界附近。
    """
    if not points:
        return

    any_clip = False

    for i, motor in enumerate(can.motors):
        lo, hi = float(motor.angle_lim[0]), float(motor.angle_lim[1])
        low_values = [float(p[i]) for p in points if float(p[i]) < lo]
        high_values = [float(p[i]) for p in points if float(p[i]) > hi]

        if low_values:
            any_clip = True
            min_raw = min(low_values)
            max_exceed = lo - min_raw
            level = "WARN" if max_exceed <= PV_LIMIT_SOFT_MARGIN else "ERR"
            print(
                f"[TRAJ {level}] 电机 {motor.ID} 有 {len(low_values)} 个轨迹点低于下限，"
                f"min_raw={min_raw:.4f}, limit_low={lo:.4f}, max_exceed={max_exceed:.4f} rad"
            )

        if high_values:
            any_clip = True
            max_raw = max(high_values)
            max_exceed = max_raw - hi
            level = "WARN" if max_exceed <= PV_LIMIT_SOFT_MARGIN else "ERR"
            print(
                f"[TRAJ {level}] 电机 {motor.ID} 有 {len(high_values)} 个轨迹点高于上限，"
                f"max_raw={max_raw:.4f}, limit_high={hi:.4f}, max_exceed={max_exceed:.4f} rad"
            )

    if any_clip:
        print(
            "[TRAJ INFO] 轻微越界点会被夹到软件限位边界；"
            "如果越界点很多，说明示教轨迹贴近或超过了当前软件限位。"
        )


def replay_trajectory_in_pv(can: USBCANFD, traj: Trajectory) -> bool:
    """
    在 PV 模式下复现记录轨迹。
    复现使用电机侧角度 motor_1~motor_6，不使用 DH 角作为目标。
    """
    if traj.size < 2:
        print("[ERR] 轨迹点数不足")
        return False

    points = traj.motor_q[::REPLAY_POINT_STRIDE]
    times = traj.t[::REPLAY_POINT_STRIDE]

    if len(points) < 2:
        print("[ERR] 跳点后轨迹点数不足")
        return False

    print_trajectory_limit_summary(can, points)

    print("[REPLAY] 准备切换到 PV 模式")
    if not online_switch_mode_keep_enable(can, MODE_PV):
        return False

    first_q = points[0]
    print("[REPLAY] 先移动到记录轨迹起点")
    if not set_pv_target_motor_position(can, first_q, PV_RETURN_TO_START_VEL):
        return False

    arrived = wait_until_motor_close(can, first_q, tol=PV_POSITION_TOL, timeout_s=PV_RETURN_TIMEOUT_S)

    if not arrived:
        print("[ERR] 机械臂未能在规定时间内回到轨迹起点，取消本次复现")
        return False

    input("[REPLAY] 已回到轨迹起点。确认机械臂到位且周围安全后，按回车开始复现...\n")

    print(f"[REPLAY] 开始 PV 复现，点数={len(points)}, 原始轨迹时长={times[-1] - times[0]:.3f}s")
    t_replay_start = time.perf_counter()
    t0 = times[0]

    for idx, q in enumerate(points):
        if idx == 0:
            vel_lim = [PV_RETURN_TO_START_VEL] * 6
        else:
            raw_dt = max(times[idx] - times[idx - 1], 0.001)
            dt = raw_dt / max(REPLAY_SPEED_SCALE, 1e-6)
            vel_lim = compute_segment_velocity_limits(can, points[idx - 1], q, dt)

        if not set_pv_target_motor_position(can, q, vel_lim):
            print(f"[ERR] 第 {idx} 个轨迹点超限或写入失败，中止复现")
            return False

        # 按记录时间节拍播放
        if idx + 1 < len(points):
            next_elapsed = (times[idx + 1] - t0) / max(REPLAY_SPEED_SCALE, 1e-6)
            sleep_until = t_replay_start + next_elapsed
            while True:
                remain = sleep_until - time.perf_counter()
                if remain <= 0:
                    break
                time.sleep(min(0.002, remain))

        if idx % max(1, int(1.0 / (RECORD_PERIOD_S * REPLAY_POINT_STRIDE))) == 0:
            print(f"[REPLAY] {idx + 1}/{len(points)}")

    # 复现结束后保持最后一个轨迹点
    set_pv_target_motor_position(can, points[-1], PV_RETURN_TO_START_VEL)
    print("[REPLAY] 轨迹复现完成，PV 模式保持最后一个点")
    return True


# =============================================================================
# 状态显示和菜单
# =============================================================================

def print_motor_status(can: USBCANFD) -> None:
    print("-" * 100)
    print(
        f"{'ID':>3} | {'Mode':>5} | {'Enable':>6} | {'ERR':>8} | "
        f"{'Pos':>10} | {'Vel':>10} | {'Tau':>10} | {'Recv':>6}"
    )
    print("-" * 100)

    for m in can.motors:
        print(
            f"{m.ID:>3} | {m.ModeName:>5} | {str(m.Enable):>6} | {m.ERRCODE:>8} | "
            f"{m.Position:>10.4f} | {m.Velocity:>10.4f} | {m.Torque:>10.4f} | {m.recv_num:>6}"
        )


def print_current_dh_angle(can: USBCANFD, robot: Robot) -> None:
    q_now = robot.motor2dh(can.motors)
    print("[NOW DH rad] =", ["{:.4f}".format(float(x)) for x in q_now])
    print("[NOW DH deg] =", ["{:.2f}".format(float(x * 180.0 / math.pi)) for x in q_now])


def print_menu(current_mode: Optional[int], last_traj_path: Optional[Path]) -> None:
    print("\n" + "=" * 72)
    print(f"当前目标模式：{MODE_NAME.get(current_mode, '未知')}")
    print(f"最近轨迹文件：{last_traj_path if last_traj_path else '无'}")
    print("-" * 72)
    print("1 - 切换到 MIT 示教模式：重力补偿，可手拖")
    print("r - 开始记录示教轨迹：按回车停止并保存 CSV")
    print("p - 切换到 PV 模式并复现最近一次轨迹")
    print("l - 手动加载一个 CSV 轨迹文件")
    print("s - 查看电机状态和当前 DH 关节角")
    print("0 - 退出程序")
    print("=" * 72)


def main() -> None:
    can: Optional[USBCANFD] = None
    robot: Optional[Robot] = None

    gravity_stop_event = threading.Event()
    gravity_thread: Optional[threading.Thread] = None

    current_mode: Optional[int] = None
    last_traj_path: Optional[Path] = get_latest_trajectory_file()
    loaded_traj: Optional[Trajectory] = None

    try:
        can, robot = init_canfd_system()

        print("[8] 启动重力补偿线程...")
        gravity_thread = threading.Thread(
            target=gravity_comp_loop,
            args=(can, robot, gravity_stop_event),
            name="gravity_comp_loop",
            daemon=True,
        )
        gravity_thread.start()

        print("[9] 自动切换到 MIT 示教模式...")
        if online_switch_mode_keep_enable(can, MODE_MIT):
            current_mode = MODE_MIT

        print("[OK] 初始化完成。现在可以选择 r 开始拖动示教记录。")
        print("[INFO] 本程序只操作 1~6 号关节电机，不操作第 7 个工具电机。")

        while True:
            print_menu(current_mode, last_traj_path)
            choice = input("请输入: ").strip().lower()

            if choice == "0":
                print("[EXIT] 准备退出")
                break

            if choice == "1":
                if online_switch_mode_keep_enable(can, MODE_MIT):
                    current_mode = MODE_MIT
                continue

            if choice == "r":
                path = record_trajectory_until_enter(can, robot)
                if path is not None:
                    last_traj_path = path
                    loaded_traj = load_trajectory(path)
                continue

            if choice == "p":
                if loaded_traj is None:
                    if last_traj_path is None:
                        print("[ERR] 没有可复现的轨迹，请先记录或加载轨迹")
                        continue
                    loaded_traj = load_trajectory(last_traj_path)

                ok = replay_trajectory_in_pv(can, loaded_traj)
                if all(m.Mode == MODE_PV for m in can.motors):
                    current_mode = MODE_PV
                if not ok:
                    print("[WARN] 本次复现未完成。若需要重新示教，请先按 1 切回 MIT 模式。")
                continue

            if choice == "l":
                path_str = input("请输入 CSV 轨迹文件路径: ").strip().strip('"')
                try:
                    loaded_traj = load_trajectory(path_str)
                    last_traj_path = loaded_traj.path
                except Exception as exc:
                    print(f"[ERR] 加载轨迹失败: {exc}")
                continue

            if choice == "s":
                print_motor_status(can)
                print_current_dh_angle(can, robot)
                continue

            print("[WARN] 无效输入")

    except KeyboardInterrupt:
        print("\n[INTERRUPT] 用户中断")

    except Exception as exc:
        print(f"[ERR] 程序异常: {exc}")

    finally:
        print("[CLEANUP] 准备退出...")

        gravity_stop_event.set()
        if gravity_thread is not None and gravity_thread.is_alive():
            gravity_thread.join(timeout=1.0)

        if can is not None:
            try:
                set_all_mit_zero_torque(can)
                time.sleep(0.05)
            except Exception as exc:
                print(f"[WARN] 清零 MIT 力矩异常: {exc}")

            if DISABLE_ON_EXIT:
                try:
                    print("[CLEANUP] 退出时失能前 6 个关节电机...")
                    disable_motors_only_at_exit(can)
                except Exception as exc:
                    print(f"[WARN] 退出失能异常: {exc}")

            try:
                can.stop_can()
                can.close_device()
            except Exception as exc:
                print(f"[WARN] 关闭 CANFD 设备异常: {exc}")

        print("[END] 程序退出")


if __name__ == "__main__":
    main()