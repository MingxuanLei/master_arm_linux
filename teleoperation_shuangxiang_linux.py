"""
teleoperation_shuangxiang_linux.py

Linux 版 7 电机弱双向同构遥操作：
    主端：MIT 重力补偿 + 从端弱反馈力矩；
    从端：PV 模式跟随主端相对 DH 变化；
    第 7 个工具电机：可选主从跟随，可选弱反馈。

相对于 Windows 版 teleoperation_shuangxiang_7motors.py 的主要变化：
    1. 不再使用 zlgcan.ZCAN 扫描设备，改为 libusbcanfd.so / VCI_* 接口扫描；
    2. 导入 GUIyemian_7motors_linux.ArmController；
    3. 强制让 GUI 控制器使用 USBCANFD_gai.USBCANFD，以支持同时打开两块 CANFD 设备；
    4. 保留原有弱双向反馈、从端安全夹紧、第 7 工具电机跟随等控制逻辑。

依赖文件建议放在同一目录：
    teleoperation_shuangxiang_linux.py
    GUIyemian_7motors_linux.py
    USBCANFD_gai.py
    USBCANFD_DEMO.py
    libusbcanfd.so
    DMMotor.py
    Robot.py
    TreeStruct.py
"""
from __future__ import annotations
from ctypes import Structure, byref, c_uint32, c_ubyte, c_ushort, c_void_p, create_string_buffer
import inspect
import math
import threading
import time
from types import MethodType
import numpy as np
# =========================
# 0. Linux 控制器导入
# =========================
import USBCANFD_gai as _usbcanfd_module
from USBCANFD_gai import USBCANFD as LinuxUSBCANFD
import GUIyemian_7motors_linux as _gui_module
_gui_module.USBCANFD = LinuxUSBCANFD
ArmController = _gui_module.ArmController
MODE_MIT = _gui_module.MODE_MIT
MODE_PV = _gui_module.MODE_PV
GRAVITY_COMP_PERIOD_S = _gui_module.GRAVITY_COMP_PERIOD_S
GRAVITY_TORQUE_SCALE = _gui_module.GRAVITY_TORQUE_SCALE
# =========================
# 1. CANFD 序列号配置
# =========================

MASTER_SERIAL = "9820ECA9B0E80D6418B0"
SLAVE_SERIAL = "EBD6C68A50FF0DD4F0B0"

DEVICE_SCAN_MAX = 3
CHANNEL_INDEX = 0
DEVICE_TYPE = int(getattr(getattr(LinuxUSBCANFD, "DEFAULT_DEVICE_TYPE", 43), "value", getattr(LinuxUSBCANFD, "DEFAULT_DEVICE_TYPE", 43)))

# =========================
# 2. 弱双向遥操作参数
# =========================

CONTROL_HZ = 500.0
PV_VEL_LIM = 0.8

# 每个控制周期允许从端目标 DH 角变化的最大值，单位 rad/cycle
MAX_DELTA_PER_CYCLE = 0.02

# 从端目标角低通滤波系数
ALPHA = 0.20

# 如果某个关节方向相反，把对应项改成 -1
SCALE = np.array([1, 1, 1, 1, 1, 1], dtype=float)

# 电机目标夹紧时离边界留一点余量，单位 rad
MOTOR_LIMIT_MARGIN = 0.003

# 每隔多少次循环打印一次夹紧信息，避免刷屏
CLIP_PRINT_INTERVAL = 100

# 是否打印安全夹紧信息
PRINT_CLIP_INFO = True

# 夹紧量超过该阈值时才打印，单位 rad；小于该值的轻微夹紧不刷屏
CLIP_PRINT_MIN_DELTA = 0.03

# =========================
# 2.1 第 7 个工具电机遥操作参数
# =========================

# 是否启用第 7 个工具电机主从遥操作
ENABLE_TOOL_TELEOP = True

# 第 7 个工具电机的主从映射比例；方向相反时改成 -1.0
TOOL_SCALE = 2.0

# 第 7 个工具电机单周期最大变化量，单位 rad/cycle
TOOL_MAX_DELTA_PER_CYCLE = 0.02

# 第 7 个工具电机低通滤波系数
TOOL_ALPHA = 0.20

# 第 7 个工具电机 PV 速度限制，默认与前 6 轴相同
TOOL_PV_VEL_LIM = PV_VEL_LIM


# =========================
# 3. 弱双向反馈参数
# =========================
# 弱双向含义：
# 主端 -> 从端：主端关节角驱动从端 PV 跟随
# 从端 -> 主端：从端跟踪误差和电机反馈力矩，转换成主端 MIT 反馈力矩

ENABLE_WEAK_BILATERAL = True

# 是否启用从端“位置跟踪误差”反馈
ENABLE_ERROR_FEEDBACK = True

# 是否启用从端“电机反馈力矩”反馈
ENABLE_TORQUE_FEEDBACK = True

# 启动遥操作前，自动采集一段从端空载/静止时的电机力矩作为零偏
AUTO_ZERO_SLAVE_TORQUE = True
SLAVE_TORQUE_ZERO_TIME_S = 0.5
SLAVE_TORQUE_ZERO_SAMPLE_HZ = 100.0

# 误差反馈：q_error = q_target - q_slave
# 若从端因为碰到物体、限位或阻力而跟不上目标，q_error 会变大；
# 主端反馈力矩默认取 -K * q_error，用来阻碍操作者继续往该方向推。
ERROR_FB_GAIN = np.array([0.80, 0.80, 0.60, 0.18, 0.15, 0.10], dtype=float)

# 误差死区，单位 rad。小误差不反馈，避免从端正常滞后造成主端抖动。
ERROR_FB_DEADZONE = np.array([0.015, 0.015, 0.015, 0.020, 0.020, 0.020], dtype=float)

# 力矩反馈：使用从端电机反馈力矩减去启动时零偏后的残差。
# 注意：电机反馈力矩不等价于真实末端接触力，只能作为弱反馈/阻力感来源。
TORQUE_FB_GAIN = np.array([0.08, 0.08, 0.07, 0.035, 0.030, 0.020], dtype=float)

# 从端电机力矩死区，单位取决于电机反馈 Torque 的单位，一般可先按 N·m 理解。
TORQUE_FB_DEADZONE = np.array([0.15, 0.15, 0.12, 0.08, 0.06, 0.05], dtype=float)

# 电机力矩反馈方向。
# 如果你发现“从端受阻时主端反而被助推”，把这个值改为 +1.0。
# 默认 -1.0 表示产生反向阻力。
TORQUE_FB_SIGN = -1.0

# 主端反馈力矩最大值，单位与 MIT torque_set 一致。
# 第一次实验一定要保守，确认方向正确后再逐步增大。
MASTER_FB_TAU_MAX = np.array([0.80, 0.80, 0.60, 0.30, 0.22, 0.15], dtype=float)

# 主端反馈力矩低通滤波，越小越平滑，越大越跟手。
FEEDBACK_ALPHA = 0.12

# 反馈力矩总开关掩码；如果某个关节不想反馈，设为 0。
FEEDBACK_MASK = np.array([1, 1, 1, 1, 1, 1], dtype=float)

# 每隔多少次循环打印一次反馈信息。
FEEDBACK_PRINT_INTERVAL = 300


# =========================
# 3.1 第 7 个工具电机弱反馈参数
# =========================
# 默认关闭第 7 电机弱反馈，只启用第 7 电机位置跟随。
# 如果确认方向和力矩安全后，可以改为 True。
ENABLE_TOOL_WEAK_FEEDBACK = True

# 工具电机位置误差反馈参数，单位 rad / torque_set
TOOL_ERROR_FB_GAIN = 0.05
TOOL_ERROR_FB_DEADZONE = 0.02

# 工具电机反馈力矩残差反馈参数
TOOL_TORQUE_FB_GAIN = 0.02
TOOL_TORQUE_FB_DEADZONE = 0.05
TOOL_TORQUE_FB_SIGN = -1.0

# 输出到主端第 7 电机的最大反馈力矩，第一次实验建议保守
TOOL_MASTER_FB_TAU_MAX = 0.08

# =========================
# 4. Linux CANFD 设备扫描函数
# =========================

_demo = _usbcanfd_module._demo
lib = _usbcanfd_module._demo.lib

CMD_SET_SN = int(getattr(_demo, "CMD_SET_SN", 0x42))
CMD_GET_SN = int(getattr(_demo, "CMD_GET_SN", 0x43))


def _configure_scan_ctypes() -> None:
    lib.VCI_OpenDevice.argtypes = [c_uint32, c_uint32, c_uint32]
    lib.VCI_OpenDevice.restype = c_uint32
    lib.VCI_CloseDevice.argtypes = [c_uint32, c_uint32]
    lib.VCI_CloseDevice.restype = c_uint32

    try:
        lib.VCI_ReadBoardInfo.argtypes = [c_uint32, c_uint32, c_void_p]
        lib.VCI_ReadBoardInfo.restype = c_uint32
    except AttributeError:
        pass

    try:
        lib.VCI_GetReference.argtypes = [c_uint32, c_uint32, c_uint32, c_uint32, c_void_p]
        lib.VCI_GetReference.restype = c_uint32
    except AttributeError:
        pass


_configure_scan_ctypes()


class VCI_BOARD_INFO(Structure):
    """常见 ZLG VCI 设备信息结构。"""
    _fields_ = [
        ("hw_Version", c_ushort),
        ("fw_Version", c_ushort),
        ("dr_Version", c_ushort),
        ("in_Version", c_ushort),
        ("irq_Num", c_ushort),
        ("can_Num", c_ubyte),
        ("str_Serial_Num", c_ubyte * 20),
        ("str_hw_Type", c_ubyte * 40),
        ("Reserved", c_ushort * 4),
    ]


def normalize_serial(serial):
    if serial is None:
        return ""
    return str(serial).strip().upper()


def _decode_c_ubyte_array(arr) -> str:
    values = []
    for x in arr:
        v = int(x)
        if v == 0:
            break
        values.append(v)
    return bytes(values).decode("ascii", errors="ignore")


def _format_version(v: int) -> str:
    v = int(v)
    hi = (v >> 8) & 0xFF
    lo = v & 0xFF
    return f"V{hi}.{lo:02X}"


def _read_board_info(device_type: int, device_index: int):
    if not hasattr(lib, "VCI_ReadBoardInfo"):
        return None

    info = VCI_BOARD_INFO()
    try:
        ret = lib.VCI_ReadBoardInfo(c_uint32(device_type), c_uint32(device_index), byref(info))
    except Exception:
        return None

    if int(ret) == 0:
        return None

    return {
        "serial": normalize_serial(_decode_c_ubyte_array(info.str_Serial_Num)),
        "hw_type": _decode_c_ubyte_array(info.str_hw_Type),
        "can_num": int(info.can_Num),
        "hw_version": _format_version(info.hw_Version),
        "fw_version": _format_version(info.fw_Version),
        "source": "VCI_ReadBoardInfo",
    }


def _try_get_reference_string(device_type: int, device_index: int, channel_index: int, cmd: int, size: int = 128):
    if not hasattr(lib, "VCI_GetReference"):
        return None

    buf = create_string_buffer(size)
    try:
        ret = lib.VCI_GetReference(
            c_uint32(device_type),
            c_uint32(device_index),
            c_uint32(channel_index),
            c_uint32(cmd),
            byref(buf),
        )
    except Exception:
        return None

    if int(ret) == 0:
        return None

    raw = bytes(buf.raw).split(b"\x00", 1)[0]
    text = raw.decode("ascii", errors="ignore").strip()
    return normalize_serial(text) if text else None


def _read_serial_by_reference(device_type: int, device_index: int, channel_index: int):
    # 不同版本 demo 对 CMD_SET_SN / CMD_GET_SN 的中文注释可能写反，所以两个都尝试。
    for cmd_name, cmd in (("CMD_GET_SN(0x43)", CMD_GET_SN), ("CMD_SET_SN(0x42)", CMD_SET_SN)):
        text = _try_get_reference_string(device_type, device_index, channel_index, cmd)
        if text:
            return text, cmd_name
    return None, None


def scan_canfd_devices(max_index=8, device_type: int = DEVICE_TYPE, channel_index: int = CHANNEL_INDEX):
    """
    Linux 版设备扫描：逐个 device_index 调用 VCI_OpenDevice，读取序列号后关闭。
    返回格式与 Windows 版保持一致：{serial: {device_index, hw_type, can_num}}
    """
    devices = {}

    print("=" * 70)
    print("开始扫描 CANFD 设备...")
    print(f"device_type={device_type}, max_index={max_index}, channel_index={channel_index}")
    print("=" * 70)

    for idx in range(int(max_index)):
        try:
            ret = lib.VCI_OpenDevice(c_uint32(device_type), c_uint32(idx), c_uint32(0))
        except Exception as e:
            print(f"device_index={idx}: 打开异常: {e}")
            continue

        if int(ret) == 0:
            print(f"device_index={idx}: 打开失败")
            continue

        print(f"device_index={idx}: 打开成功")
        print(f"  open_ret = {int(ret)}")

        info = _read_board_info(device_type, idx)
        serial_from_ref, ref_source = _read_serial_by_reference(device_type, idx, channel_index)

        if info is None:
            info = {
                "serial": serial_from_ref or "",
                "hw_type": "",
                "can_num": None,
                "hw_version": "",
                "fw_version": "",
                "source": ref_source or "未读取到完整信息",
            }
        else:
            if serial_from_ref and not info.get("serial"):
                info["serial"] = serial_from_ref
            if serial_from_ref:
                info["sn_ref"] = f"{serial_from_ref} ({ref_source})"

        serial = normalize_serial(info.get("serial", ""))

        print(f"  serial     = {serial if serial else '未读取到'}")
        print(f"  hw_type    = {info.get('hw_type', '')}")
        print(f"  can_num    = {info.get('can_num', '')}")
        print(f"  hw_version = {info.get('hw_version', '')}")
        print(f"  fw_version = {info.get('fw_version', '')}")
        print(f"  info_src   = {info.get('source', '')}")
        if info.get("sn_ref"):
            print(f"  sn_ref     = {info['sn_ref']}")

        if serial:
            devices[serial] = {
                "device_index": idx,
                "hw_type": info.get("hw_type", ""),
                "can_num": info.get("can_num", None),
            }

        try:
            close_ret = lib.VCI_CloseDevice(c_uint32(device_type), c_uint32(idx))
            print(f"  close_ret  = {int(close_ret)}")
        except Exception as e:
            print(f"  关闭设备异常: {e}")

        time.sleep(0.05)

    print("=" * 70)
    print(f"扫描完成，共发现 {len(devices)} 个带序列号的可用 CANFD 设备")

    for serial, item in devices.items():
        print(f"serial={serial}, device_index={item['device_index']}, hw_type={item.get('hw_type', '')}")

    print("=" * 70)
    return devices


def get_device_index_by_serial(devices, target_serial, role_name):
    target_serial = normalize_serial(target_serial)

    if target_serial not in devices:
        print(f"[ERR] 没有找到 {role_name} CANFD 设备")
        print(f"[ERR] 目标 serial = {target_serial}")
        print("[ERR] 当前扫描到的 serial 有：")

        for serial in devices.keys():
            print(f"  {serial}")

        raise RuntimeError(f"未找到 {role_name} CANFD 设备: serial={target_serial}")

    device_index = int(devices[target_serial]["device_index"])

    print(f"[OK] {role_name} CANFD 匹配成功")
    print(f"     serial       = {target_serial}")
    print(f"     device_index = {device_index}")

    return device_index

# =========================
# 5. 角度处理函数
# =========================

def wrap_to_pi(q):
    q = np.asarray(q, dtype=float)
    return (q + math.pi) % (2 * math.pi) - math.pi


def angle_diff(q_target, q_now):
    """
    返回 q_target - q_now 的最短角度差，范围 [-pi, pi]。
    注意：这个函数只用于计算差值，不用于强行包裹绝对目标角。
    """
    return wrap_to_pi(np.asarray(q_target, dtype=float) - np.asarray(q_now, dtype=float))


def limit_delta_continuous(q_target, q_last, max_delta):
    q_target = np.asarray(q_target, dtype=float)
    q_last = np.asarray(q_last, dtype=float)

    delta = angle_diff(q_target, q_last)
    delta = np.clip(delta, -max_delta, max_delta)

    return q_last + delta


def lowpass_continuous(q_target, q_last, alpha):
    q_target = np.asarray(q_target, dtype=float)
    q_last = np.asarray(q_last, dtype=float)

    delta = angle_diff(q_target, q_last)
    return q_last + alpha * delta


def deadzone_vector(x, dz):
    """
    对向量做死区处理：
    |x| <= dz 时输出 0；
    |x| > dz 时输出 sign(x) * (|x|-dz)。
    """
    x = np.asarray(x, dtype=float)
    dz = np.asarray(dz, dtype=float)
    return np.sign(x) * np.maximum(np.abs(x) - dz, 0.0)


def as_float(x):
    return float(np.asarray(x, dtype=float).reshape(()))


def get_dh(controller):
    snapshot = controller.get_status_snapshot()

    if snapshot is None:
        return None

    return np.array(snapshot["dh_rad"], dtype=float)


def get_motor_torque(controller):
    snapshot = controller.get_status_snapshot()

    if snapshot is None:
        return None

    motors = snapshot.get("motors", [])
    if len(motors) < 6:
        return None

    return np.array([float(m["tau"]) for m in motors[:6]], dtype=float)


# =========================
# 5.1 第 7 个工具电机状态函数
# =========================

def get_tool_motor(controller):
    if controller is None or controller.can is None:
        return None
    tools = getattr(controller.can, "tools", [])
    if not tools:
        return None
    return tools[0]


def get_tool_position(controller):
    tool = get_tool_motor(controller)
    if tool is None:
        return None
    return float(tool.Position)


def get_tool_torque(controller):
    tool = get_tool_motor(controller)
    if tool is None:
        return None
    return float(tool.Torque)


def check_7motor_controller(controller, role_name):
    """确认当前导入的 ArmController 版本真正支持 1~7 号电机。"""
    if not hasattr(controller, "all_actuators"):
        raise RuntimeError(
            f"{role_name} 当前导入的 ArmController 仍是 6 电机版本。\n"
            "请把之前生成的 GUIyemian_7motors.py 放到当前目录，"
            "或者把 7 电机版 GUI 文件命名/替换为 GUIyemian.py。"
        )

    sig = inspect.signature(controller.set_pv_target_motor_position)
    if "tool_target_q" not in sig.parameters:
        raise RuntimeError(
            f"{role_name} 的 set_pv_target_motor_position() 不支持 tool_target_q 参数。\n"
            "请使用支持第 7 个工具电机的 GUI 控制器文件。"
        )


# =========================
# 6. 从端安全目标写入
# =========================

def dh_to_motor_near_current_clamped(controller, target_dh, margin=0.003):
    """
    将目标 DH 角转换为电机角。

    与 robot.dh2motor() 的区别：
    1. 会优先选择最接近当前电机位置的 2pi 等效电机角；
    2. 如果目标略微超出电机限位，会夹紧到限位内部；
    3. 适合遥操作连续跟随，避免实际反馈在边界附近造成反复超限。
    """
    if controller.can is None or controller.robot is None:
        raise RuntimeError("controller 尚未初始化")

    target_dh = np.asarray(target_dh, dtype=float).reshape(6)

    ratio = np.asarray(controller.robot.ratio, dtype=float)
    zero_offset = np.asarray(controller.robot.zero_offset, dtype=float)

    motor_target = np.zeros(6, dtype=float)
    clip_infos = []

    for i, motor in enumerate(controller.can.motors):
        lo = float(motor.angle_lim[0])
        hi = float(motor.angle_lim[1])

        # 原始 DH -> motor
        raw = (target_dh[i] - zero_offset[i]) * ratio[i]

        # 选择与当前电机反馈最接近的等效角
        current_motor = float(motor.Position)
        candidates = np.array([raw, raw + 2.0 * math.pi, raw - 2.0 * math.pi], dtype=float)

        valid_candidates = [c for c in candidates if lo + margin <= c <= hi - margin]

        if valid_candidates:
            chosen = min(valid_candidates, key=lambda x: abs(x - current_motor))
            clipped = chosen
            was_clipped = False
        else:
            chosen = min(candidates, key=lambda x: abs(x - current_motor))
            clipped = float(np.clip(chosen, lo + margin, hi - margin))
            was_clipped = abs(clipped - chosen) > 1e-9

        motor_target[i] = clipped

        if was_clipped:
            clip_infos.append({
                "joint": i + 1,
                "dh": float(target_dh[i]),
                "raw_motor": float(chosen),
                "clipped_motor": float(clipped),
                "limit": [lo, hi],
            })

    return motor_target, clip_infos


def tool_to_motor_near_current_clamped(controller, target_tool_q, margin=0.003):
    """
    第 7 个工具电机不属于 6 轴 DH 模型，直接按电机角进行主从映射。
    这里同样选择最接近当前电机位置的 2pi 等效角，并做限位夹紧。
    """
    tool = get_tool_motor(controller)
    if tool is None:
        raise RuntimeError("未检测到第 7 个工具电机")

    lo = float(tool.angle_lim[0])
    hi = float(tool.angle_lim[1])
    raw = float(target_tool_q)
    current_tool = float(tool.Position)

    candidates = np.array([raw, raw + 2.0 * math.pi, raw - 2.0 * math.pi], dtype=float)
    valid_candidates = [c for c in candidates if lo + margin <= c <= hi - margin]

    if valid_candidates:
        chosen = min(valid_candidates, key=lambda x: abs(x - current_tool))
        clipped = chosen
        was_clipped = False
    else:
        chosen = min(candidates, key=lambda x: abs(x - current_tool))
        clipped = float(np.clip(chosen, lo + margin, hi - margin))
        was_clipped = abs(clipped - chosen) > 1e-9

    clip_infos = []
    if was_clipped:
        clip_infos.append({
            "joint": 7,
            "dh": None,
            "raw_motor": float(chosen),
            "clipped_motor": float(clipped),
            "limit": [lo, hi],
        })

    return float(clipped), clip_infos


def print_clip_infos(clip_infos, loop_count):
    large_clip_infos = [
        item for item in clip_infos
        if abs(item["clipped_motor"] - item["raw_motor"]) >= CLIP_PRINT_MIN_DELTA
    ]

    if not (PRINT_CLIP_INFO and large_clip_infos and loop_count % CLIP_PRINT_INTERVAL == 0):
        return

    print("[INFO] 从端目标接近或超过电机限位，已进行安全夹紧：")
    for item in large_clip_infos:
        name = "工具电机7" if item["joint"] == 7 else f"关节{item['joint']}"
        print(
            f"  {name}: "
            f"raw_motor={item['raw_motor']:.4f}, "
            f"clipped={item['clipped_motor']:.4f}, "
            f"limit=[{item['limit'][0]:.4f}, {item['limit'][1]:.4f}]"
        )


def safe_set_slave_target_dh(slave, target_dh, velocity_lim, loop_count=0):
    """
    从端安全写入目标：
    仅写入 1~6 号机械臂关节；第 7 个工具电机保持当前位置。
    """
    try:
        motor_target, clip_infos = dh_to_motor_near_current_clamped(slave, target_dh, MOTOR_LIMIT_MARGIN)
        print_clip_infos(clip_infos, loop_count)
        ok = slave.set_pv_target_motor_position(motor_target.tolist(), velocity_lim)
        return ok

    except Exception as e:
        print(f"[WARN] safe_set_slave_target_dh 异常: {e}")
        return False


def safe_set_slave_target_7d(slave, target_dh, target_tool_q, velocity_lim, tool_velocity_lim, loop_count=0):
    """
    从端安全写入目标：
    1~6 号：DH 角 -> 电机角，必要时夹紧；
    7 号：工具电机角直接映射，必要时夹紧；
    然后统一写入 PV 目标。
    """
    try:
        motor_target, clip_infos_6 = dh_to_motor_near_current_clamped(slave, target_dh, MOTOR_LIMIT_MARGIN)
        tool_target, clip_infos_7 = tool_to_motor_near_current_clamped(slave, target_tool_q, MOTOR_LIMIT_MARGIN)

        print_clip_infos(clip_infos_6 + clip_infos_7, loop_count)

        ok = slave.set_pv_target_motor_position(
            motor_target.tolist(),
            velocity_lim,
            tool_target_q=tool_target,
        )

        return ok, float(tool_target)

    except Exception as e:
        print(f"[WARN] safe_set_slave_target_7d 异常: {e}")
        return False, None


# =========================
# 7. 主端弱双向反馈力矩叠加
# =========================

def enable_master_external_feedback(controller):
    """
    给主端 ArmController 增加外部反馈力矩通道。

    前 6 轴：tau_master = tau_g + external_feedback_tau
    第 7 轴：默认零力矩；若 ENABLE_TOOL_WEAK_FEEDBACK=True，则叠加 external_tool_feedback_tau

    注意：这个函数必须在 master.initialize_system() 之前调用，
    因为 initialize_system() 内部会启动重力补偿线程。
    """
    controller.external_feedback_tau = np.zeros(6, dtype=float)
    controller.external_tool_feedback_tau = 0.0
    controller.external_feedback_lock = threading.RLock()

    def set_external_feedback_tau(self, tau, tool_tau=0.0):
        tau_arr = np.asarray(tau, dtype=float).reshape(-1)
        if tau_arr.size < 6:
            raise ValueError("tau 至少需要包含前 6 个关节的反馈力矩")
        with self.external_feedback_lock:
            self.external_feedback_tau = tau_arr[:6].copy()
            if tau_arr.size >= 7:
                self.external_tool_feedback_tau = float(tau_arr[6])
            else:
                self.external_tool_feedback_tau = float(tool_tau)

    def get_external_feedback_tau(self):
        with self.external_feedback_lock:
            return np.concatenate([
                self.external_feedback_tau.copy(),
                np.array([self.external_tool_feedback_tau], dtype=float),
            ])

    def _pack_mit_command(self, motor):
        if hasattr(self, "pack_command_for_mode"):
            self.pack_command_for_mode(motor, MODE_MIT)
        else:
            motor.set()

    def gravity_comp_loop_with_feedback(self):
        assert self.can is not None
        assert self.robot is not None

        self.log("[GRAVITY] 重力补偿线程已启动：tau = tau_g + 弱双向反馈力矩，第7电机默认零力矩")

        while not self.gravity_stop_event.is_set():
            try:
                with self.data_lock:
                    self.robot.Angle = self.robot.motor2dh(self.can.motors)

                    if not self.robot.set_robot():
                        time.sleep(GRAVITY_COMP_PERIOD_S)
                        continue

                    tau_g_motor = self.robot.Tau_G_Motor

                    with self.external_feedback_lock:
                        tau_fb = self.external_feedback_tau.copy()
                        tau_tool_fb = float(self.external_tool_feedback_tau)

                    for i, motor in enumerate(self.can.motors):
                        motor.MIT.position_set = 0.0
                        motor.MIT.velocity_set = 0.0
                        motor.MIT.kp_set = 0.0
                        motor.MIT.kd_set = 0.0
                        motor.MIT.torque_set = float(tau_g_motor[i] * GRAVITY_TORQUE_SCALE[i] + tau_fb[i])
                        _pack_mit_command(self, motor)

                    for tool in getattr(self.can, "tools", []):
                        tool.MIT.position_set = 0.0
                        tool.MIT.velocity_set = 0.0
                        tool.MIT.kp_set = 0.0
                        tool.MIT.kd_set = 0.0
                        tool.MIT.torque_set = float(tau_tool_fb if ENABLE_TOOL_WEAK_FEEDBACK else 0.0)
                        _pack_mit_command(self, tool)

                time.sleep(GRAVITY_COMP_PERIOD_S)

            except Exception as e:
                self.log(f"[ERR] 重力补偿/反馈线程异常: {e}")
                time.sleep(0.01)

        self.log("[GRAVITY] 重力补偿/反馈线程退出")

    controller.set_external_feedback_tau = MethodType(set_external_feedback_tau, controller)
    controller.get_external_feedback_tau = MethodType(get_external_feedback_tau, controller)
    controller.gravity_comp_loop = MethodType(gravity_comp_loop_with_feedback, controller)


def measure_slave_torque_zero(slave, sample_time_s=0.5, sample_hz=100.0):
    """
    采集从端静止时的电机反馈力矩均值作为零偏。
    这样后续反馈使用 tau_slave - tau_zero，避免把静态保持力矩一直反馈到主端。
    """
    samples = []
    dt = 1.0 / max(float(sample_hz), 1.0)
    end_time = time.time() + float(sample_time_s)

    print(f"[ZERO] 开始采集从端电机力矩零偏，持续 {sample_time_s:.2f}s...")

    while time.time() < end_time:
        tau = get_motor_torque(slave)
        if tau is not None and tau.shape == (6,):
            samples.append(tau.copy())
        time.sleep(dt)

    if not samples:
        print("[WARN] 从端电机力矩零偏采集失败，使用 0 作为零偏")
        return np.zeros(6, dtype=float)

    tau_zero = np.mean(np.vstack(samples), axis=0)

    print("[ZERO] 从端电机力矩零偏:")
    print(["{:.4f}".format(x) for x in tau_zero])

    return tau_zero



def measure_slave_tool_torque_zero(slave, sample_time_s=0.5, sample_hz=100.0):
    """采集从端第 7 个工具电机静止时的反馈力矩均值作为零偏。"""
    samples = []
    dt = 1.0 / max(float(sample_hz), 1.0)
    end_time = time.time() + float(sample_time_s)

    print(f"[ZERO] 开始采集从端第7工具电机力矩零偏，持续 {sample_time_s:.2f}s...")

    while time.time() < end_time:
        tau = get_tool_torque(slave)
        if tau is not None:
            samples.append(float(tau))
        time.sleep(dt)

    if not samples:
        print("[WARN] 从端第7工具电机力矩零偏采集失败，使用 0 作为零偏")
        return 0.0

    tau_zero = float(np.mean(samples))
    print(f"[ZERO] 从端第7工具电机力矩零偏: {tau_zero:.4f}")
    return tau_zero


def compute_weak_bilateral_feedback(q_target, q_slave, tau_slave, tau_slave_zero, tau_fb_last):
    """
    根据从端反馈计算主端外部反馈力矩。

    输入：
        q_target: 当前给从端的目标 DH 角
        q_slave: 从端当前 DH 角
        tau_slave: 从端当前电机反馈力矩
        tau_slave_zero: 从端静止零偏力矩
        tau_fb_last: 上一次输出到主端的反馈力矩

    输出：
        tau_fb: 当前输出到主端 MIT 的反馈力矩
        info: 诊断信息
    """
    tau_fb_total = np.zeros(6, dtype=float)

    q_error = angle_diff(q_target, q_slave)
    q_error_eff = deadzone_vector(q_error, ERROR_FB_DEADZONE)

    tau_from_error = np.zeros(6, dtype=float)
    if ENABLE_ERROR_FEEDBACK:
        # q_error 为正：从端实际落后于正方向目标，主端施加反向阻力
        tau_from_error = -ERROR_FB_GAIN * SCALE * q_error_eff
        tau_fb_total += tau_from_error

    tau_residual = np.zeros(6, dtype=float)
    tau_residual_eff = np.zeros(6, dtype=float)
    tau_from_torque = np.zeros(6, dtype=float)

    if ENABLE_TORQUE_FEEDBACK and tau_slave is not None:
        tau_residual = np.asarray(tau_slave, dtype=float).reshape(6) - np.asarray(tau_slave_zero, dtype=float).reshape(6)
        tau_residual_eff = deadzone_vector(tau_residual, TORQUE_FB_DEADZONE)

        # TORQUE_FB_SIGN 默认 -1，如果发现方向反了，改成 +1
        tau_from_torque = TORQUE_FB_SIGN * TORQUE_FB_GAIN * SCALE * tau_residual_eff
        tau_fb_total += tau_from_torque

    tau_fb_total *= FEEDBACK_MASK
    tau_fb_total = np.clip(tau_fb_total, -MASTER_FB_TAU_MAX, MASTER_FB_TAU_MAX)

    # 对反馈力矩做低通滤波，避免主端手感发抖
    tau_fb = np.asarray(tau_fb_last, dtype=float).reshape(6) + FEEDBACK_ALPHA * (
        tau_fb_total - np.asarray(tau_fb_last, dtype=float).reshape(6)
    )

    info = {
        "q_error": q_error,
        "q_error_eff": q_error_eff,
        "tau_slave": tau_slave,
        "tau_residual": tau_residual,
        "tau_residual_eff": tau_residual_eff,
        "tau_from_error": tau_from_error,
        "tau_from_torque": tau_from_torque,
        "tau_fb_raw": tau_fb_total,
        "tau_fb": tau_fb,
    }

    return tau_fb, info


def compute_tool_weak_bilateral_feedback(q_tool_target, q_tool_slave, tau_tool_slave, tau_tool_zero, tau_tool_fb_last):
    """
    第 7 个工具电机的弱反馈。默认由 ENABLE_TOOL_WEAK_FEEDBACK 控制，
    不影响前 6 轴原有弱双向反馈算法。
    """
    q_error = as_float(angle_diff(q_tool_target, q_tool_slave))
    q_error_eff = as_float(deadzone_vector(q_error, TOOL_ERROR_FB_DEADZONE))

    tau_from_error = 0.0
    if ENABLE_ERROR_FEEDBACK:
        tau_from_error = -float(TOOL_ERROR_FB_GAIN) * float(TOOL_SCALE) * q_error_eff

    tau_residual = 0.0
    tau_residual_eff = 0.0
    tau_from_torque = 0.0

    if ENABLE_TORQUE_FEEDBACK and tau_tool_slave is not None:
        tau_residual = float(tau_tool_slave) - float(tau_tool_zero)
        tau_residual_eff = as_float(deadzone_vector(tau_residual, TOOL_TORQUE_FB_DEADZONE))
        tau_from_torque = float(TOOL_TORQUE_FB_SIGN) * float(TOOL_TORQUE_FB_GAIN) * float(TOOL_SCALE) * tau_residual_eff

    tau_fb_raw = tau_from_error + tau_from_torque
    tau_fb_raw = float(np.clip(tau_fb_raw, -TOOL_MASTER_FB_TAU_MAX, TOOL_MASTER_FB_TAU_MAX))
    tau_fb = float(tau_tool_fb_last) + FEEDBACK_ALPHA * (tau_fb_raw - float(tau_tool_fb_last))

    info = {
        "q_error": q_error,
        "q_error_eff": q_error_eff,
        "tau_tool_slave": tau_tool_slave,
        "tau_residual": tau_residual,
        "tau_residual_eff": tau_residual_eff,
        "tau_from_error": tau_from_error,
        "tau_from_torque": tau_from_torque,
        "tau_fb_raw": tau_fb_raw,
        "tau_fb": tau_fb,
    }

    return tau_fb, info


# =========================
# 8. 主程序
# =========================

def main():
    devices = scan_canfd_devices(DEVICE_SCAN_MAX)

    master_index = get_device_index_by_serial(devices, MASTER_SERIAL, "主端")
    slave_index = get_device_index_by_serial(devices, SLAVE_SERIAL, "从端")

    if master_index == slave_index:
        raise RuntimeError("主端和从端匹配到了同一个 device_index，请检查 serial 配置")

    print()
    print("最终设备匹配结果：")
    print(f"  主端 CANFD: serial={MASTER_SERIAL}, device_index={master_index}")
    print(f"  从端 CANFD: serial={SLAVE_SERIAL}, device_index={slave_index}")
    print()

    master = ArmController(device_index=master_index, channel_index=CHANNEL_INDEX, name="master")
    slave = ArmController(device_index=slave_index, channel_index=CHANNEL_INDEX, name="slave")

    # 关键：在主端初始化前替换重力补偿线程，使其支持叠加弱双向反馈力矩
    enable_master_external_feedback(master)

    tau_slave_zero = np.zeros(6, dtype=float)
    tau_fb_last = np.zeros(6, dtype=float)
    tau_slave_tool_zero = 0.0
    tau_tool_fb_last = 0.0

    try:
        if ENABLE_TOOL_TELEOP or ENABLE_TOOL_WEAK_FEEDBACK:
            check_7motor_controller(master, "主端")
            check_7motor_controller(slave, "从端")

        print("初始化主端机械臂...")
        if not master.initialize_system():
            print("[ERR] 主端初始化失败")
            return

        print("初始化从端机械臂...")
        if not slave.initialize_system():
            print("[ERR] 从端初始化失败")
            return

        time.sleep(1.0)

        q_master_home = get_dh(master)
        q_slave_home = get_dh(slave)

        if q_master_home is None or q_slave_home is None:
            print("[ERR] 无法读取初始关节角")
            return

        q_master_tool_home = get_tool_position(master)
        q_slave_tool_home = get_tool_position(slave)

        if ENABLE_TOOL_TELEOP and (q_master_tool_home is None or q_slave_tool_home is None):
            print("[ERR] 无法读取第 7 个工具电机初始角度")
            return

        print()
        print("主端初始 DH 角 rad:")
        print(["{:.4f}".format(x) for x in q_master_home])

        print("从端初始 DH 角 rad:")
        print(["{:.4f}".format(x) for x in q_slave_home])

        if ENABLE_TOOL_TELEOP:
            print(f"主端第 7 工具电机初始角 rad: {q_master_tool_home:.4f}")
            print(f"从端第 7 工具电机初始角 rad: {q_slave_tool_home:.4f}")
        print()

        print("主端切换到 MIT 模式...")
        if not master.switch_mode(MODE_MIT, None, PV_VEL_LIM):
            print("[ERR] 主端切换 MIT 模式失败")
            return

        print("从端切换到 PV 模式...")
        # 不再用 q_slave_home 作为 PV 目标，避免初始点边界附近反算超限。
        # 使用 7 电机版控制器时，第 7 个工具电机也会先保持当前位置。
        if not slave.switch_mode(MODE_PV, None, PV_VEL_LIM):
            print("[ERR] 从端切换 PV 模式失败")
            return

        if AUTO_ZERO_SLAVE_TORQUE:
            tau_slave_zero = measure_slave_torque_zero(
                slave,
                sample_time_s=SLAVE_TORQUE_ZERO_TIME_S,
                sample_hz=SLAVE_TORQUE_ZERO_SAMPLE_HZ,
            )
            if ENABLE_TOOL_WEAK_FEEDBACK:
                tau_slave_tool_zero = measure_slave_tool_torque_zero(
                    slave,
                    sample_time_s=SLAVE_TORQUE_ZERO_TIME_S,
                    sample_hz=SLAVE_TORQUE_ZERO_SAMPLE_HZ,
                )

        q_cmd_last = q_slave_home.copy()
        q_tool_cmd_last = float(q_slave_tool_home) if ENABLE_TOOL_TELEOP else 0.0

        dt = 1.0 / CONTROL_HZ
        loop_count = 0

        print()
        print("开始弱双向同构遥操作")
        print("主端：MIT 重力补偿 + 从端弱反馈力矩")
        print("从端：PV 模式跟随")
        print("第 7 个工具电机：" + ("启用主从同构跟随" if ENABLE_TOOL_TELEOP else "未启用"))
        print(f"弱双向反馈：{'开启' if ENABLE_WEAK_BILATERAL else '关闭'}")
        print(f"误差反馈：{'开启' if ENABLE_ERROR_FEEDBACK else '关闭'}")
        print(f"电机力矩反馈：{'开启' if ENABLE_TORQUE_FEEDBACK else '关闭'}")
        print(f"第7电机弱反馈：{'开启' if ENABLE_TOOL_WEAK_FEEDBACK else '关闭'}")
        print("按 Ctrl+C 停止")
        print()

        while True:
            loop_start = time.time()
            loop_count += 1

            q_master = get_dh(master)
            q_slave = get_dh(slave)
            tau_slave = get_motor_torque(slave)

            if q_master is None or q_slave is None:
                time.sleep(dt)
                continue

            # 主端相对初始姿态变化量
            delta_master = angle_diff(q_master, q_master_home)

            # 从端目标 DH 角，保持连续形式
            q_target_raw = q_slave_home + SCALE * delta_master

            # 单周期限幅，防止目标突变
            q_target = limit_delta_continuous(q_target_raw, q_cmd_last, MAX_DELTA_PER_CYCLE)

            # 低通滤波，使从端运动更平滑
            q_target = lowpass_continuous(q_target, q_cmd_last, ALPHA)

            q_tool_target = q_tool_cmd_last
            q_slave_tool = get_tool_position(slave) if ENABLE_TOOL_TELEOP else None
            tau_slave_tool = get_tool_torque(slave) if ENABLE_TOOL_WEAK_FEEDBACK else None

            if ENABLE_TOOL_TELEOP:
                q_master_tool = get_tool_position(master)
                if q_master_tool is None or q_slave_tool is None:
                    time.sleep(dt)
                    continue

                # 第 7 个工具电机不走 DH，直接用主端工具电机相对变化量映射到从端工具电机。
                delta_tool = as_float(angle_diff(q_master_tool, q_master_tool_home))
                q_tool_target_raw = q_slave_tool_home + TOOL_SCALE * delta_tool

                q_tool_target = limit_delta_continuous(q_tool_target_raw, q_tool_cmd_last, TOOL_MAX_DELTA_PER_CYCLE)
                q_tool_target = lowpass_continuous(q_tool_target, q_tool_cmd_last, TOOL_ALPHA)
                q_tool_target = as_float(q_tool_target)

                # 安全写入从端 PV 目标，包含前 6 轴和第 7 个工具电机
                ok, q_tool_written = safe_set_slave_target_7d(
                    slave,
                    q_target,
                    q_tool_target,
                    PV_VEL_LIM,
                    TOOL_PV_VEL_LIM,
                    loop_count,
                )
            else:
                ok = safe_set_slave_target_dh(slave, q_target, PV_VEL_LIM, loop_count)
                q_tool_written = None

            if ok:
                q_cmd_last = q_target.copy()
                if ENABLE_TOOL_TELEOP and q_tool_written is not None:
                    q_tool_cmd_last = float(q_tool_written)
            else:
                print("[WARN] 从端目标写入失败，保持当前位置")
                slave.set_pv_hold_current_position(PV_VEL_LIM)

            # =========================
            # 从端 -> 主端：弱双向反馈
            # =========================
            if ENABLE_WEAK_BILATERAL:
                tau_fb, fb_info = compute_weak_bilateral_feedback(
                    q_target=q_target,
                    q_slave=q_slave,
                    tau_slave=tau_slave,
                    tau_slave_zero=tau_slave_zero,
                    tau_fb_last=tau_fb_last,
                )
                tau_fb_last = tau_fb.copy()

                tau_tool_fb = 0.0
                tool_fb_info = None
                if ENABLE_TOOL_WEAK_FEEDBACK and ENABLE_TOOL_TELEOP and q_slave_tool is not None:
                    tau_tool_fb, tool_fb_info = compute_tool_weak_bilateral_feedback(
                        q_tool_target=q_tool_target,
                        q_tool_slave=q_slave_tool,
                        tau_tool_slave=tau_slave_tool,
                        tau_tool_zero=tau_slave_tool_zero,
                        tau_tool_fb_last=tau_tool_fb_last,
                    )
                    tau_tool_fb_last = float(tau_tool_fb)

                master.set_external_feedback_tau(tau_fb, tool_tau=tau_tool_fb)

                if loop_count % FEEDBACK_PRINT_INTERVAL == 0:
                    print("[FB] 从端反馈 -> 主端 MIT 反馈力矩")
                    print("  q_error rad       =", ["{:.4f}".format(x) for x in fb_info["q_error"]])
                    if tau_slave is not None:
                        print("  tau_slave         =", ["{:.4f}".format(x) for x in tau_slave])
                        print("  tau_slave-zero    =", ["{:.4f}".format(x) for x in fb_info["tau_residual"]])
                    print("  tau_fb_to_master  =", ["{:.4f}".format(x) for x in fb_info["tau_fb"]])
                    if tool_fb_info is not None:
                        print(f"  tool_q_error      = {tool_fb_info['q_error']:.4f}")
                        print(f"  tool_tau_fb       = {tool_fb_info['tau_fb']:.4f}")
            else:
                master.set_external_feedback_tau(np.zeros(6, dtype=float), tool_tau=0.0)

            elapsed = time.time() - loop_start
            time.sleep(max(0.0, dt - elapsed))

    except KeyboardInterrupt:
        print()
        print("收到 Ctrl+C，停止遥操作")

    finally:
        print("清零主端反馈力矩...")
        try:
            master.set_external_feedback_tau(np.zeros(6, dtype=float), tool_tau=0.0)
            time.sleep(0.05)
        except Exception as e:
            print(f"[WARN] 清零主端反馈力矩异常: {e}")

        print("清理从端机械臂...")
        try:
            slave.cleanup()
        except Exception as e:
            print(f"[WARN] 从端清理异常: {e}")

        print("清理主端机械臂...")
        try:
            master.cleanup()
        except Exception as e:
            print(f"[WARN] 主端清理异常: {e}")

        print("程序结束")


if __name__ == "__main__":
    main()
