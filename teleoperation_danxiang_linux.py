"""
teleoperation_danxiang_linux.py
Linux 版 7 电机单向同构遥操作：
    主端：MIT 重力补偿，可手拖；
    从端：PV 模式跟随主端相对 DH 变化；
    第 7 个工具电机：可选主从跟随。
相对于 Windows 版 teleoperation_danxiang_7motors.py 的主要变化：
    1. 不再使用 zlgcan.ZCAN 扫描设备，改为 libusbcanfd.so / VCI_* 接口扫描；
    2. 导入 GUIyemian_7motors_linux.ArmController；
    3. 强制让 GUI 控制器使用 USBCANFD_gai.USBCANFD，以支持同时打开两块 CANFD 设备；
    4. 其余遥操作控制逻辑基本保持不变。
依赖文件建议放在同一目录：
    teleoperation_danxiang_linux.py
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
import time
import numpy as np
# =========================
# 0. Linux 控制器导入
# =========================
# 必须使用支持“多设备打开/引用计数”的 Linux 版 USBCANFD_gai。
import USBCANFD_gai as _usbcanfd_module
from USBCANFD_gai import USBCANFD as LinuxUSBCANFD
# 优先导入前面转换好的 Linux 版 7 电机 GUI 控制器。
import GUIyemian_7motors_linux as _gui_module
# 关键：GUIyemian_7motors_linux.py 内部原本可能导入 USBCANFD_fix/USBCANFD。
# 这里强制替换为 USBCANFD_gai.USBCANFD，避免两个 ArmController 同时 open_device 时互相冲突。
_gui_module.USBCANFD = LinuxUSBCANFD

ArmController = _gui_module.ArmController
MODE_MIT = _gui_module.MODE_MIT
MODE_PV = _gui_module.MODE_PV

# =========================
# 1. CANFD 序列号配置
# =========================

# 运行 scan_usbcanfd_devices.py 或本脚本自带扫描后，根据输出确认主从对应关系。
MASTER_SERIAL = "9820ECA9B0E80D6418B0"   # 白色机械臂：主端
SLAVE_SERIAL = "EBD6C68A50FF0DD4F0B0"    # 黑色机械臂：从端

DEVICE_SCAN_MAX = 3
CHANNEL_INDEX = 0
DEVICE_TYPE = int(getattr(getattr(LinuxUSBCANFD, "DEFAULT_DEVICE_TYPE", 43), "value", getattr(LinuxUSBCANFD, "DEFAULT_DEVICE_TYPE", 43)))

# =========================
# 2. 遥操作参数
# =========================

CONTROL_HZ = 800.0
PV_VEL_LIM = 1.0

# 第一次实验建议先小一点，例如 0.005~0.02
MAX_DELTA_PER_CYCLE = 0.02

# 低通滤波系数
ALPHA = 0.20

# 如果某个关节方向相反，把对应项改成 -1。
# 这里只对应前 6 个机械臂关节，保持原功能不变。
SCALE = np.array([1, 1, 1, 1, 1, 1], dtype=float)

# =========================
# 2.1 第 7 个工具电机遥操作参数
# =========================

ENABLE_TOOL_TELEOP = True
TOOL_SCALE = 2.0
TOOL_MAX_DELTA_PER_CYCLE = 0.02
TOOL_ALPHA = 0.20
TOOL_PV_VEL_LIM = PV_VEL_LIM

# 电机目标夹紧时离边界留一点余量，单位 rad
MOTOR_LIMIT_MARGIN = 0.003

# 每隔多少次循环打印一次夹紧信息，避免刷屏
CLIP_PRINT_INTERVAL = 100
PRINT_CLIP_INFO = True

# 夹紧量超过该阈值时才认为值得打印，单位 rad
CLIP_PRINT_MIN_DELTA = 0.03

# =========================
# 3. Linux CANFD 设备扫描函数
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
# 4. 角度处理函数
# =========================

def wrap_to_pi(q):
    q = np.asarray(q, dtype=float)
    return (q + math.pi) % (2 * math.pi) - math.pi

def angle_diff(q_target, q_now):
    """
    返回 q_target - q_now 的最短角度差，范围 [-pi, pi]。
    注意：这个函数只用于计算差值，不再用于强行包裹绝对目标角。
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

def as_float(x):
    return float(np.asarray(x, dtype=float).reshape(()))

def get_dh(controller):
    snapshot = controller.get_status_snapshot()
    if snapshot is None:
        return None
    return np.array(snapshot["dh_rad"], dtype=float)
# =========================
# 4.1 第 7 个工具电机状态函数
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


def check_7motor_controller(controller, role_name):
    """确认当前导入的 ArmController 版本真正支持 1~7 号电机。"""
    if not hasattr(controller, "all_actuators"):
        raise RuntimeError(
            f"{role_name} 当前导入的 ArmController 不是 7 电机版本。\n"
            "请使用 GUIyemian_7motors_linux.py。"
        )

    sig = inspect.signature(controller.set_pv_target_motor_position)
    if "tool_target_q" not in sig.parameters:
        raise RuntimeError(
            f"{role_name} 的 set_pv_target_motor_position() 不支持 tool_target_q 参数。\n"
            "请使用支持第 7 个工具电机的 Linux GUI 控制器文件。"
        )

# =========================
# 5. 从端安全目标写入
# =========================


def dh_to_motor_near_current_clamped(controller, target_dh, margin=0.003):
    """
    将目标 DH 角转换为前 6 个机械臂电机角。

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

        raw = (target_dh[i] - zero_offset[i]) * ratio[i]
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


def safe_set_slave_target_7d(slave, target_dh, target_tool_q, velocity_lim, tool_velocity_lim, loop_count=0):
    """
    从端安全写入目标：
    1~6 号：DH 角 -> 电机角，必要时夹紧；
    7 号：工具电机角直接映射，必要时夹紧；
    然后统一写入 PV 目标。
    """
    _ = tool_velocity_lim  # 当前 GUI 控制器的接口共用 velocity_lim，这里保留参数便于以后扩展。
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
# 6. 主程序
# =========================


def main():
    devices = scan_canfd_devices(DEVICE_SCAN_MAX, device_type=DEVICE_TYPE, channel_index=CHANNEL_INDEX)

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

    try:
        if ENABLE_TOOL_TELEOP:
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
        # 不用 q_slave_home 作为 PV 目标，因为 q_slave_home 反算成电机角时可能略微越过限位。
        # 这里让从端 1~7 号电机全部先保持当前位置。
        if not slave.switch_mode(MODE_PV, None, PV_VEL_LIM):
            print("[ERR] 从端切换 PV 模式失败")
            return

        q_cmd_last = q_slave_home.copy()
        q_tool_cmd_last = float(q_slave_tool_home) if ENABLE_TOOL_TELEOP else 0.0

        dt = 1.0 / CONTROL_HZ
        loop_count = 0

        print()
        print("开始 Linux 单向同构遥操作")
        print("主端：MIT 重力补偿，可拖动")
        print("从端：PV 模式跟随")
        print("第 7 个工具电机：" + ("启用主从同构跟随" if ENABLE_TOOL_TELEOP else "未启用"))
        print("按 Ctrl+C 停止")
        print()

        while True:
            loop_start = time.time()
            loop_count += 1

            q_master = get_dh(master)
            if q_master is None:
                time.sleep(dt)
                continue

            # 只对“主端相对变化量”做 wrap，不再对从端绝对目标角整体 wrap
            delta_master = angle_diff(q_master, q_master_home)

            # 从端目标 DH 角，保持连续形式
            q_target_raw = q_slave_home + SCALE * delta_master

            # 单周期限幅，防止目标突变
            q_target = limit_delta_continuous(q_target_raw, q_cmd_last, MAX_DELTA_PER_CYCLE)

            # 低通滤波，使从端运动更平滑
            q_target = lowpass_continuous(q_target, q_cmd_last, ALPHA)

            if ENABLE_TOOL_TELEOP:
                q_master_tool = get_tool_position(master)
                if q_master_tool is None:
                    time.sleep(dt)
                    continue

                # 第 7 个工具电机不走 DH，直接用主端工具电机相对变化量映射到从端工具电机。
                delta_tool = as_float(angle_diff(q_master_tool, q_master_tool_home))
                q_tool_target_raw = q_slave_tool_home + TOOL_SCALE * delta_tool

                q_tool_target = limit_delta_continuous(
                    q_tool_target_raw,
                    q_tool_cmd_last,
                    TOOL_MAX_DELTA_PER_CYCLE,
                )
                q_tool_target = lowpass_continuous(q_tool_target, q_tool_cmd_last, TOOL_ALPHA)
                q_tool_target = as_float(q_tool_target)

                ok, q_tool_written = safe_set_slave_target_7d(
                    slave,
                    q_target,
                    q_tool_target,
                    PV_VEL_LIM,
                    TOOL_PV_VEL_LIM,
                    loop_count,
                )
            else:
                ok, q_tool_written = safe_set_slave_target_7d(
                    slave,
                    q_target,
                    q_tool_cmd_last,
                    PV_VEL_LIM,
                    TOOL_PV_VEL_LIM,
                    loop_count,
                )

            if ok:
                q_cmd_last = q_target.copy()
                if ENABLE_TOOL_TELEOP and q_tool_written is not None:
                    q_tool_cmd_last = float(q_tool_written)
            else:
                print("[WARN] 从端目标写入失败，保持当前位置")
                slave.set_pv_hold_current_position(PV_VEL_LIM)

            elapsed = time.time() - loop_start
            time.sleep(max(0.0, dt - elapsed))

    except KeyboardInterrupt:
        print()
        print("收到 Ctrl+C，停止遥操作")

    finally:
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
