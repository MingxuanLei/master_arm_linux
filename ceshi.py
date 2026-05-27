#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import threading
import math
import queue
import sys
import os
from ctypes import *

# 导入官方驱动模块（USBCANFD_DEMO.py 必须位于同一目录）
sys.path.insert(0, os.path.dirname(__file__))
from USBCANFD_DEMO import lib, ZCAN_FD_MSG, ZCAN_20_MSG, ZCANFD_INIT


# =========================
# ctypes 函数声明
# =========================

lib.VCI_OpenDevice.argtypes = [c_uint32, c_uint32, c_uint32]
lib.VCI_OpenDevice.restype = c_uint32

lib.VCI_CloseDevice.argtypes = [c_uint32, c_uint32]
lib.VCI_CloseDevice.restype = c_uint32

lib.VCI_InitCAN.argtypes = [c_uint32, c_uint32, c_uint32, POINTER(ZCANFD_INIT)]
lib.VCI_InitCAN.restype = c_uint32

lib.VCI_StartCAN.argtypes = [c_uint32, c_uint32, c_uint32]
lib.VCI_StartCAN.restype = c_uint32

lib.VCI_ResetCAN.argtypes = [c_uint32, c_uint32, c_uint32]
lib.VCI_ResetCAN.restype = c_uint32

lib.VCI_SetReference.argtypes = [c_uint32, c_uint32, c_uint32, c_uint32, c_void_p]
lib.VCI_SetReference.restype = c_uint32

lib.VCI_GetReceiveNum.argtypes = [c_uint32, c_uint32, c_uint32]
lib.VCI_GetReceiveNum.restype = c_uint32

lib.VCI_TransmitFD.argtypes = [c_uint32, c_uint32, c_uint32, POINTER(ZCAN_FD_MSG), c_uint32]
lib.VCI_TransmitFD.restype = c_uint32

lib.VCI_ReceiveFD.argtypes = [c_uint32, c_uint32, c_uint32, POINTER(ZCAN_FD_MSG), c_uint32, c_int]
lib.VCI_ReceiveFD.restype = c_uint32

lib.VCI_Receive.argtypes = [c_uint32, c_uint32, c_uint32, POINTER(ZCAN_20_MSG), c_uint32, c_int]
lib.VCI_Receive.restype = c_uint32


# =========================
# 设备参数
# =========================

DEVICE_TYPE = 43
DEVICE_INDEX = 0
CAN_CHANNEL = 0

# 使能/失能命令
ENABLE_DATA = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
DISABLE_DATA = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]

# CANFD 是否使用 BRS 加速。
# 如果你发现打开 BRS 后电机不响应，可以改为 False。
CANFD_BRS = True

# 是否打印收到的前若干帧。调通后建议改为 False，避免终端刷屏。
DEBUG_PRINT_RX = True
MAX_DEBUG_RX_PRINT = 30
MAX_RX_BATCH = 200

ENABLE_CONFIRM_TIMEOUT = 1.0


# =========================
# 电机参数
# =========================

MOTOR1_CONFIG = {
    "id": 1,
    "can_id": 0x01,
    "feedback_id": 0x11,
    "P_MAX": 12.5,
    "V_MAX": 30.0,
    "T_MAX": 10.0,
    "wave_amplitude": 0.8,
}

MOTOR2_CONFIG = {
    "id": 2,
    "can_id": 0x02,
    "feedback_id": 0x12,
    "P_MAX": 12.5,
    "V_MAX": 10.0,
    "T_MAX": 28.0,
    "wave_amplitude": 3.0,
}

MOTORS = [MOTOR1_CONFIG, MOTOR2_CONFIG]

WAVE_PERIOD = 10.0
SEND_FREQ = 500.0
RUN_DURATION = 10

thread_flag = True
cmd_queue = queue.Queue()

motor_status = {
    1: {
        "motor_id": 1,
        "status_str": "未知",
        "position_rad": 0.0,
        "velocity_rps": 0.0,
        "torque_nm": 0.0,
        "mos_temp_c": 0,
        "coil_temp_c": 0,
        "raw_hex": "",
    },
    2: {
        "motor_id": 2,
        "status_str": "未知",
        "position_rad": 0.0,
        "velocity_rps": 0.0,
        "torque_nm": 0.0,
        "mos_temp_c": 0,
        "coil_temp_c": 0,
        "raw_hex": "",
    },
}

status_lock = threading.Lock()


# =========================
# 协议解析与打包
# =========================

def parse_damiao_feedback(data_bytes, p_max, v_max, t_max):
    """解析达妙电机 MIT 模式反馈报文。"""
    if len(data_bytes) < 8:
        return {"error": "数据长度不足8字节", "raw_hex": data_bytes.hex()}

    byte0 = data_bytes[0]
    motor_id = byte0 & 0x0F
    err_code = (byte0 >> 4) & 0x0F

    err_map = {
        0: "失能",
        1: "使能",
        8: "超压",
        9: "欠压",
        10: "过电流",
        11: "MOS过温",
        12: "电机线圈过温",
        13: "通讯丢失",
        14: "过载",
    }
    status_str = err_map.get(err_code, f"未知({err_code})")

    pos_raw = (data_bytes[1] << 8) | data_bytes[2]
    position_rad = (pos_raw / 65535.0) * (2.0 * p_max) - p_max

    vel_raw = ((data_bytes[3] << 4) | (data_bytes[4] >> 4)) & 0x0FFF
    velocity_rps = (vel_raw / 4095.0) * (2.0 * v_max) - v_max

    t_raw = ((data_bytes[4] & 0x0F) << 8) | data_bytes[5]
    torque_nm = (t_raw / 4095.0) * (2.0 * t_max) - t_max

    mos_temp = data_bytes[6]
    coil_temp = data_bytes[7]

    return {
        "motor_id": motor_id,
        "status_str": status_str,
        "position_rad": round(position_rad, 4),
        "velocity_rps": round(velocity_rps, 4),
        "torque_nm": round(torque_nm, 4),
        "mos_temp_c": mos_temp,
        "coil_temp_c": coil_temp,
        "raw_hex": data_bytes.hex(),
    }


def pack_mit_torque_frame(torque_nm, p_max, v_max, t_max):
    """打包力矩指令为 8 字节 MIT 模式控制帧。"""
    p_int = int((0.0 - (-p_max)) / (2.0 * p_max) * 65535)
    v_int = int((0.0 - (-v_max)) / (2.0 * v_max) * 4095)
    kp_int = 0
    kd_int = 0

    t_clamped = max(min(torque_nm, t_max), -t_max)
    t_int = int((t_clamped + t_max) / (2.0 * t_max) * 4095)

    data = [0] * 8
    data[0] = (p_int >> 8) & 0xFF
    data[1] = p_int & 0xFF
    data[2] = (v_int >> 4) & 0xFF
    data[3] = ((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F)
    data[4] = kp_int & 0xFF
    data[5] = (kd_int >> 4) & 0xFF
    data[6] = ((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F)
    data[7] = t_int & 0xFF
    return data


# =========================
# CANFD 初始化与发送
# =========================

def get_canfd_init_params():
    """仲裁域 1 Mbps，数据域 5 Mbps，采样点约 75%。"""
    canfd_init = ZCANFD_INIT()
    canfd_init.clk = 60000000
    canfd_init.mode = 0

    # 仲裁域 1 Mbps
    canfd_init.abit.tseg1 = 43
    canfd_init.abit.tseg2 = 14
    canfd_init.abit.sjw = 1
    canfd_init.abit.smp = 0
    canfd_init.abit.brp = 0

    # 数据域 5 Mbps
    canfd_init.dbit.tseg1 = 1
    canfd_init.dbit.tseg2 = 0
    canfd_init.dbit.sjw = 1
    canfd_init.dbit.smp = 0
    canfd_init.dbit.brp = 2

    return canfd_init


def init_can_device():
    ret = lib.VCI_OpenDevice(DEVICE_TYPE, DEVICE_INDEX, 0)
    if ret == 0:
        raise RuntimeError("打开设备失败")
    print("设备打开成功")

    canfd_init = get_canfd_init_params()
    ret = lib.VCI_InitCAN(DEVICE_TYPE, DEVICE_INDEX, CAN_CHANNEL, byref(canfd_init))
    if ret == 0:
        raise RuntimeError("初始化 CAN 通道失败")
    print(f"通道 {CAN_CHANNEL} 初始化成功 (波特率: 仲裁域1Mbps, 数据域5Mbps)")

    # 开启终端电阻
    res = c_ubyte(1)
    ret = lib.VCI_SetReference(DEVICE_TYPE, DEVICE_INDEX, CAN_CHANNEL, 0x18, byref(res))
    if ret == 0:
        print("[警告] 终端电阻设置失败，请检查是否需要手动接入120Ω终端电阻")

    ret = lib.VCI_StartCAN(DEVICE_TYPE, DEVICE_INDEX, CAN_CHANNEL)
    if ret == 0:
        raise RuntimeError("启动 CAN 通道失败")
    print("CAN 通道已启动")

    return DEVICE_TYPE, DEVICE_INDEX, CAN_CHANNEL


def send_canfd_frame(dev_type, dev_idx, chn_idx, can_id, data_bytes, brs=CANFD_BRS):
    """发送 CANFD 数据帧。"""
    if len(data_bytes) > 64:
        raise ValueError("CANFD 最大数据长度为 64 字节")

    msg = ZCAN_FD_MSG()
    msg.hdr.id = can_id
    msg.hdr.chn = chn_idx
    msg.hdr.len = len(data_bytes)

    msg.hdr.inf.txm = 0          # 0 = 正常发送
    msg.hdr.inf.fmt = 1          # 1 = CANFD
    msg.hdr.inf.sdf = 0          # 0 = 数据帧
    msg.hdr.inf.sef = 0          # 0 = 标准帧
    msg.hdr.inf.brs = 1 if brs else 0
    msg.hdr.inf.echo = 0
    msg.hdr.inf.qsend = 0
    msg.hdr.inf.qsend_100us = 0

    for i, b in enumerate(data_bytes):
        msg.dat[i] = b

    ret = lib.VCI_TransmitFD(dev_type, dev_idx, chn_idx, byref(msg), 1)
    if ret != 1:
        print(f"[发送警告] ID=0x{can_id:X}, ret={ret}, data={bytes(data_bytes).hex()}")
    return ret


# =========================
# 接收处理
# =========================

def update_motor_status_from_frame(can_id, data_bytes):
    """
    根据反馈 CAN ID 或数据第 0 字节低 4 位识别电机，并更新 motor_status。
    """
    cfg = None

    # 优先根据反馈帧 ID 识别电机，如 0x11 / 0x12
    for motor in MOTORS:
        if can_id == motor["feedback_id"]:
            cfg = motor
            break

    # 如果反馈 ID 不匹配，则根据 data[0] 低 4 位尝试识别电机编号
    if cfg is None and len(data_bytes) >= 1:
        frame_motor_id = data_bytes[0] & 0x0F
        for motor in MOTORS:
            if frame_motor_id == motor["id"]:
                cfg = motor
                break

    if cfg is None:
        if DEBUG_PRINT_RX:
            print(f"[接收线程] 未识别反馈帧 ID=0x{can_id:X}, data={data_bytes.hex()}")
        return

    parsed = parse_damiao_feedback(
        data_bytes[:8],
        cfg["P_MAX"],
        cfg["V_MAX"],
        cfg["T_MAX"],
    )

    with status_lock:
        motor_status[cfg["id"]].update(parsed)


def receiver_thread(dev_type, dev_idx, chn_idx):
    """
    接收线程：
    1. 普通 CAN 报文数量使用 chn_idx 查询，并用 VCI_Receive 读取；
    2. CANFD 报文数量使用 0x80000000 + chn_idx 查询，并用 VCI_ReceiveFD 读取；
    3. 传 ctypes 数组时直接传 can_data / canfd_data，不要 byref(array)。
    """
    global thread_flag

    print("[接收线程] 启动")
    debug_print_count = 0

    while thread_flag:
        try:
            # ---------- 普通 CAN 接收 ----------
            can_count = lib.VCI_GetReceiveNum(dev_type, dev_idx, chn_idx)
            if can_count > 0:
                read_count = min(can_count, MAX_RX_BATCH)
                can_data = (ZCAN_20_MSG * read_count)()

                # 关键：这里直接传 can_data，不要 byref(can_data)
                rcount = lib.VCI_Receive(dev_type, dev_idx, chn_idx, can_data, read_count, 10)

                for i in range(rcount):
                    # 如果是发送回显帧，则跳过
                    if can_data[i].hdr.inf.tx == 1:
                        continue

                    can_id = can_data[i].hdr.id & 0x1FFFFFFF
                    data_len = min(can_data[i].hdr.len, 8)
                    data_bytes = bytes(can_data[i].dat[:data_len])

                    if DEBUG_PRINT_RX and debug_print_count < MAX_DEBUG_RX_PRINT:
                        print(f"[CAN RX] ID=0x{can_id:X}, len={data_len}, data={data_bytes.hex()}")
                        debug_print_count += 1

                    if data_len >= 8:
                        update_motor_status_from_frame(can_id, data_bytes)

            # ---------- CANFD 接收 ----------
            fd_count = lib.VCI_GetReceiveNum(dev_type, dev_idx, 0x80000000 + chn_idx)
            if fd_count > 0:
                read_count = min(fd_count, MAX_RX_BATCH)
                canfd_data = (ZCAN_FD_MSG * read_count)()

                # 关键：这里直接传 canfd_data，不要 byref(canfd_data)
                rcount = lib.VCI_ReceiveFD(dev_type, dev_idx, chn_idx, canfd_data, read_count, 10)

                for i in range(rcount):
                    # 如果是发送回显帧，则跳过
                    if canfd_data[i].hdr.inf.tx == 1:
                        continue

                    can_id = canfd_data[i].hdr.id & 0x1FFFFFFF
                    data_len = min(canfd_data[i].hdr.len, 64)
                    data_bytes = bytes(canfd_data[i].dat[:data_len])

                    if DEBUG_PRINT_RX and debug_print_count < MAX_DEBUG_RX_PRINT:
                        print(f"[CANFD RX] ID=0x{can_id:X}, len={data_len}, data={data_bytes.hex()}")
                        debug_print_count += 1

                    if data_len >= 8:
                        update_motor_status_from_frame(can_id, data_bytes[:8])

            time.sleep(0.001)

        except Exception as e:
            print(f"[接收线程] 异常: {e}")
            time.sleep(0.05)

    print("[接收线程] 退出")


# =========================
# 发送线程与主流程
# =========================

def sender_thread(dev_type, dev_idx, chn_idx):
    """发送线程：使能 -> 从队列取力矩指令 -> 失能。"""
    global thread_flag, cmd_queue

    print("[发送线程] 发送使能命令...")
    for motor in MOTORS:
        send_canfd_frame(dev_type, dev_idx, chn_idx, motor["can_id"], ENABLE_DATA)
        time.sleep(0.002)

    timeout_start = time.time()
    both_enabled = False

    while time.time() - timeout_start < ENABLE_CONFIRM_TIMEOUT:
        if not thread_flag:
            return

        with status_lock:
            s1 = motor_status[1]["status_str"]
            s2 = motor_status[2]["status_str"]

        if s1 == "使能" and s2 == "使能":
            both_enabled = True
            break

        time.sleep(0.02)

    if both_enabled:
        print("[发送线程] 两个电机均已使能")
    else:
        print("[发送线程] 警告：未检测到全部电机使能，将继续发送指令")

    while thread_flag:
        try:
            cmd = cmd_queue.get(timeout=0.1)
            send_canfd_frame(dev_type, dev_idx, chn_idx, cmd["can_id"], cmd["data"])

            while not cmd_queue.empty():
                try:
                    cmd = cmd_queue.get_nowait()
                    send_canfd_frame(dev_type, dev_idx, chn_idx, cmd["can_id"], cmd["data"])
                except queue.Empty:
                    break

        except queue.Empty:
            continue

    print("[发送线程] 发送失能命令...")
    for motor in MOTORS:
        send_canfd_frame(dev_type, dev_idx, chn_idx, motor["can_id"], DISABLE_DATA)
        time.sleep(0.002)

    print("[发送线程] 失能命令已发送，线程退出")


def print_motor_status(elapsed):
    with status_lock:
        s1 = motor_status[1].copy()
        s2 = motor_status[2].copy()

    print(f"\n[时间 {elapsed:5.1f}s]")
    print(
        f"  电机1: 状态={s1['status_str']:4s}, "
        f"位置={s1['position_rad']:7.4f} rad, "
        f"速度={s1['velocity_rps']:7.3f} rps, "
        f"力矩={s1['torque_nm']:7.4f} Nm, "
        f"MOS={s1['mos_temp_c']:3d}℃, "
        f"线圈={s1['coil_temp_c']:3d}℃, "
        f"raw={s1.get('raw_hex', '')}"
    )
    print(
        f"  电机2: 状态={s2['status_str']:4s}, "
        f"位置={s2['position_rad']:7.4f} rad, "
        f"速度={s2['velocity_rps']:7.3f} rps, "
        f"力矩={s2['torque_nm']:7.4f} Nm, "
        f"MOS={s2['mos_temp_c']:3d}℃, "
        f"线圈={s2['coil_temp_c']:3d}℃, "
        f"raw={s2.get('raw_hex', '')}"
    )


def main():
    global thread_flag

    dev_type = dev_idx = chn_idx = None
    recv_thread = None
    send_thread = None

    try:
        dev_type, dev_idx, chn_idx = init_can_device()

        recv_thread = threading.Thread(
            target=receiver_thread,
            args=(dev_type, dev_idx, chn_idx),
            daemon=True,
        )
        recv_thread.start()

        send_thread = threading.Thread(
            target=sender_thread,
            args=(dev_type, dev_idx, chn_idx),
            daemon=False,
        )
        send_thread.start()

        print("\n===== 开始双电机力矩波形控制 =====")
        print(f"电机1: 振幅 {MOTOR1_CONFIG['wave_amplitude']} Nm, 周期 {WAVE_PERIOD}s")
        print(f"电机2: 振幅 {MOTOR2_CONFIG['wave_amplitude']} Nm, 周期 {WAVE_PERIOD}s")
        print(f"运行时间: {RUN_DURATION}s, 发送频率 {SEND_FREQ}Hz")

        start_time = time.perf_counter()
        last_print_time = start_time
        period_sec = 1.0 / SEND_FREQ
        next_tick = start_time + period_sec

        while time.perf_counter() - start_time < RUN_DURATION:
            current = time.perf_counter()
            elapsed = current - start_time

            for motor in MOTORS:
                amp = motor["wave_amplitude"]
                torque_cmd = amp * math.sin(2.0 * math.pi * elapsed / WAVE_PERIOD)
                frame_data = pack_mit_torque_frame(
                    torque_cmd,
                    motor["P_MAX"],
                    motor["V_MAX"],
                    motor["T_MAX"],
                )
                cmd_queue.put({"can_id": motor["can_id"], "data": frame_data})

            if current - last_print_time >= 0.1:
                print_motor_status(elapsed)
                last_print_time = current

            next_tick += period_sec
            sleep_until = next_tick - time.perf_counter()

            if sleep_until > 0:
                time.sleep(sleep_until * 0.8)
                while time.perf_counter() < next_tick:
                    pass
            else:
                next_tick = time.perf_counter() + period_sec

        print("\n程序运行时间已到，正在停止...")

    except KeyboardInterrupt:
        print("\n用户手动中断，正在退出...")

    except Exception as e:
        print(f"\n[主线程] 异常: {e}")

    finally:
        thread_flag = False

        if send_thread is not None:
            send_thread.join(timeout=3.0)

        if recv_thread is not None:
            recv_thread.join(timeout=1.0)

        if dev_type is not None and dev_idx is not None and chn_idx is not None:
            lib.VCI_ResetCAN(dev_type, dev_idx, chn_idx)
            lib.VCI_CloseDevice(dev_type, dev_idx)
            print("CAN 设备已关闭")


if __name__ == "__main__":
    main()