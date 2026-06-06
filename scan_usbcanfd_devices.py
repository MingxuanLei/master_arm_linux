#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scan_usbcanfd_devices.py

Linux 下扫描 ZLG / USBCANFD 设备索引的小工具。

用途：
    逐个尝试打开 device_index=0,1,2,...，读取设备序列号、硬件类型、通道数等信息，
    方便你确认哪一台机械臂对应哪个 device_index。

依赖：
    1. USBCANFD_DEMO.py
    2. libusbcanfd.so

建议放置目录：
    scan_usbcanfd_devices.py
    USBCANFD_DEMO.py
    libusbcanfd.so

运行：
    python3 scan_usbcanfd_devices.py
    python3 scan_usbcanfd_devices.py --max-index 8
    python3 scan_usbcanfd_devices.py --device-type 43 --max-index 16
"""

from __future__ import annotations

from ctypes import (
    POINTER,
    Structure,
    byref,
    c_char,
    c_int,
    c_uint,
    c_uint32,
    c_uint8,
    c_ubyte,
    c_ushort,
    c_void_p,
    create_string_buffer,
)
from contextlib import contextmanager
from pathlib import Path
import argparse
import importlib
import importlib.util
import os
import sys
import time


# =========================
# 默认参数
# =========================

# 你前面已经验证可用的设备类型：ZCAN_USBCANFD_MINI = 43
DEFAULT_DEVICE_TYPE = 43
DEFAULT_MAX_INDEX = 3
DEFAULT_CHANNEL_INDEX = 0

# USBCANFD_DEMO.py 里的定义：
# CMD_SET_SN = 0x42   # 注释里写“获取SN号”
# CMD_GET_SN = 0x43   # 注释里写“设置SN号”
# 不同版本注释可能有误，所以脚本会两个都尝试。
CMD_SET_SN = 0x42
CMD_GET_SN = 0x43


# =========================
# 导入 USBCANFD_DEMO.py
# =========================

@contextmanager
def temporary_chdir(path: Path):
    old = Path.cwd()
    os.chdir(str(path))
    try:
        yield
    finally:
        os.chdir(str(old))


def load_usbcanfd_demo_module():
    """
    加载 USBCANFD_DEMO.py。

    官方 demo 里通常是：
        lib = cdll.LoadLibrary("./libusbcanfd.so")

    所以如果当前工作目录不是脚本所在目录，直接 import 可能失败。
    这里先切换到脚本所在目录再加载，保证能找到 libusbcanfd.so。
    """
    script_dir = Path(__file__).resolve().parent
    demo_path = script_dir / "USBCANFD_DEMO.py"

    if not demo_path.exists():
        # 允许当前 Python 路径中已有 USBCANFD_DEMO
        try:
            return importlib.import_module("USBCANFD_DEMO")
        except Exception as exc:
            raise FileNotFoundError(
                "找不到 USBCANFD_DEMO.py。请把 scan_usbcanfd_devices.py、USBCANFD_DEMO.py、"
                "libusbcanfd.so 放在同一目录。"
            ) from exc

    sys.modules.pop("USBCANFD_DEMO", None)
    with temporary_chdir(script_dir):
        spec = importlib.util.spec_from_file_location("USBCANFD_DEMO", demo_path)
        if spec is None or spec.loader is None:
            raise ImportError(f"无法加载 {demo_path}")
        module = importlib.util.module_from_spec(spec)
        sys.modules["USBCANFD_DEMO"] = module
        spec.loader.exec_module(module)
        return module


_demo = load_usbcanfd_demo_module()
lib = _demo.lib


# =========================
# ctypes 函数声明
# =========================

def configure_ctypes() -> None:
    lib.VCI_OpenDevice.argtypes = [c_uint32, c_uint32, c_uint32]
    lib.VCI_OpenDevice.restype = c_uint32

    lib.VCI_CloseDevice.argtypes = [c_uint32, c_uint32]
    lib.VCI_CloseDevice.restype = c_uint32

    # 有些驱动库提供 VCI_ReadBoardInfo，有些不提供。这里按需配置。
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


configure_ctypes()


# =========================
# 设备信息结构体
# =========================

class VCI_BOARD_INFO(Structure):
    """
    常见 VCI 设备信息结构体。

    不同版本驱动的字段名可能略有不同，但 ZLG 常见库的布局与这个结构基本一致：
    hw_Version / fw_Version / dr_Version / in_Version / irq_Num / can_Num /
    str_Serial_Num[20] / str_hw_Type[40] / Reserved[4]
    """
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


def decode_c_ubyte_array(arr) -> str:
    values = []
    for x in arr:
        v = int(x)
        if v == 0:
            break
        values.append(v)
    return bytes(values).decode("ascii", errors="ignore")


def format_version(v: int) -> str:
    """尽量按 ZLG 常见显示习惯格式化版本号。"""
    v = int(v)
    hi = (v >> 8) & 0xFF
    lo = v & 0xFF
    return f"V{hi}.{lo:02X}"


def read_board_info(device_type: int, device_index: int) -> dict | None:
    """优先用 VCI_ReadBoardInfo 读取完整设备信息。"""
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
        "serial": decode_c_ubyte_array(info.str_Serial_Num),
        "hw_type": decode_c_ubyte_array(info.str_hw_Type),
        "can_num": int(info.can_Num),
        "hw_version": format_version(info.hw_Version),
        "fw_version": format_version(info.fw_Version),
        "dr_version": format_version(info.dr_Version),
        "in_version": format_version(info.in_Version),
        "irq_num": int(info.irq_Num),
        "source": "VCI_ReadBoardInfo",
    }


def _try_get_reference_string(device_type: int, device_index: int, channel_index: int, cmd: int, size: int = 128) -> str | None:
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

    raw = bytes(buf.raw)
    raw = raw.split(b"\x00", 1)[0]
    text = raw.decode("ascii", errors="ignore").strip()
    return text or None


def read_serial_by_reference(device_type: int, device_index: int, channel_index: int) -> tuple[str | None, str | None]:
    """
    用 VCI_GetReference 尝试读取 SN。

    由于 USBCANFD_DEMO.py 中 CMD_SET_SN/CMD_GET_SN 的中文注释可能写反，
    这里会依次尝试 0x43 和 0x42，哪个能读到可显示字符串就用哪个。
    """
    for cmd_name, cmd in (("CMD_GET_SN(0x43)", CMD_GET_SN), ("CMD_SET_SN(0x42)", CMD_SET_SN)):
        text = _try_get_reference_string(device_type, device_index, channel_index, cmd)
        if text:
            return text, cmd_name
    return None, None


def scan_devices(max_index: int = DEFAULT_MAX_INDEX, device_type: int = DEFAULT_DEVICE_TYPE, channel_index: int = DEFAULT_CHANNEL_INDEX) -> list[dict]:
    print("开始扫描 CANFD 设备...")
    print(f"device_type={device_type}, max_index={max_index}, channel_index={channel_index}")
    print("=" * 72)

    found: list[dict] = []

    for idx in range(int(max_index)):
        try:
            ret = lib.VCI_OpenDevice(c_uint32(device_type), c_uint32(idx), c_uint32(0))
        except Exception as exc:
            print(f"device_index={idx}: 打开异常: {exc}")
            continue

        if int(ret) == 0:
            print(f"device_index={idx}: 打开失败")
            continue

        print(f"device_index={idx}: 打开成功")
        print(f"  open_ret   = {int(ret)}")

        info = read_board_info(device_type, idx)
        serial_from_ref, ref_source = read_serial_by_reference(device_type, idx, channel_index)

        if info is not None:
            if serial_from_ref and not info.get("serial"):
                info["serial"] = serial_from_ref

            print(f"  serial     = {info.get('serial', '')}")
            print(f"  hw_type    = {info.get('hw_type', '')}")
            print(f"  can_num    = {info.get('can_num', '')}")
            print(f"  hw_version = {info.get('hw_version', '')}")
            print(f"  fw_version = {info.get('fw_version', '')}")
            print(f"  info_src   = {info.get('source', '')}")
            if serial_from_ref:
                print(f"  sn_ref     = {serial_from_ref} ({ref_source})")

            item = {
                "device_index": idx,
                "serial": info.get("serial", ""),
                "hw_type": info.get("hw_type", ""),
                "can_num": info.get("can_num", None),
                "hw_version": info.get("hw_version", ""),
                "fw_version": info.get("fw_version", ""),
            }
        else:
            print("  读取完整设备信息失败")
            if serial_from_ref:
                print(f"  serial     = {serial_from_ref} ({ref_source})")
            else:
                print("  serial     = 未读取到")

            item = {
                "device_index": idx,
                "serial": serial_from_ref or "",
                "hw_type": "",
                "can_num": None,
                "hw_version": "",
                "fw_version": "",
            }

        found.append(item)

        try:
            close_ret = lib.VCI_CloseDevice(c_uint32(device_type), c_uint32(idx))
            print(f"  close_ret  = {int(close_ret)}")
        except Exception as exc:
            print(f"  关闭设备异常: {exc}")

        # 给 USB 设备一点释放时间，避免连续扫描时驱动还没释放完。
        time.sleep(0.05)

    print("=" * 72)
    print(f"共发现 {len(found)} 个可打开的 CANFD 设备")

    for item in found:
        print(
            f"device_index={item['device_index']}, "
            f"serial={item.get('serial', '')}, "
            f"hw_type={item.get('hw_type', '')}, "
            f"can_num={item.get('can_num', '')}"
        )

    return found


def main() -> None:
    parser = argparse.ArgumentParser(description="Linux 下扫描 USBCANFD 设备索引和序列号")
    parser.add_argument("--max-index", type=int, default=DEFAULT_MAX_INDEX, help="最大扫描索引数量，默认 8，即扫描 0~7")
    parser.add_argument("--device-type", type=int, default=DEFAULT_DEVICE_TYPE, help="设备类型，默认 43=ZCAN_USBCANFD_MINI")
    parser.add_argument("--channel", type=int, default=DEFAULT_CHANNEL_INDEX, help="读取 GetReference 时使用的通道号，默认 0")
    args = parser.parse_args()

    scan_devices(max_index=args.max_index, device_type=args.device_type, channel_index=args.channel)


if __name__ == "__main__":
    main()
