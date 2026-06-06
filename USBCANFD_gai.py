"""
Linux version of the USBCANFD motor-control wrapper.

This file keeps the public interface of the Windows USBCANFD.py as much as
possible, but replaces the zlgcan.dll/ZCAN handle API with the Linux
libusbcanfd.so / VCI_* API used by USBCANFD_DEMO.py.

Expected files in the same directory:
    - DMMotor.py
    - USBCANFD_DEMO.py
    - libusbcanfd.so

Typical usage:
    from USBCANFD import USBCANFD

    can = USBCANFD(device_index=0, channel_index=0)
    can.open_device()
    can.init_device()
    can.start_device()
    can.get_status_all()
    can.enable_all()
    can.start_can_thread(1)   # 1 = CANFD receive thread, continuous CANFD command send thread

Multiple devices/channels:
    can0 = USBCANFD(device_index=0, channel_index=0)
    can1 = USBCANFD(device_index=1, channel_index=0)
    # If one adapter has two channels, use the same device_index and different channel_index:
    can_ch0 = USBCANFD(device_index=0, channel_index=0)
    can_ch1 = USBCANFD(device_index=0, channel_index=1)

Notes:
    1. This module imports structures/constants from USBCANFD_DEMO.py.
    2. USBCANFD_DEMO.py loads ./libusbcanfd.so. For robustness, this file
       tries to import USBCANFD_DEMO from the same directory as this file.
    3. DMMotor.py is still responsible for packing/unpacking MIT/PV/PVT
       motor commands and feedback.
    4. The default device type and bitrate are copied from the verified
       ceshi.py: DEVICE_TYPE=43, arbitration 1 Mbps, data 5 Mbps.
"""

from __future__ import annotations

from contextlib import contextmanager
from ctypes import POINTER, byref, c_int, c_uint32, c_uint8, c_void_p, sizeof, memset
import importlib
import importlib.util
import math
import os
from pathlib import Path
import sys
import threading
import time
from typing import Optional, Sequence

from DMMotor import DMMotor


@contextmanager
def _temporary_chdir(path: Path):
    old = Path.cwd()
    os.chdir(str(path))
    try:
        yield
    finally:
        os.chdir(str(old))


def _load_usbcanfd_demo_module():
    """
    Load USBCANFD_DEMO.py.

    The vendor demo uses cdll.LoadLibrary("./libusbcanfd.so"), so importing it
    only works when the current working directory contains libusbcanfd.so.
    This helper first tries normal import, then retries from this file's folder.
    """
    try:
        return importlib.import_module("USBCANFD_DEMO")
    except Exception as first_exc:
        sys.modules.pop("USBCANFD_DEMO", None)
        module_dir = Path(__file__).resolve().parent
        demo_path = module_dir / "USBCANFD_DEMO.py"
        if not demo_path.exists():
            raise ImportError(
                "找不到 USBCANFD_DEMO.py。请把 USBCANFD_DEMO.py 与当前 USBCANFD.py 放在同一目录。"
            ) from first_exc

        try:
            with _temporary_chdir(module_dir):
                spec = importlib.util.spec_from_file_location("USBCANFD_DEMO", demo_path)
                if spec is None or spec.loader is None:
                    raise ImportError(f"无法加载 {demo_path}")
                module = importlib.util.module_from_spec(spec)
                sys.modules["USBCANFD_DEMO"] = module
                spec.loader.exec_module(module)
                return module
        except Exception as second_exc:
            raise ImportError(
                "导入 USBCANFD_DEMO.py 失败。请确认当前目录或 USBCANFD.py 所在目录下存在 libusbcanfd.so，"
                "并且 Linux 下已经安装好 USBCANFD 驱动/动态库依赖。"
            ) from second_exc


_demo = _load_usbcanfd_demo_module()


def _configure_vci_ctypes() -> None:
    """Configure libusbcanfd.so function prototypes as used in ceshi.py."""
    lib = _demo.lib
    lib.VCI_OpenDevice.argtypes = [c_uint32, c_uint32, c_uint32]
    lib.VCI_OpenDevice.restype = c_uint32

    lib.VCI_CloseDevice.argtypes = [c_uint32, c_uint32]
    lib.VCI_CloseDevice.restype = c_uint32

    lib.VCI_InitCAN.argtypes = [c_uint32, c_uint32, c_uint32, POINTER(_demo.ZCANFD_INIT)]
    lib.VCI_InitCAN.restype = c_uint32

    lib.VCI_StartCAN.argtypes = [c_uint32, c_uint32, c_uint32]
    lib.VCI_StartCAN.restype = c_uint32

    lib.VCI_ResetCAN.argtypes = [c_uint32, c_uint32, c_uint32]
    lib.VCI_ResetCAN.restype = c_uint32

    lib.VCI_SetReference.argtypes = [c_uint32, c_uint32, c_uint32, c_uint32, c_void_p]
    lib.VCI_SetReference.restype = c_uint32

    lib.VCI_GetReceiveNum.argtypes = [c_uint32, c_uint32, c_uint32]
    lib.VCI_GetReceiveNum.restype = c_uint32

    lib.VCI_TransmitFD.argtypes = [c_uint32, c_uint32, c_uint32, POINTER(_demo.ZCAN_FD_MSG), c_uint32]
    lib.VCI_TransmitFD.restype = c_uint32

    lib.VCI_ReceiveFD.argtypes = [c_uint32, c_uint32, c_uint32, POINTER(_demo.ZCAN_FD_MSG), c_uint32, c_int]
    lib.VCI_ReceiveFD.restype = c_uint32

    lib.VCI_Transmit.argtypes = [c_uint32, c_uint32, c_uint32, POINTER(_demo.ZCAN_20_MSG), c_uint32]
    lib.VCI_Transmit.restype = c_uint32

    lib.VCI_Receive.argtypes = [c_uint32, c_uint32, c_uint32, POINTER(_demo.ZCAN_20_MSG), c_uint32, c_int]
    lib.VCI_Receive.restype = c_uint32


_configure_vci_ctypes()


# =============================================================================
# 多设备打开管理
# =============================================================================
#
# 说明：
# Linux 版 VCI_OpenDevice 是按照 DevType + DevIdx 打开“设备”的，
# 而不是按照通道打开。很多驱动不允许同一个进程对同一个 DevType/DevIdx
# 重复 OpenDevice。为了支持：
#   1) 同时打开 device_index=0、device_index=1、... 多个物理 CANFD 设备；
#   2) 同一个物理设备上的多个通道由多个 USBCANFD 实例使用；
# 这里增加进程内注册表和引用计数：
#   - 第一次打开某个 (device_type, device_index) 时真正调用 VCI_OpenDevice；
#   - 后续相同设备只复用已打开状态，不再次调用 VCI_OpenDevice；
#   - close_device() 时引用计数减 1，最后一个实例关闭时才真正 VCI_CloseDevice。
#
_DEVICE_REGISTRY_LOCK = threading.RLock()
_OPENED_DEVICES: dict[tuple[int, int], dict[str, int]] = {}


def _u32_value(value) -> int:
    """Return the Python int value of a c_uint32/int-like object."""
    return int(value.value) if hasattr(value, "value") else int(value)


class USBCANFD:
    """
    Linux USBCANFD controller.

    The class intentionally preserves the Windows version's main fields and
    methods:
        open_device, close_device, init_device, start_device,
        can_send, canfd_send, start_can_thread, stop_can,
        enable_all, disable_all, set_zero, get_status_all, set_mode_all,
        clearRecvBuffer, setQueueSend, clearQueueSend, setResistanceEnable.

    Parameters
    ----------
    device_index:
        Linux vendor API device index. Usually 0. Use 0/1/2... to open
        multiple physical USBCANFD devices in the same Python process.
    channel_index:
        CAN channel index. Usually 0. If one physical device has multiple
        channels, you can create several USBCANFD instances with the same
        device_index but different channel_index; open_device() will reuse the
        already-opened device and only initialize/start each channel separately.
    device_type:
        Vendor device type. Default is 43, the value verified in your Linux
        ceshi.py. Keep the default unless your hardware model requires another
        type.
    canfd_extended:
        Whether CANFD frames use extended frame format. The Windows wrapper
        effectively sends standard IDs; therefore default is False.
        If your Linux demo/device setup requires extended CANFD frames, create
        USBCANFD(canfd_extended=True).
    canfd_brs:
        Whether CANFD bit-rate switch is enabled.
    queue_pad_ms:
        Hardware queue-send delay. 0 means no additional delay.
    """

    MOTOR_NUM = 6
    TOOL_NUM = 1
    SLAVE_NUM = MOTOR_NUM + TOOL_NUM

    TYPE_CAN = 0
    TYPE_CANFD = 1

    # Device type used by the verified Linux ceshi.py: ZCAN_USBCANFD_MINI = 43.
    # Do not use the older USBCANFD_DEMO.py default value 33 here.
    DEFAULT_DEVICE_TYPE = c_uint32(43)

    def __init__(
        self,
        device_index: int = 0,
        channel_index: int = 0,
        *,
        device_type: Optional[int] = None,
        canfd_extended: bool = False,
        canfd_brs: bool = True,
        queue_pad_ms: int = 0,
    ):
        self.lib = _demo.lib

        # device_type + device_index identify one physical USBCANFD device.
        # Multiple USBCANFD instances can now safely share the same physical
        # device when they use different channels, while different device_index
        # values open different physical devices.
        self.kDeviceType = c_uint32(
            _u32_value(self.DEFAULT_DEVICE_TYPE) if device_type is None else int(device_type)
        )
        self.device_index_ = c_uint32(int(device_index))
        self.channel_index_ = int(channel_index)
        self._device_key = (_u32_value(self.kDeviceType), _u32_value(self.device_index_))
        self._open_registered = False

        # Linux VCI API does not return the same handle objects as zlgcan.
        # These two fields are kept for compatibility with the Windows wrapper.
        self.device_handle_ = 0
        self.channel_handle_ = 0

        self.canfd_extended = bool(canfd_extended)
        self.canfd_brs = bool(canfd_brs)
        self.queue_pad_ms = max(0, int(queue_pad_ms))

        # Default timing copied from the verified ceshi.py:
        # arbitration 1 Mbps, data 5 Mbps, sampling point about 75%, clock 60 MHz.
        self.canfd_clock_hz = 60_000_000
        self.abit_timing = dict(tseg1=43, tseg2=14, sjw=1, smp=0, brp=0)
        self.dbit_timing = dict(tseg1=1, tseg2=0, sjw=1, smp=0, brp=2)
        self.canfd_mode = 0

        self.tool_update = 50
        self.is_open = False
        self.is_started = False
        self.is_updating = False

        self.can_param_time = 200  # ms
        self.can_param = [0.0] * 7
        self.send_suc_num = 0
        self.send_err_num = 0
        self.recv_num = 0
        self.system_update = 0

        self.motors = [DMMotor(i + 1, 4340, "PV") for i in range(3)] + [
            DMMotor(i + 1, 4310, "PV") for i in range(3, 6)
        ]
        self.motors[1].angle_lim = [0.0, 213.0 * math.pi / 180.0]
        self.motors[2].angle_lim = [0.0, 182.0 * math.pi / 180.0]
        self.motors[4].angle_lim = [-84.0 * math.pi / 180.0, 98.0 * math.pi / 180.0]
        self.tools = [DMMotor(7, 3507, "PVT")]

        self.canfd_queue = [
            self._new_canfd_frame(0, b"\x00" * 8, queue=True) for _ in range(self.SLAVE_NUM)
        ]
        for i in range(self.MOTOR_NUM):
            self.canfd_queue[i] = self._new_canfd_frame(
                self.motors[i].ID_OFFSET, self.motors[i].Command, queue=True
            )

        self.canfd_send_data = self._new_canfd_frame(0, b"\x00" * 8, queue=False)

        self.motor_mode = [m.Mode for m in self.motors]
        self.motor_lock = [False] * self.MOTOR_NUM
        self.mode_switch_flag = 0

        self.can_recv_trd: Optional[threading.Thread] = None
        self.can_send_trd: Optional[threading.Thread] = None
        self.can_param_update_trd: Optional[threading.Thread] = None
        self._lock = threading.RLock()

    @property
    def IsOpen(self) -> bool:
        return self.is_open

    @property
    def IsUpdating(self) -> bool:
        return self.is_updating

    @property
    def CanParam(self) -> list[float]:
        with self._lock:
            return list(self.can_param)

    @property
    def Mode(self) -> int:
        if all(v == 1 for v in self.motor_mode):
            return 1
        if all(v == 2 for v in self.motor_mode):
            return 2
        return 0

    @staticmethod
    def _copy_data_to_ctypes(dst, src: Sequence[int], max_len: int) -> None:
        data = list(src)[:max_len]
        for i in range(max_len):
            dst[i] = int(data[i]) & 0xFF if i < len(data) else 0

    def _new_can_frame(
        self,
        can_id: int,
        data: Sequence[int],
        *,
        queue: bool = False,
        extended: Optional[bool] = None,
    ):
        frame = _demo.ZCAN_20_MSG()
        frame.hdr.inf.txm = 0
        frame.hdr.inf.fmt = 0
        frame.hdr.inf.sdf = 0
        frame.hdr.inf.sef = 1 if (self.canfd_extended if extended is None else extended) else 0
        frame.hdr.inf.brs = 0
        frame.hdr.inf.echo = 0
        frame.hdr.inf.qsend = 1 if queue else 0
        frame.hdr.inf.qsend_100us = 0
        frame.hdr.pad = self.queue_pad_ms if queue else 0
        frame.hdr.id = int(can_id) & 0x1FFFFFFF
        frame.hdr.chn = self.channel_index_
        frame.hdr.len = 8
        self._copy_data_to_ctypes(frame.dat, data, 8)
        return frame

    def _new_canfd_frame(
        self,
        can_id: int,
        data: Sequence[int],
        *,
        queue: bool = False,
        extended: Optional[bool] = None,
    ):
        frame = _demo.ZCAN_FD_MSG()
        frame.hdr.inf.txm = 0
        frame.hdr.inf.fmt = 1
        frame.hdr.inf.sdf = 0
        frame.hdr.inf.sef = 1 if (self.canfd_extended if extended is None else extended) else 0
        frame.hdr.inf.brs = 1 if self.canfd_brs else 0
        frame.hdr.inf.echo = 0
        frame.hdr.inf.qsend = 1 if queue else 0
        frame.hdr.inf.qsend_100us = 0
        frame.hdr.pad = self.queue_pad_ms if queue else 0
        frame.hdr.id = int(can_id) & 0x1FFFFFFF
        frame.hdr.chn = self.channel_index_
        frame.hdr.len = 8
        self._copy_data_to_ctypes(frame.dat, data, 64)
        return frame

    def _set_value(self, path: str, value: str | int | bytes) -> bool:
        """
        Compatibility placeholder for the Windows zlgcan ZCAN_SetValue path API.

        Linux libusbcanfd.so mainly uses VCI_SetReference instead of string paths.
        The public methods below map the meaningful operations to VCI_SetReference.
        """
        _ = (path, value)
        return True

    def open_device(self) -> bool:
        """
        Open one physical USBCANFD device.

        Multi-device behavior:
        - Different device_index values, such as 0 and 1, call VCI_OpenDevice
          independently and can run at the same time.
        - Re-opening the same (device_type, device_index) in the same Python
          process will reuse the existing device open and only increase a
          reference count. This is useful when one physical adapter has two
          channels and you create two USBCANFD objects for channel 0 and 1.
        """
        key = self._device_key

        with _DEVICE_REGISTRY_LOCK:
            record = _OPENED_DEVICES.get(key)
            if record is not None:
                record["ref_count"] += 1
                self.device_handle_ = record.get("open_ret", 1)
                self.is_open = True
                self._open_registered = True
                print(
                    f"设备已打开，复用 DevType={key[0]}, DevIdx={key[1]}, "
                    f"ref_count={record['ref_count']}"
                )
                return True

            ret = self.lib.VCI_OpenDevice(self.kDeviceType, self.device_index_, 0)
            if int(ret) == 0:
                print(f"无法打开设备 DevType={key[0]}, DevIdx={key[1]}")
                return False

            _OPENED_DEVICES[key] = {"ref_count": 1, "open_ret": int(ret)}
            self.device_handle_ = int(ret)
            self.is_open = True
            self._open_registered = True
            print(f"设备打开成功 DevType={key[0]}, DevIdx={key[1]}")
            return True

    def close_device(self) -> bool:
        """
        Close this instance.

        The physical device is closed only when the last USBCANFD instance using
        the same (device_type, device_index) has called close_device().
        """
        self.stop_can()

        if self.is_started:
            try:
                self.lib.VCI_ResetCAN(self.kDeviceType, self.device_index_, self.channel_index_)
            except Exception:
                pass
            self.is_started = False

        if self.is_open:
            key = self._device_key
            with _DEVICE_REGISTRY_LOCK:
                record = _OPENED_DEVICES.get(key)

                if record is None:
                    # Defensive fallback: if the registry was cleared externally,
                    # still try to close the physical device once.
                    try:
                        self.lib.VCI_CloseDevice(self.kDeviceType, self.device_index_)
                    except Exception:
                        pass
                else:
                    record["ref_count"] -= 1
                    if record["ref_count"] <= 0:
                        try:
                            self.lib.VCI_CloseDevice(self.kDeviceType, self.device_index_)
                        finally:
                            _OPENED_DEVICES.pop(key, None)
                        print(f"设备已关闭 DevType={key[0]}, DevIdx={key[1]}")
                    else:
                        print(
                            f"设备仍被其他实例使用 DevType={key[0]}, DevIdx={key[1]}, "
                            f"ref_count={record['ref_count']}"
                        )

            self.device_handle_ = 0
            self.channel_handle_ = 0
            self.is_open = False
            self._open_registered = False

        return not self.is_open

    @classmethod
    def opened_device_keys(cls) -> list[tuple[int, int]]:
        """Return currently opened (device_type, device_index) keys."""
        with _DEVICE_REGISTRY_LOCK:
            return list(_OPENED_DEVICES.keys())

    @classmethod
    def open_multiple(cls, configs: Sequence[dict]) -> list["USBCANFD"]:
        """
        Convenience helper to open multiple devices/channels.

        Example:
            cans = USBCANFD.open_multiple([
                {"device_index": 0, "channel_index": 0},
                {"device_index": 1, "channel_index": 0},
            ])
        If any open fails, already-opened instances will be closed.
        """
        controllers: list[USBCANFD] = []
        try:
            for cfg in configs:
                can = cls(**dict(cfg))
                if not can.open_device():
                    raise RuntimeError(f"打开设备失败: {cfg}")
                controllers.append(can)
            return controllers
        except Exception:
            for can in reversed(controllers):
                try:
                    can.close_device()
                except Exception:
                    pass
            raise

    def _build_canfd_init(self):
        canfd_init = _demo.ZCANFD_INIT()
        canfd_init.clk = int(self.canfd_clock_hz)
        canfd_init.mode = int(self.canfd_mode)

        for key, value in self.abit_timing.items():
            setattr(canfd_init.abit, key, int(value))
        for key, value in self.dbit_timing.items():
            setattr(canfd_init.dbit, key, int(value))

        return canfd_init

    def init_device(self) -> bool:
        if not self.is_open:
            print("设备尚未打开，请先调用 open_device()")
            return False

        canfd_init = self._build_canfd_init()
        ret = self.lib.VCI_InitCAN(
            self.kDeviceType, self.device_index_, self.channel_index_, byref(canfd_init)
        )
        if int(ret) == 0:
            print(f"初始化 CAN 通道 {self.channel_index_} 失败")
            return False

        self.channel_handle_ = ret

        if not self.setResistanceEnable(True):
            print("使能终端电阻失败")
            return False

        if not self.setFilter():
            print("滤波设置失败")
            return False

        self.clearRecvBuffer()
        return True

    def start_device(self) -> bool:
        ret = self.lib.VCI_StartCAN(self.kDeviceType, self.device_index_, self.channel_index_)
        if int(ret) == 0:
            print(f"启动 CAN 通道 {self.channel_index_} 失败")
            return False
        self.is_started = True
        return True

    def canfd_send(self, can_id: int, data: Sequence[int]) -> bool:
        self.canfd_send_data = self._new_canfd_frame(can_id, data, queue=False)
        FrameArray = _demo.ZCAN_FD_MSG * 1
        frames = FrameArray(self.canfd_send_data)
        try:
            ret = self.lib.VCI_TransmitFD(
                self.kDeviceType, self.device_index_, self.channel_index_, frames, 1
            )
        except Exception as exc:
            print(f"CANFD 发送异常: {exc}")
            return False
        return int(ret) == 1

    def can_send(self, can_id: int, data: Sequence[int]) -> bool:
        can_data = self._new_can_frame(can_id, data, queue=False)
        FrameArray = _demo.ZCAN_20_MSG * 1
        frames = FrameArray(can_data)
        try:
            ret = self.lib.VCI_Transmit(
                self.kDeviceType, self.device_index_, self.channel_index_, frames, 1
            )
        except Exception as exc:
            print(f"CAN 发送异常: {exc}")
            return False
        return int(ret) == 1

    def _fill_send_queue(self) -> int:
        if self.mode_switch_flag != 0:
            for i, motor in enumerate(self.motors):
                cmd = motor.set_mit_command if self.mode_switch_flag == 1 else motor.set_pv_command
                self.canfd_queue[i] = self._new_canfd_frame(
                    motor.PARAM_SET_ID, cmd, queue=True
                )
        else:
            for i, motor in enumerate(self.motors):
                self.canfd_queue[i] = self._new_canfd_frame(
                    motor.ID_OFFSET, motor.Command, queue=True
                )

        for i, tool in enumerate(self.tools):
            self.canfd_queue[self.MOTOR_NUM + i] = self._new_canfd_frame(
                tool.ID_OFFSET, tool.Command, queue=True
            )
        return self.SLAVE_NUM

    def _canfd_queue_send_thread(self) -> None:
        self.setQueueSend()
        self.clearQueueSend()

        frame = 0
        self.system_update = 0
        FrameArray = _demo.ZCAN_FD_MSG * self.SLAVE_NUM

        while self.is_updating:
            frame += 1
            self._fill_send_queue()
            count = self.SLAVE_NUM if frame % self.tool_update == 0 else self.MOTOR_NUM
            frames = FrameArray(*self.canfd_queue)

            try:
                ret = self.lib.VCI_TransmitFD(
                    self.kDeviceType, self.device_index_, self.channel_index_, frames, count
                )
            except Exception:
                ret = 0

            with self._lock:
                self.send_err_num += max(0, count - int(ret))
                self.send_suc_num += int(ret)
                self.system_update = frame

            # Yield to receive/status threads. Do not add a long sleep here,
            # because the original design relies on high-rate queue sending.
            time.sleep(0)

    def _can_param_update_thread(self) -> None:
        while self.is_updating:
            with self._lock:
                recv_before = self.recv_num
                send_suc_before = self.send_suc_num
                send_err_before = self.send_err_num
                system_before = self.system_update

            time.sleep(self.can_param_time / 1000.0)

            with self._lock:
                recv_in_time = self.recv_num - recv_before
                send_suc_in_time = self.send_suc_num - send_suc_before
                send_err_in_time = self.send_err_num - send_err_before
                system_in_time = self.system_update - system_before

                self.can_param[0] = recv_in_time / self.can_param_time * 1000.0
                self.can_param[1] = (
                    recv_in_time + send_suc_in_time + send_err_in_time
                ) * 49.4 / 1000.0 / self.can_param_time * 100.0
                self.can_param[2] = float(self.send_suc_num)
                self.can_param[3] = float(self.send_err_num)
                self.can_param[4] = float(self.recv_num)
                self.can_param[5] = float(self.system_update)
                self.can_param[6] = system_in_time / self.can_param_time * 1000.0

        with self._lock:
            self.send_suc_num = 0
            self.send_err_num = 0
            self.recv_num = 0

    def start_can_thread(self, type: int) -> None:
        """
        Start three threads:
            1. receive thread: CAN if type == 0, otherwise CANFD
            2. CANFD queue-send thread
            3. communication parameter update thread
        """
        self.is_updating = True
        recv_target = self._can_receive_thread if int(type) == 0 else self._canfd_receive_thread

        self.can_recv_trd = threading.Thread(
            target=recv_target,
            name="can_receive_thread" if int(type) == 0 else "canfd_receive_thread",
            daemon=True,
        )
        self.can_send_trd = threading.Thread(
            target=self._canfd_queue_send_thread, name="canfd_queue_send_thread", daemon=True
        )
        self.can_param_update_trd = threading.Thread(
            target=self._can_param_update_thread, name="can_param_update_thread", daemon=True
        )

        self.can_recv_trd.start()
        self.can_send_trd.start()
        self.can_param_update_trd.start()

    def _get_can_receive_num(self) -> int:
        try:
            return int(self.lib.VCI_GetReceiveNum(
                self.kDeviceType, self.device_index_, self.channel_index_
            ))
        except Exception:
            return 0

    def _get_canfd_receive_num(self) -> int:
        try:
            return int(self.lib.VCI_GetReceiveNum(
                self.kDeviceType, self.device_index_, 0x80000000 + self.channel_index_
            ))
        except Exception:
            return 0

    def _receive_can(self, max_count: int = 100, timeout_ms: int = 50):
        count = min(max(0, int(max_count)), 100)
        if count <= 0:
            return None, 0
        FrameArray = _demo.ZCAN_20_MSG * count
        frames = FrameArray()
        ret = self.lib.VCI_Receive(
            self.kDeviceType, self.device_index_, self.channel_index_, frames, count, timeout_ms
        )
        return frames, int(ret)

    def _receive_canfd(self, max_count: int = 100, timeout_ms: int = 50):
        count = min(max(0, int(max_count)), 100)
        if count <= 0:
            return None, 0
        FrameArray = _demo.ZCAN_FD_MSG * count
        frames = FrameArray()
        ret = self.lib.VCI_ReceiveFD(
            self.kDeviceType, self.device_index_, self.channel_index_, frames, count, timeout_ms
        )
        return frames, int(ret)

    def _can_receive_thread(self) -> None:
        while self.is_updating:
            length = self._get_can_receive_num()
            if length > 0:
                can_data, ret = self._receive_can(min(length, 100), 50)
                for i in range(max(0, int(ret))):
                    self._can_data_proc(can_data[i])
            else:
                time.sleep(0.0005)

    def _canfd_receive_thread(self) -> None:
        while self.is_updating:
            length = self._get_canfd_receive_num()
            if length > 0:
                canfd_data, ret = self._receive_canfd(min(length, 100), 50)
                for i in range(max(0, int(ret))):
                    self._canfd_data_proc(canfd_data[i])
            else:
                time.sleep(0.0005)

    @staticmethod
    def _frame_data8(frame) -> list[int]:
        return [int(frame.dat[i]) & 0xFF for i in range(8)]

    def _canfd_data_proc(self, canfd_data) -> bool:
        with self._lock:
            self.recv_num += 1

        data = self._frame_data8(canfd_data)
        can_id = int(canfd_data.hdr.id) & 0x1FFFFFFF

        if self.mode_switch_flag != 0:
            motor_index = data[0] - 1
            if 0 <= motor_index < self.MOTOR_NUM:
                if self.motors[motor_index].get_motor_mode(data):
                    self.motor_mode[motor_index] = self.motors[motor_index].Mode
                    if all(v == self.motor_mode[0] for v in self.motor_mode):
                        self.mode_switch_flag = 0
                        print("模式切换完成")
            return True

        if 0x11 <= can_id <= 0x16:
            motor_index = (data[0] & 0x0F) - 1
            if 0 <= motor_index < self.MOTOR_NUM:
                self.motors[motor_index].read_motor(data)
                return True
        elif can_id == 0x17:
            self.tools[0].read_motor(data)
            return True
        elif can_id == 0x31:
            return True
        return False

    def _can_data_proc(self, can_data) -> None:
        data = self._frame_data8(can_data)
        can_id = int(can_data.hdr.id) & 0x1FFFFFFF
        if 0x10 <= can_id <= 0x16:
            motor_index = (data[0] & 0x0F) - 1
            if 0 <= motor_index < self.MOTOR_NUM:
                self.motors[motor_index].read_motor(data)

    def stop_can(self) -> None:
        self.is_updating = False
        for th in (self.can_recv_trd, self.can_send_trd, self.can_param_update_trd):
            if th is not None and th.is_alive():
                th.join(timeout=1.0)

    def _wait_for_rec(self, timeout_ms: float) -> Optional[bytes]:
        deadline = time.perf_counter() + timeout_ms / 1000.0

        while time.perf_counter() <= deadline:
            can_len = self._get_can_receive_num()
            if can_len > 0:
                can_data, ret = self._receive_can(min(can_len, 100), 50)
                if ret > 0:
                    frame = can_data[0]
                    data = bytes(int(frame.dat[i]) & 0xFF for i in range(8))
                    return data + bytes([int(frame.hdr.id) & 0xFF])

            canfd_len = self._get_canfd_receive_num()
            if canfd_len > 0:
                canfd_data, ret = self._receive_canfd(min(canfd_len, 100), 50)
                if ret > 0:
                    frame = canfd_data[0]
                    data = bytes(int(frame.dat[i]) & 0xFF for i in range(8))
                    return data + bytes([int(frame.hdr.id) & 0xFF])

            time.sleep(0.0005)

        return None

    def send_wait(
        self, type: int, motor_id: int, send_data: Sequence[int], timeout: float
    ) -> Optional[bytes]:
        self.stop_can()
        self.clearRecvBuffer()

        if int(type) == 0:
            self.can_send(motor_id, send_data)
        else:
            self.canfd_send(motor_id, send_data)

        return self._wait_for_rec(timeout)

    def enable_all(self) -> bool:
        self.delayms(5)
        self.stop_can()

        for motor in self.motors:
            data = self.send_wait(1, motor.ID, DMMotor.clear_error_command, 20)
            if not motor.read_motor(data):
                print(f"电机 {motor.ID} 清错无有效回复")
                return False

            data = self.send_wait(1, motor.ID, DMMotor.enable_command, 20)
            if not motor.read_motor(data):
                print(f"电机 {motor.ID} 使能无有效回复")
                return False

            if not motor.Enable:
                print(f"电机 {motor.ID} 使能失败，ERR={motor.ERRCODE}")
                return False

        for tool in self.tools:
            data = self.send_wait(1, tool.ID, DMMotor.clear_error_command, 20)
            if not tool.read_motor(data):
                print(f"工具电机 {tool.ID} 清错无有效回复")
                return False

            data = self.send_wait(1, tool.ID, DMMotor.enable_command, 20)
            if not tool.read_motor(data):
                print(f"工具电机 {tool.ID} 使能无有效回复")
                return False

        return True

    def disable_all(self) -> None:
        self.stop_can()

        for motor in self.motors:
            data = self.send_wait(1, motor.ID, DMMotor.disable_command, 5)
            motor.read_motor(data)

        for tool in self.tools:
            data = self.send_wait(1, tool.ID, DMMotor.disable_command, 5)
            tool.read_motor(data)

    def set_zero(self, id: int) -> None:
        self.stop_can()

        data = self.send_wait(1, id, DMMotor.set_zero_command, 5)

        if id > self.MOTOR_NUM:
            tool = self.tools[id - self.MOTOR_NUM - 1]
            tool.read_motor(data)
            tool.set_empty_command()
            data = self.send_wait(1, id, tool.Command, 5)
            tool.read_motor(data)
        else:
            motor = self.motors[id - 1]
            motor.read_motor(data)
            motor.set_empty_command()
            data = self.send_wait(1, id, motor.Command, 5)
            motor.read_motor(data)

    def get_status_all(self) -> bool:
        for i in range(self.MOTOR_NUM):
            self.motor_mode[i] = i

        for i, motor in enumerate(self.motors):
            data = self.send_wait(1, 0x7FF, motor.get_mode_command, 5)
            if motor.get_motor_mode(data):
                self.motor_mode[i] = motor.Mode
            else:
                return False

        if not all(v == self.motor_mode[0] for v in self.motor_mode):
            return False

        for motor in self.motors:
            motor.set_empty_command()
            data = self.send_wait(1, motor.ID, motor.Command, 5)
            if data is None:
                return False
            motor.read_motor(data)

        for tool in self.tools:
            data = self.send_wait(1, 0x7FF, tool.get_mode_command, 5)
            tool.get_motor_mode(data)
            tool.set_empty_command()
            data = self.send_wait(1, tool.ID, tool.Command, 5)
            tool.read_motor(data)

        return True

    def set_mode_all(self, mode: int) -> bool:
        if mode not in (1, 2):
            return False

        for i, motor in enumerate(self.motors):
            cmd = motor.set_mit_command if mode == 1 else motor.set_pv_command
            data = self.send_wait(1, 0x7FF, cmd, 50)

            if data is None:
                print(f"电机 {motor.ID} 切换模式无回复")
                return False

            if len(data) < 8:
                print(f"电机 {motor.ID} 切换模式回复长度不足")
                return False

            if data[0] != motor.ID:
                print(f"电机 {motor.ID} 切换模式回复ID不匹配: data[0]={data[0]}")
                return False

            if not motor.get_motor_mode(data):
                print(f"电机 {motor.ID} 模式回复解析失败")
                return False

            if motor.Mode != mode:
                print(f"电机 {motor.ID} 模式切换失败，当前模式={motor.Mode}，目标模式={mode}")
                return False

            self.motor_mode[i] = motor.Mode

        return True

    @staticmethod
    def delayms(time_ms: float) -> float:
        if time_ms == 0:
            return 0.0

        start = time.perf_counter()
        deadline = start + time_ms / 1000.0
        while time.perf_counter() < deadline:
            pass

        return (time.perf_counter() - start) * 1000.0

    def clearRecvBuffer(self) -> bool:
        """
        Linux demo does not expose VCI_ClearBuffer. Drain CAN/CANFD receive queues.
        """
        ok = True

        try:
            for _ in range(20):
                can_len = self._get_can_receive_num()
                canfd_len = self._get_canfd_receive_num()

                if can_len <= 0 and canfd_len <= 0:
                    break

                if can_len > 0:
                    self._receive_can(min(can_len, 100), 1)

                if canfd_len > 0:
                    self._receive_canfd(min(canfd_len, 100), 1)
        except Exception:
            ok = False

        return ok

    def setQueueSend(self) -> None:
        on = c_uint8(1)
        try:
            self.lib.VCI_SetReference(
                self.kDeviceType,
                self.device_index_,
                self.channel_index_,
                _demo.ZCAN_CMD_SET_SEND_QUEUE_EN,
                byref(on),
            )
        except Exception:
            pass

    def clearQueueSend(self) -> None:
        on = c_uint8(1)
        try:
            self.lib.VCI_SetReference(
                self.kDeviceType,
                self.device_index_,
                self.channel_index_,
                _demo.ZCAN_CMD_SET_SEND_QUEUE_CLR,
                byref(on),
            )
        except Exception:
            pass

    @staticmethod
    def MakeCanId(id: int, eff: int, rtr: int, err: int) -> int:
        return int(id) | ((1 if eff else 0) << 31) | ((1 if rtr else 0) << 30) | (
            (1 if err else 0) << 29
        )

    def setCANFDStandard(self, canfd_standard: int) -> bool:
        # The Linux demo configures CANFD through timing parameters, not this string path.
        self.canfd_standard = int(canfd_standard)
        return True

    def setFdBaudrate(self, abaud: int, dbaud: int) -> bool:
        """
        Compatibility method.

        The verified ceshi.py gives explicit 1M/5M timing values:
            abit: tseg1=43, tseg2=14, sjw=1, brp=0
            dbit: tseg1=1, tseg2=0, sjw=1, brp=2

        If you need a different bitrate, modify abit_timing/dbit_timing directly
        before init_device(), or extend this method with timing-table entries.
        """
        self.abit_baud = int(abaud)
        self.dbit_baud = int(dbaud)
        return True

    def set_canfd_timing(
        self,
        *,
        clk: int = 60_000_000,
        abit_tseg1: int = 43,
        abit_tseg2: int = 14,
        abit_sjw: int = 1,
        abit_smp: int = 0,
        abit_brp: int = 0,
        dbit_tseg1: int = 1,
        dbit_tseg2: int = 0,
        dbit_sjw: int = 1,
        dbit_smp: int = 0,
        dbit_brp: int = 2,
    ) -> None:
        """
        Set Linux CANFD timing manually before init_device().
        """
        self.canfd_clock_hz = int(clk)
        self.abit_timing = dict(
            tseg1=int(abit_tseg1),
            tseg2=int(abit_tseg2),
            sjw=int(abit_sjw),
            smp=int(abit_smp),
            brp=int(abit_brp),
        )
        self.dbit_timing = dict(
            tseg1=int(dbit_tseg1),
            tseg2=int(dbit_tseg2),
            sjw=int(dbit_sjw),
            smp=int(dbit_smp),
            brp=int(dbit_brp),
        )

    def setResistanceEnable(self, enable: bool) -> bool:
        value = c_uint8(1 if enable else 0)
        try:
            ret = self.lib.VCI_SetReference(
                self.kDeviceType,
                self.device_index_,
                self.channel_index_,
                _demo.CMD_CAN_TRES,
                byref(value),
            )
        except Exception:
            return False
        return int(ret) != 0

    def setCustomBaudrate(self, ABIT: str) -> bool:
        """
        Compatibility method for the Windows version.

        The Linux demo does not accept the same string-style custom bitrate.
        The actual timing used by init_device() is stored in abit_timing/dbit_timing.
        """
        self.custom_baudrate_string = str(ABIT)
        return True

    def setFilter(self) -> bool:
        """
        Set a broad receive filter. If the vendor library rejects the filter,
        keep running and return True because some devices accept all frames by default.
        """
        try:
            filter_table = _demo.ZCAN_FILTER_TABLE()
            memset(byref(filter_table), 0, sizeof(filter_table))
            filter_table.size = sizeof(_demo.ZCAN_FILTER) * 1
            filter_table.table[0].type = 1 if self.canfd_extended else 0
            filter_table.table[0].sid = 0x0
            filter_table.table[0].eid = 0x1FFFFFFF if self.canfd_extended else 0x7FF

            ret = self.lib.VCI_SetReference(
                self.kDeviceType,
                self.device_index_,
                self.channel_index_,
                _demo.CMD_CAN_FILTER,
                byref(filter_table),
            )
            # Some driver versions return 0 here but still use default pass-all.
            return True if int(ret) == 0 else True
        except Exception:
            return True


__all__ = ["USBCANFD"]
