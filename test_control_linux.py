# Linux系统下交互式切换 1~6 号电机模式：
# 1 -> MIT 模式，并使用重力补偿
# 2 -> PV 模式，移动到指定 DH 关节角位置
# 3 -> PVT 模式，保持当前电机位置
# 0 -> 退出

import time
import math
import threading
from types import MethodType
from typing import Optional

# Linux版建议使用前面生成的 USBCANFD_fix.py；
# 如果你已经把 USBCANFD_fix.py 改名为 USBCANFD.py，下面的备用导入也可以兼容。

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

# 模式切换参数
MODE_SWITCH_TIMEOUT_S = 3.0
# 重力补偿参数
GRAVITY_COMP_PERIOD_S = 0.001

# 1轴和6轴默认设为0
GRAVITY_TORQUE_SCALE = [0.0, 1.2, 1.1, 1.0, 1.0, 0.0]

# PV 模式目标位置参数

# [NOW DH rad] = ['-3.1367', '-3.1357', '3.1036', '0.0496', '-1.5958', '-0.1751']
PV_TARGET_DH_Q = [
    3.10,
    -3.13,
    3.10,
    0.05,
    -1.40,
    -0.07,
]

# PV 模式速度限制，越小运动越慢
# 第一次测试建议小一点，比如 0.2~0.5
PV_MOVE_VEL_LIM = 0.3

# 如果目标角度无法转换为合法电机角，是否退回当前位置保持
PV_FALLBACK_TO_HOLD = True

# PVT 模式保持参数
PVT_HOLD_VEL_LIM = 0
PVT_HOLD_TORQUE_LIM = 0
# 其他参数
POWER_ON_WAIT_S = 1.0
DISABLE_ON_EXIT = True

def patch_usbcanfd_for_linux_pvt_and_six_motors(can: USBCANFD):
    """
    针对 Linux 版 USBCANFD_fix.py 做运行时补丁：
    1. 让 mode_switch_flag 支持 MIT / PV / PVT 三种模式；
    2. 连续队列发送线程只发送 1~6 号关节电机，不发送第 7 个工具电机；
    3. 适配 Linux 版 _new_canfd_frame(can_id, data, queue=True) 接口。

    Windows 版 _new_canfd_frame(..., flags=0x11) 是 zlgcan 风格，
    Linux 版 USBCANFD_fix.py 使用 libusbcanfd.so / VCI_* API，
    因此这里必须改为 queue=True。
    """

    def _fill_send_queue_patched(self) -> int:
        if self.mode_switch_flag != 0:
            for i, motor in enumerate(self.motors):
                if self.mode_switch_flag == MODE_MIT:
                    cmd = motor.set_mit_command
                elif self.mode_switch_flag == MODE_PV:
                    cmd = motor.set_pv_command
                elif self.mode_switch_flag == MODE_PVT:
                    cmd = motor.set_pvt_command
                else:
                    cmd = motor.set_pv_command

                self.canfd_queue[i] = self._new_canfd_frame(motor.PARAM_SET_ID, cmd, queue=True)
        else:
            for i, motor in enumerate(self.motors):
                self.canfd_queue[i] = self._new_canfd_frame(motor.ID_OFFSET, motor.Command, queue=True)

        # 只操作前6个关节电机；第7个工具电机不放入本次发送数量。
        return self.MOTOR_NUM

    def _canfd_queue_send_thread_six_motors(self) -> None:
        """Linux连续发送线程：只发送 self.motors[0:6]，不周期性发送工具电机。"""
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
    只使能前6个电机。
    注意：这个函数会使用 send_wait()，所以只应在 start_can_thread() 之前调用。
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

def disable_motors_only_at_exit(can: USBCANFD):
    """
    退出程序时失能前6个电机。
    这里会 stop_can()，只在退出阶段使用。
    """
    can.stop_can()

    for motor in can.motors:
        data = can.send_wait(1, motor.ID, DMMotor.disable_command, 50)
        motor.read_motor(data)
        print(f"[EXIT] 电机 {motor.ID} 已发送失能命令，Enable={motor.Enable}, ERR={motor.ERRCODE}")

def set_all_mit_zero_torque(can: USBCANFD):
    """
    清零 MIT 命令缓存。
    """
    for motor in can.motors:
        motor.MIT.position_set = 0.0
        motor.MIT.velocity_set = 0.0
        motor.MIT.kp_set = 0.0
        motor.MIT.kd_set = 0.0
        motor.MIT.torque_set = 0.0
        motor.set()

def set_pv_hold_current_position(can: USBCANFD):
    """
    PV 模式下保持当前电机位置。
    注意：这里用的是电机侧 Position，不是 DH 角。
    """
    for motor in can.motors:
        motor.PV.position_set = float(motor.Position)
        motor.PV.velocity_lim = float(PV_MOVE_VEL_LIM)
        motor.set()

    print("[PV] 已设置为当前位置保持")

def set_pv_target_motor_position(can: USBCANFD, target_motor_q, velocity_lim: float = PV_MOVE_VEL_LIM) -> bool:
    """
    PV 模式下设置目标电机角。
    target_motor_q 是电机侧角度，单位 rad。
    """
    if len(target_motor_q) != 6:
        print("[ERR] target_motor_q 必须是 6 个数")
        return False

    # 限位检查
    for i, motor in enumerate(can.motors):
        target = float(target_motor_q[i])

        if not (motor.angle_lim[0] <= target <= motor.angle_lim[1]):
            print(
                f"[ERR] 电机 {motor.ID} 目标角超限: "
                f"target={target:.4f}, "
                f"limit=[{motor.angle_lim[0]:.4f}, {motor.angle_lim[1]:.4f}]"
            )
            return False

    # 写入 PV 命令
    for i, motor in enumerate(can.motors):
        motor.PV.position_set = float(target_motor_q[i])
        motor.PV.velocity_lim = float(velocity_lim)
        motor.set()

    print("[PV] 已写入目标电机角 rad:")
    print(["{:.4f}".format(float(x)) for x in target_motor_q])

    return True

def set_pv_target_dh_position(can: USBCANFD, robot: Robot, target_dh_q, velocity_lim: float = PV_MOVE_VEL_LIM) -> bool:
    """
    PV 模式下设置目标 DH 关节角。
    target_dh_q 是机械臂 DH 关节角，单位 rad。
    函数内部会通过 robot.dh2motor() 转换为电机侧角度。
    """
    if len(target_dh_q) != 6:
        print("[ERR] target_dh_q 必须是 6 个数")
        return False

    print("[PV] 目标 DH 关节角 rad:")
    print(["{:.4f}".format(float(x)) for x in target_dh_q])

    print("[PV] 目标 DH 关节角 deg:")
    print(["{:.2f}".format(float(x * 180.0 / math.pi)) for x in target_dh_q])

    ok, target_motor_q, in_range = robot.dh2motor(can.motors, target_dh_q)

    if not ok:
        print("[ERR] 目标 DH 关节角无法转换为合法电机角")
        print(f"in_range={in_range}")
        print("target_dh_q rad =", ["{:.4f}".format(float(x)) for x in target_dh_q])
        return False

    return set_pv_target_motor_position(can, target_motor_q, velocity_lim)

def set_pv_move_to_fixed_position(can: USBCANFD, robot: Robot) -> bool:
    """
    切换到 PV 模式后，让机械臂移动到固定 DH 关节角位置。
    """
    print("[PV] 准备移动到固定 DH 关节角位置")
    return set_pv_target_dh_position(can, robot, PV_TARGET_DH_Q, PV_MOVE_VEL_LIM)

def set_pvt_hold_current_position(can: USBCANFD):
    """
    PVT 模式下保持当前电机位置。
    具体 velocity_lim / torque_lim 是否需要非零，要根据你电机协议实际定义调整。
    """
    for motor in can.motors:
        motor.PVT.position_set = float(motor.Position)
        motor.PVT.velocity_lim = int(PVT_HOLD_VEL_LIM)
        motor.PVT.torque_lim = int(PVT_HOLD_TORQUE_LIM)
        motor.set()

    print("[PVT] 已设置为当前位置保持")

def prepare_command_for_target_mode(can: USBCANFD, robot: Robot, target_mode: int):
    """
    切换模式前，先准备目标模式下的第一帧命令。
    这样模式切换完成后，队列发送线程不会发送未初始化命令。
    """
    if target_mode == MODE_MIT:
        print("[PREPARE] 目标为 MIT：保持重力补偿线程写入的 MIT 命令")

    elif target_mode == MODE_PV:
        print("[PREPARE] 目标为 PV：设置为固定 DH 关节角目标")
        ok = set_pv_move_to_fixed_position(can, robot)

        if not ok:
            if PV_FALLBACK_TO_HOLD:
                print("[WARN] 固定 DH 目标设置失败，退回当前位置保持")
                set_pv_hold_current_position(can)
            else:
                raise RuntimeError("PV 固定 DH 目标设置失败")

    elif target_mode == MODE_PVT:
        print("[PREPARE] 目标为 PVT：设置为当前位置保持")
        set_pvt_hold_current_position(can)

def wait_motor_feedback(can: USBCANFD, timeout_s: float = 2.0) -> bool:
    deadline = time.time() + timeout_s

    while time.time() < deadline:
        if all(m.recv_num > 0 for m in can.motors):
            return True
        time.sleep(0.005)

    print(f"[WARN] 等待反馈超时，recv_num={[m.recv_num for m in can.motors]}")
    return False

def online_switch_mode_keep_enable(can: USBCANFD, robot: Robot, target_mode: int) -> bool:
    """
    在线切换模式：
    - 不失能
    - 不停止 CANFD 三线程
    - 不停止重力补偿线程
    - 通过 mode_switch_flag 让队列发送线程持续发送模式切换命令
    """
    if target_mode not in (MODE_MIT, MODE_PV, MODE_PVT):
        print(f"[ERR] 不支持的模式: {target_mode}")
        return False

    target_name = MODE_NAME[target_mode]

    if not can.IsUpdating:
        print("[ERR] CANFD 连续收发线程未启动，无法在线切换模式")
        return False

    prepare_command_for_target_mode(can, robot, target_mode)

    # 先重置模式记录，避免旧状态导致误判
    for i in range(can.MOTOR_NUM):
        can.motor_mode[i] = 0

    print(f"[SWITCH] 在线切换到 {target_name} 模式：不失能、不停止重力补偿")

    # 关键：由 _fill_send_queue 在线发送模式切换命令
    can.mode_switch_flag = target_mode

    deadline = time.time() + MODE_SWITCH_TIMEOUT_S

    while time.time() < deadline:
        modes = [m.Mode for m in can.motors]

        if can.mode_switch_flag == 0 and all(m == target_mode for m in modes):
            print(f"[OK] 已在线切换到 {target_name} 模式")
            return True

        time.sleep(0.01)

    print(
        f"[ERR] 在线切换到 {target_name} 模式超时，"
        f"当前 modes={[m.Mode for m in can.motors]}, flag={can.mode_switch_flag}"
    )
    can.mode_switch_flag = 0
    return False

def gravity_comp_loop(can: USBCANFD, robot: Robot, stop_event: threading.Event):
    """
    重力补偿线程：
    - 线程一直运行，不因模式切换而退出
    - 始终根据当前电机反馈计算 tau_g_motor
    - 始终更新 MIT 命令缓存
    - 只有当电机处于 MIT 模式时，队列发送线程才会实际发送 MIT.Command
    """
    print("[GRAVITY] 重力补偿线程已启动，模式切换时不会停止")

    last_print_time = time.time()
    loop_count = 0

    while not stop_event.is_set():
        try:
            robot.Angle = robot.motor2dh(can.motors)

            if not robot.set_robot():
                time.sleep(GRAVITY_COMP_PERIOD_S)
                continue

            tau_g_motor = robot.Tau_G_Motor

            for i, motor in enumerate(can.motors):
                motor.MIT.position_set = 0.0
                motor.MIT.velocity_set = 0.0
                motor.MIT.kp_set = 0.0
                motor.MIT.kd_set = 0.0
                motor.MIT.torque_set = float(tau_g_motor[i] * GRAVITY_TORQUE_SCALE[i])
                motor.set()

            loop_count += 1

            now = time.time()
            if now - last_print_time >= 1.0:
                # 不频繁打印，避免影响控制循环
                loop_count = 0
                last_print_time = now

            time.sleep(GRAVITY_COMP_PERIOD_S)

        except Exception as e:
            print(f"[ERR] 重力补偿线程异常: {e}")
            time.sleep(0.01)

    print("[GRAVITY] 重力补偿线程退出")

def print_menu(current_mode: Optional[int]):
    print("\n" + "=" * 60)

    if current_mode is None:
        print("当前目标模式：未知")
    else:
        print(f"当前目标模式：{MODE_NAME.get(current_mode, current_mode)}")

    print("请选择操作：")
    print("1 - 在线切换到 MIT 模式，电机不失能，重力补偿不停止")
    print("2 - 在线切换到 PV 模式，移动到固定 DH 关节角位置")
    print("3 - 在线切换到 PVT 模式，当前位置保持")
    print("s - 查看当前 1~6 号电机模式/使能状态/DH关节角")
    print("0 - 退出程序")
    print("=" * 60)

def print_motor_status(can: USBCANFD):
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


def print_current_dh_angle(can: USBCANFD, robot: Robot):
    q_now = robot.motor2dh(can.motors)

    print("[NOW DH rad] =", ["{:.4f}".format(float(x)) for x in q_now])
    print("[NOW DH deg] =", ["{:.2f}".format(float(x * 180.0 / math.pi)) for x in q_now])

def main():
    # USBCANFD_fix.py 默认设备类型为 43，默认波特率为 ceshi.py 中验证的 1 Mbps / 5 Mbps。
    can = USBCANFD(device_index=0, channel_index=0, canfd_extended=False, canfd_brs=True)
    robot = Robot()

    gravity_stop_event = threading.Event()
    gravity_thread = None
    current_mode: Optional[int] = None

    try:
        patch_usbcanfd_for_linux_pvt_and_six_motors(can)

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

        print("[4] 使能前 6 个电机...")
        if not enable_motors_only_before_thread(can):
            raise RuntimeError("前 6 个电机使能失败")

        print("[5] 初始化 MIT 零力矩缓存...")
        set_all_mit_zero_torque(can)

        print("[6] 启动 CANFD 连续收发线程...")
        can.start_can_thread(1)

        print("[7] 等待电机反馈...")
        wait_motor_feedback(can, timeout_s=2.0)

        print("[8] 启动重力补偿线程...")
        gravity_thread = threading.Thread(
            target=gravity_comp_loop,
            args=(can, robot, gravity_stop_event),
            name="gravity_comp_loop",
            daemon=True,
        )
        gravity_thread.start()

        print("[OK] 初始化完成")
        print("[INFO] 后续模式切换不会主动失能，也不会停止重力补偿线程。")
        print("[INFO] 本程序只操作 1~6 号电机，不操作第 7 个工具电机。")
        print("[INFO] 输入 2 切换到 PV 模式后，机械臂将移动到指定 DH 关节角位置。")

        while True:
            print_menu(current_mode)
            choice = input("请输入: ").strip().lower()

            if choice == "0":
                print("[EXIT] 准备退出")
                break

            if choice == "s":
                print_motor_status(can)
                print_current_dh_angle(can, robot)
                continue

            if choice not in ("1", "2", "3"):
                print("[WARN] 无效输入")
                continue

            target_mode = {
                "1": MODE_MIT,
                "2": MODE_PV,
                "3": MODE_PVT,
            }[choice]

            target_name = MODE_NAME[target_mode]

            if current_mode == target_mode:
                print(f"[INFO] 当前目标模式已经是 {target_name}，不重复切换")
                continue

            ok = online_switch_mode_keep_enable(can, robot, target_mode)

            if ok:
                current_mode = target_mode
                print_motor_status(can)
                print_current_dh_angle(can, robot)
            else:
                print(f"[ERR] 切换到 {target_name} 失败，请检查电机反馈和 CAN 通信")

    except KeyboardInterrupt:
        print("\n[INTERRUPT] 用户中断")

    except Exception as e:
        print(f"[ERR] 程序异常: {e}")

    finally:
        print("[CLEANUP] 准备退出...")

        # 退出时才停止重力补偿线程
        gravity_stop_event.set()
        if gravity_thread is not None and gravity_thread.is_alive():
            gravity_thread.join(timeout=1.0)

        # 退出前先把 MIT 力矩缓存清零
        try:
            set_all_mit_zero_torque(can)
            time.sleep(0.05)
        except Exception as e:
            print(f"[WARN] 清零 MIT 力矩异常: {e}")

        if DISABLE_ON_EXIT:
            try:
                print("[CLEANUP] 退出时失能前 6 个电机...")
                disable_motors_only_at_exit(can)
            except Exception as e:
                print(f"[WARN] 退出失能异常: {e}")
        else:
            print("[CLEANUP] DISABLE_ON_EXIT=False，退出时不失能电机")

        try:
            can.stop_can()
            can.close_device()
        except Exception as e:
            print(f"[WARN] 关闭 CANFD 设备异常: {e}")

        print("[END] 程序退出")

if __name__ == "__main__":
    main()