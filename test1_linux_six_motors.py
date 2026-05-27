# Linux 下测试 MIT 模式机械臂重力补偿：只使能/控制前六个关节电机，不使能第七个工具电机
# 依赖文件建议放在同一目录：
#   DMMotor.py
#   Robot.py
#   TreeStruct.py
#   USBCANFD_fix.py      # 如果你已改名为 USBCANFD.py，请修改下面的导入语句
#   USBCANFD_DEMO.py
#   libusbcanfd.so

import time
from types import MethodType

from DMMotor import DMMotor
from Robot import Robot
from USBCANFD import USBCANFD
# 如果你把 USBCANFD_fix.py 改名成了 USBCANFD.py，则改为：
# from USBCANFD import USBCANFD


def init_canfd() -> USBCANFD:
    """初始化 Linux USBCANFD。默认设备类型与波特率由 USBCANFD_fix.py 内部配置。"""
    can = USBCANFD(device_index=0, channel_index=0, canfd_extended=False)

    if not can.open_device():
        raise RuntimeError("无法打开 Linux USBCANFD 设备，请检查 libusbcanfd.so、驱动、USB 权限和设备连接")

    if not can.init_device():
        can.close_device()
        raise RuntimeError("初始化 Linux USBCANFD 失败，请检查通道号、波特率和终端电阻")

    if not can.start_device():
        can.close_device()
        raise RuntimeError("启动 Linux USBCANFD 通道失败")

    return can


def configure_six_motor_only(can: USBCANFD) -> None:
    """
    让本测试脚本的周期发送线程只发送 1~6 号关节电机命令，不发送第 7 个工具电机命令。

    注意：这里不删除 can.tools，这样即使总线上偶然收到 0x17 工具电机反馈，
    USBCANFD_fix.py 的接收线程仍然能正常解析，不会因为 tools 为空而报错。
    """
    can.TOOL_NUM = 0
    can.SLAVE_NUM = can.MOTOR_NUM
    can.canfd_queue = can.canfd_queue[:can.MOTOR_NUM]

    def _fill_send_queue_motors_only(self) -> int:
        if self.mode_switch_flag != 0:
            for i, motor in enumerate(self.motors):
                cmd = motor.set_mit_command if self.mode_switch_flag == 1 else motor.set_pv_command
                self.canfd_queue[i] = self._new_canfd_frame(motor.PARAM_SET_ID, cmd, queue=True)
        else:
            for i, motor in enumerate(self.motors):
                self.canfd_queue[i] = self._new_canfd_frame(motor.ID_OFFSET, motor.Command, queue=True)
        return self.MOTOR_NUM

    can._fill_send_queue = MethodType(_fill_send_queue_motors_only, can)


def get_status_motors_only(can: USBCANFD) -> bool:
    """只读取 1~6 号关节电机状态，不读取第 7 个工具电机状态。"""
    for i in range(can.MOTOR_NUM):
        can.motor_mode[i] = i

    for i, motor in enumerate(can.motors):
        data = can.send_wait(1, 0x7FF, motor.get_mode_command, 5)
        if motor.get_motor_mode(data):
            can.motor_mode[i] = motor.Mode
        else:
            return False

    if not all(v == can.motor_mode[0] for v in can.motor_mode):
        return False

    for motor in can.motors:
        motor.set_empty_command()
        data = can.send_wait(1, motor.ID, motor.Command, 5)
        if data is None:
            return False
        motor.read_motor(data)

    return True


def enable_motors_only(can: USBCANFD) -> bool:
    """只清错并使能 1~6 号关节电机，不操作第 7 个工具电机。"""
    can.delayms(5)
    can.stop_can()

    for motor in can.motors:
        data = can.send_wait(1, motor.ID, DMMotor.clear_error_command, 20)
        if not motor.read_motor(data):
            print(f"电机 {motor.ID} 清错无有效回复")
            return False

        data = can.send_wait(1, motor.ID, DMMotor.enable_command, 20)
        if not motor.read_motor(data):
            print(f"电机 {motor.ID} 使能无有效回复")
            return False

        if not motor.Enable:
            print(f"电机 {motor.ID} 使能失败，ERR={motor.ERRCODE}")
            return False

    return True


def disable_motors_only(can: USBCANFD) -> None:
    """只失能 1~6 号关节电机，不操作第 7 个工具电机。"""
    can.stop_can()

    for motor in can.motors:
        data = can.send_wait(1, motor.ID, DMMotor.disable_command, 5)
        motor.read_motor(data)


def main():
    can = init_canfd()
    configure_six_motor_only(can)
    robot = Robot()

    try:
        # 刚上电后，CANFD 设备、电机驱动器和总线通信都需要一点稳定时间
        time.sleep(0.5)

        # 清空启动阶段可能残留的无效帧
        can.clearRecvBuffer()

        # 只读取 1~6 号关节电机状态/模式，让总线和电机通信预热
        for attempt in range(5):
            if get_status_motors_only(can):
                print("1~6 号关节电机状态读取成功，准备切换 MIT 模式")
                break
            print(f"第 {attempt + 1} 次读取 1~6 号关节电机状态失败，继续等待...")
            time.sleep(0.2)
        else:
            raise RuntimeError("上电后无法读取 1~6 号关节电机状态，请检查电源、CAN 接线、波特率、终端电阻、标准帧/扩展帧设置")

        # 切换 1~6 号关节电机到 MIT 模式；不切换第 7 个工具电机
        for attempt in range(5):
            if can.set_mode_all(1):
                print("1~6 号关节电机 MIT 模式切换成功")
                break
            print(f"第 {attempt + 1} 次切换 MIT 模式失败，重试...")
            time.sleep(0.2)
        else:
            raise RuntimeError("切换 MIT 模式失败")

        # MIT 空命令，避免刚启动时残留非零力矩；只设置 1~6 号关节电机
        for motor in can.motors:
            motor.set_empty_command_MIT()
            motor.set()

        # 只使能 1~6 号关节电机，不使能第 7 个工具电机
        if not enable_motors_only(can):
            raise RuntimeError("1~6 号关节电机使能失败，请检查电机状态码、供电和 CANFD 通信")

        print("1~6 号关节电机已使能；第 7 个工具电机保持不使能")

        # 启动 CANFD 三线程：接收线程、队列发送线程、CAN 参数刷新线程
        # 由于前面已重写 _fill_send_queue，本周期发送线程只会发送 1~6 号关节电机命令。
        can.start_can_thread(1)

        while True:
            # 1. 电机角度 -> DH 角
            robot.Angle = robot.motor2dh(can.motors)

            # 2. 更新正运动学、雅可比、重力补偿
            robot.set_robot()
            tau_g_motor = robot.Tau_G_Motor

            # 3. 写入 MIT 力矩命令
            # 保持原 test1_linux.py 的补偿关节和比例不变。
            can.motors[4].MIT.torque_set = float(tau_g_motor[4])
            can.motors[4].set()

            can.motors[3].MIT.torque_set = float(tau_g_motor[3])
            can.motors[3].set()

            can.motors[2].MIT.torque_set = float(tau_g_motor[2] * 1.1)
            can.motors[2].set()

            can.motors[1].MIT.torque_set = float(tau_g_motor[1] * 1.15)
            can.motors[1].set()

            # Python 主循环不要完全空转，否则会抢线程调度
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("停止控制")

    finally:
        # 只对 1~6 号关节电机清 MIT 命令并失能，不操作第 7 个工具电机
        try:
            for motor in can.motors:
                motor.set_empty_command_MIT()
                motor.set()
            time.sleep(0.05)
            disable_motors_only(can)
        finally:
            can.stop_can()
            can.close_device()


if __name__ == "__main__":
    main()
