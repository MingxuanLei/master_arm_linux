# Linux 下测试 MIT 模式机械臂重力补偿
# 依赖文件建议放在同一目录：
#   DMMotor.py
#   Robot.py
#   TreeStruct.py
#   USBCANFD.py        # 建议把之前生成的 USBCANFD_linux.py 改名为 USBCANFD.py
#   USBCANFD_DEMO.py
#   libusbcanfd.so
#
# 如果你不想改名 USBCANFD_linux.py，可以把下面导入改为：
#   from USBCANFD_linux import USBCANFD

import time

from USBCANFD import USBCANFD
from Robot import Robot


def init_canfd() -> USBCANFD:
    # Linux 版 USBCANFD 的参数：
    # device_index=0: 第一块 USBCANFD 设备
    # channel_index=0: CAN0 通道
    # canfd_extended=False: 标准帧；如果一直收不到电机回复，可以改成 True 试试扩展帧
    can = USBCANFD(device_index=0, channel_index=0, canfd_extended=False)

    # 默认就是 1 Mbps 仲裁域 + 5 Mbps 数据域，对应 ceshi.py 中的配置。
    # 如果你改过波特率，可以在 init_device() 之前调用 set_canfd_timing()。
    # can.set_canfd_timing(
    #     clk=60_000_000,
    #     abit_tseg1=14, abit_tseg2=3, abit_sjw=2, abit_smp=0, abit_brp=5,
    #     dbit_tseg1=10, dbit_tseg2=2, dbit_sjw=2, dbit_smp=0, dbit_brp=1,
    # )

    if not can.open_device():
        raise RuntimeError("无法打开 Linux USBCANFD 设备，请检查 libusbcanfd.so、驱动、USB 权限和设备连接")

    if not can.init_device():
        can.close_device()
        raise RuntimeError("初始化 Linux USBCANFD 失败，请检查通道号、波特率和终端电阻")

    if not can.start_device():
        can.close_device()
        raise RuntimeError("启动 Linux USBCANFD 通道失败")

    return can


def main():
    can = init_canfd()
    robot = Robot()

    try:
        # 刚上电后，CANFD 设备、电机驱动器和总线通信都需要一点稳定时间
        time.sleep(0.5)

        # 清空启动阶段可能残留的无效帧
        can.clearRecvBuffer()

        # 先读取一次状态/模式，让总线和电机通信预热
        for attempt in range(5):
            if can.get_status_all():
                print("电机状态读取成功，准备切换 MIT 模式")
                break
            print(f"第 {attempt + 1} 次读取电机状态失败，继续等待...")
            time.sleep(0.2)
        else:
            raise RuntimeError("上电后无法读取电机状态，请检查电源、CAN 接线、波特率、终端电阻、标准帧/扩展帧设置")

        # 切换 1~6 号电机到 MIT 模式
        for attempt in range(5):
            if can.set_mode_all(1):
                print("MIT 模式切换成功")
                break
            print(f"第 {attempt + 1} 次切换 MIT 模式失败，重试...")
            time.sleep(0.2)
        else:
            raise RuntimeError("切换 MIT 模式失败")

        # MIT 空命令，避免刚启动时残留非零力矩
        for motor in can.motors:
            motor.set_empty_command_MIT()
            motor.set()

        # 使能所有电机。Linux 版建议检查返回值，避免未使能就进入力矩循环。
        if not can.enable_all():
            raise RuntimeError("电机使能失败，请检查电机状态码、供电和 CANFD 通信")

        # 启动 CANFD 三线程：接收线程、队列发送线程、CAN 参数刷新线程
        # 1 表示使用 CANFD 接收线程；0 表示普通 CAN 接收线程。
        can.start_can_thread(1)

        while True:
            # 1. 电机角度 -> DH 角
            robot.Angle = robot.motor2dh(can.motors)

            # 2. 更新正运动学、雅可比、重力补偿
            robot.set_robot()
            tau_g_motor = robot.Tau_G_Motor

            # 3. 写入 MIT 力矩命令
            # 保持 Windows 版 test1 原来的补偿关节和比例不变。
            can.motors[4].MIT.torque_set = float(tau_g_motor[4])
            can.motors[4].set()

            can.motors[3].MIT.torque_set = float(tau_g_motor[3])
            can.motors[3].set()

            can.motors[2].MIT.torque_set = float(tau_g_motor[2] * 1.1)
            can.motors[2].set()

            can.motors[1].MIT.torque_set = float(tau_g_motor[1] * 1.1)
            can.motors[1].set()

            # Python 主循环不要完全空转，否则会抢线程调度
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("停止控制")

    finally:
        # 先把 MIT 命令清零，再失能和关闭设备
        try:
            for motor in can.motors:
                motor.set_empty_command_MIT()
                motor.set()
            time.sleep(0.05)
            can.disable_all()
        finally:
            can.stop_can()
            can.close_device()


if __name__ == "__main__":
    main()
