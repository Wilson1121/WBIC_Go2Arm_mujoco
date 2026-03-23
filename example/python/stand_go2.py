import time
import sys
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

# Go2 四条腿在“站立”姿态下的 12 个关节目标角。
# 顺序按 SDK 电机编号排列，每条腿通常是 hip/thigh/calf 三个关节。
stand_up_joint_pos = np.array([
    0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
    0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763
],
                              dtype=float)

# Go2 四条腿在“蹲下/收缩”姿态下的 12 个关节目标角。
# 例程的动作就是在 stand_down 和 stand_up 两组目标之间平滑切换。
stand_down_joint_pos = np.array([
    0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375, 0.0473455,
    1.22187, -2.44375, -0.0473455, 1.22187, -2.44375
],
                                dtype=float)

# 控制周期 2ms，对应 500Hz 发送低层关节命令。
dt = 0.002
runing_time = 0.0
# Unitree 低层指令需要 CRC 校验。
crc = CRC()

# 让用户先启动仿真器，再手动确认开始发送关节命令。
input("Press enter to start")

if __name__ == '__main__':

    # 默认连仿真器：domain_id=1，网卡 lo。
    # 如果命令行传了参数，则按实机模式初始化通信。
    if len(sys.argv) <2:
        ChannelFactoryInitialize(1, "lo")
    else:
        ChannelFactoryInitialize(0, sys.argv[1])

    # 创建 LowCmd 发布器，持续往 rt/lowcmd 话题发送关节控制指令。
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()

    # 初始化一帧低层控制消息。
    cmd = unitree_go_msg_dds__LowCmd_()
    cmd.head[0] = 0xFE
    cmd.head[1] = 0xEF
    cmd.level_flag = 0xFF
    cmd.gpio = 0

    # 先把所有电机命令清零，并切到 PMSM 伺服模式。
    # Go2 消息里一共预留了 20 个电机槽位，但这个例程只控制前 12 个腿关节。
    for i in range(20):
        cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
        cmd.motor_cmd[i].q = 0.0
        cmd.motor_cmd[i].kp = 0.0
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].kd = 0.0
        cmd.motor_cmd[i].tau = 0.0

    while True:
        step_start = time.perf_counter()

        runing_time += dt

        if (runing_time < 3.0):
            # 前 3 秒做“站起”动作。
            # 用 tanh 生成平滑相位，从 stand_down 逐步过渡到 stand_up，
            # 避免目标角突然跳变。
            phase = np.tanh(runing_time / 1.2)
            for i in range(12):
                # 位置控制：目标角在蹲下姿态和站立姿态之间插值。
                cmd.motor_cmd[i].q = phase * stand_up_joint_pos[i] + (
                    1 - phase) * stand_down_joint_pos[i]
                # 刚开始增益小一些，站稳后逐渐提高 kp。
                cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = 3.5
                # 这里不额外给前馈力矩，主要靠 PD 控制把关节拉到目标角。
                cmd.motor_cmd[i].tau = 0.0
        else:
            # 3 秒后开始“蹲下”动作，从站立姿态反向过渡回收缩姿态。
            phase = np.tanh((runing_time - 3.0) / 1.2)
            for i in range(12):
                cmd.motor_cmd[i].q = phase * stand_down_joint_pos[i] + (
                    1 - phase) * stand_up_joint_pos[i]
                cmd.motor_cmd[i].kp = 50.0
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = 3.5
                cmd.motor_cmd[i].tau = 0.0

        # 每次发送前都要重新计算 CRC，否则消息会被接收端判为无效。
        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)

        # 按 dt 节拍稳定运行控制循环。
        time_until_next_step = dt - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
