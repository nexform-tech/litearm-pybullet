#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 01 · 独立仿真 — 创建仿真机械臂并读取状态（只读，不运动）

演示:
  PyBulletArm(render=True).connect()  创建仿真并打开可视化窗口
  arm.get_state()                     读取当前状态（Msg 信封，状态在 .value）
  arm.get_tcp()                       当前末端位姿（xyz + rpy）
  arm.close()                         关闭仿真

运行:
  python3 examples/01_hello_sim.py
"""
import time

from litearm_pybullet import PyBulletArm


def main():
    arm = PyBulletArm(render=True).connect()

    try:
        time.sleep(1.0)

        # litearm-python 2.0 起，"读一帧"的接口都返回 Msg 信封，值在 .value
        msg = arm.get_state()
        state = msg.value
        print("\n[仿真状态]")
        print("  q    (关节角 rad) =", [round(x, 3) for x in state.q])
        print("  dq   (关节速度)   =", [round(x, 3) for x in state.dq])
        print("  tau  (关节力矩)   =", [round(x, 3) for x in state.tau])
        print("  mode (状态机)     =", state.mode_name)
        print("  enabled           =", state.enabled, " faulted =", state.faulted)
        print("  hz   (帧到达频率) =", round(msg.hz, 1))
        print("  firmware          =", arm.firmware, "  n =", arm.n)

        tcp = arm.get_tcp()
        pos = tcp.value[:3]
        print("\n[末端位姿] pos =", [round(x, 4) for x in pos], "m",
              " rpy =", [round(x, 4) for x in tcp.value[3:]])

        print("\n观察 PyBullet 窗口，按 Ctrl+C 退出")
        while True:
            time.sleep(1)
            st = arm.get_state().value
            print(f"  q[0]={st.q[0]:.4f}  mode={st.mode_name}", end="\r")
    except KeyboardInterrupt:
        print("\n\n用户中断")
    finally:
        arm.close()
        print("[仿真] 已关闭")


if __name__ == "__main__":
    main()
