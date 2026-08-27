#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 01 · 独立仿真 — 创建仿真机械臂并读取状态（只读，不运动）

演示:
  PyBulletArm(render=True)    创建仿真并打开可视化窗口
  arm.get_state()             读取当前状态（q / dq / tau / state）
  arm.get_tcp_pose()          当前末端位姿
  arm.close()                 关闭仿真

运行:
  python3 examples/01_hello_sim.py
"""
import time

from litearm_pybullet import PyBulletArm


def main():
    arm = PyBulletArm(render=True)
    arm.start()

    try:
        time.sleep(1.0)

        state = arm.get_state()
        print("\n[仿真状态]")
        print("  q    (关节角 rad) =", [round(x, 3) for x in state["q"]])
        print("  dq   (关节速度)   =", [round(x, 3) for x in state["dq"]])
        print("  tau  (关节力矩)   =", [round(x, 3) for x in state["tau"]])
        print("  state(状态机)     =", state["state"])
        print("  serial            =", state["robot_serial"])

        pos, R = arm.get_tcp_pose()
        print("\n[末端位姿] pos =", [round(x, 4) for x in pos], "m")

        print("\n观察 PyBullet 窗口，按 Ctrl+C 退出")
        while True:
            time.sleep(1)
            state = arm.get_state()
            print(f"  q[0]={state['q'][0]:.4f}  state={state['state']}", end="\r")
    except KeyboardInterrupt:
        print("\n\n用户中断")
    finally:
        arm.close()
        print("[仿真] 已关闭")


if __name__ == "__main__":
    main()