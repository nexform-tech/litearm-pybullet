#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 02 · 仿真运动 — 关节空间运动 movej + 笛卡尔运动 movel + FK/IK

演示:
  arm.movej(q_target, speed=...)     关节空间点到点运动
  arm.movel(pose_goal, speed=...)    笛卡尔直线运动
  arm.fk(q) / arm.ik(pos, R)         正逆运动学

运行:
  python3 examples/02_movej_sim.py
"""
import time

import numpy as np

from litearm_pybullet import PyBulletArm


# 一个舒展、远离奇异的构型
Q_HOME = [0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0]


def main():
    arm = PyBulletArm(render=True)
    arm.start()

    try:
        time.sleep(0.5)

        # ── 1. 关节空间运动 ──
        print("\n[1] movej → 舒展构型 (speed=0.3)")
        arm.movej(Q_HOME, speed=0.3)
        time.sleep(0.5)

        print("[2] movej → 回零位 (speed=0.3)")
        arm.movej([0.0] * 7, speed=0.3)
        time.sleep(0.5)

        # ── 2. 正运动学 ──
        print("\n[3] 正运动学 fk(Q_HOME)")
        pos, R = arm.fk(Q_HOME)
        print(f"     pos = {[round(x, 4) for x in pos]}")
        print(f"     R   = {[[round(x, 4) for x in row] for row in R]}")

        # ── 3. 逆运动学 ──
        print("\n[4] 逆运动学 ik(pos, R)")
        q_sol, success = arm.ik(pos, R)
        print(f"     success = {success}")
        if success:
            print(f"     q_sol   = {[round(x, 4) for x in q_sol]}")
            print(f"     error   = {np.max(np.abs(np.array(q_sol) - np.array(Q_HOME))):.6f}")

        # ── 4. 笛卡尔直线运动 ──
        print("\n[5] movel → 沿 Z 轴下降 0.1m (speed=0.2)")
        pos_current, R_current = arm.get_tcp_pose()
        pos_target = [pos_current[0], pos_current[1], pos_current[2] - 0.1]
        arm.movel([pos_target, R_current], speed=0.2)

        print("\n所有运动完成")
        print("按 Ctrl+C 退出...")
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        arm.close()
        print("[仿真] 已关闭")


if __name__ == "__main__":
    main()