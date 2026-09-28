#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 02 · 仿真运动 — 关节空间 movej + 笛卡尔 move_l + FK/IK

演示:
  arm.movej(q_target, speed=...)     关节空间点到点运动（到位才返回）
  arm.move_l(pose, speed=...)        笛卡尔直线运动，pose = xyz + rpy
  arm.fk(q) / arm.ik(pose)           正逆运动学（仿真独有 / 与 SDK 同形）

运行:
  python3 examples/02_movej_sim.py
"""
import time

import numpy as np

from litearm_pybullet import PyBulletArm
from litearm_pybullet._compat import mat_to_rpy

# 一个舒展、远离奇异的构型
Q_HOME = [0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0]


def main():
    arm = PyBulletArm(render=True).connect()

    try:
        time.sleep(0.5)

        # ── 1. 关节空间运动。`speed` 是 0..1 的倍率，不是百分比 ──
        print("\n[1] movej → 舒展构型 (speed=0.3)")
        arm.movej(Q_HOME, speed=0.3)
        time.sleep(0.5)

        print("[2] movej → 回零位 (speed=0.3)")
        arm.movej([0.0] * 7, speed=0.3)
        time.sleep(0.5)

        # ── 2. 正运动学。fk 是仿真独有的：SDK 只回固件当前反馈的 FK ──
        print("\n[3] 正运动学 fk(Q_HOME)")
        pos, R = arm.fk(Q_HOME)
        print(f"     pos = {[round(x, 4) for x in pos]}")
        print(f"     R   = {[[round(x, 4) for x in row] for row in R]}")

        # ── 3. 逆运动学。ik 收 xyz+rpy，无解抛 IKError ──
        print("\n[4] 逆运动学 ik(pos + rpy)")
        pose = list(pos) + mat_to_rpy(R)
        q_sol = arm.ik(pose)
        print(f"     q_sol = {[round(x, 4) for x in q_sol]}")
        print(f"     error = {np.max(np.abs(np.array(q_sol) - np.array(Q_HOME))):.6f}")

        # ── 4. 笛卡尔直线运动 ──
        print("\n[5] move_l → 沿 Z 轴下降 0.1m (speed=0.2)")
        cur = arm.get_tcp().value
        target = [cur[0], cur[1], cur[2] - 0.1] + list(cur[3:])
        plan = arm.move_l(target, speed=0.2)
        # CartPlan.ok 只说明"规划并下发成功"，不说明"停在目标上"
        print(f"     ok={plan.ok} n_wp={plan.n_wp} settled={plan.settled}")

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
