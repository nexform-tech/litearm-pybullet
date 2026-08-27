#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 03 · 轨迹录制与回放 — 录制仿真运动轨迹，然后回放

演示:
  arm.record_trajectory()            录制仿真运动轨迹
  arm.replay_trajectory(traj)        回放录制的轨迹
  traj.save("path.json")            保存轨迹到文件
  JointTrajectory.load("path.json") 从文件加载轨迹

运行:
  python3 examples/03_trajectory.py
"""
import time

from litearm_pybullet import PyBulletArm
from litearm_pybullet._litearm.types import JointTrajectory


def main():
    arm = PyBulletArm(render=True)
    arm.start()
    time.sleep(0.5)

    try:
        # ── 1. 先运动到舒展构型 ──
        print("[1] 运动到舒展构型")
        arm.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.3)

        # ── 2. 录制一段轨迹 ──
        print("\n[2] 开始录制轨迹 (3秒, 100Hz)")
        traj = arm.record_trajectory(duration_s=3.0, sample_rate_hz=100.0, name="demo")
        print(f"     录制完成: {len(traj.frames)} 帧")

        # ── 3. 保存到文件 ──
        traj.save("trajectories/demo.json")
        print("     已保存到 trajectories/demo.json")

        # ── 4. 从文件加载 ──
        loaded = JointTrajectory.load("trajectories/demo.json")
        print(f"     从文件加载: {len(loaded.frames)} 帧")

        # ── 5. 回放轨迹 ──
        print("\n[3] 回放轨迹...")
        arm.replay_trajectory(loaded, speed=1.0, goto_start=True)

        print("\n完成！按 Ctrl+C 退出...")
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        arm.close()
        print("[仿真] 已关闭")


if __name__ == "__main__":
    main()