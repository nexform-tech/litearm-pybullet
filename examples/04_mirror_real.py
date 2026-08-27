#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 04 · 镜像模式 — 仿真跟随真实机械臂同步运动

前提:
  1. 机械臂控制器上 litearm-server 已启动
  2. 客户端与控制器网络互通
  3. 客户端已安装 litearm-python (pip install litearm-pybullet[mirror])

运行:
  python3 examples/04_mirror_real.py --endpoint tcp/192.168.31.139:7447
"""
import argparse
import time

from litearm_pybullet import PyBulletArm, litearm


def main():
    ap = argparse.ArgumentParser(description="仿真镜像真实机械臂")
    ap.add_argument("--endpoint", default="tcp/192.168.31.139:7447",
                    help="litearm-server 的 zenoh 端点")
    ap.add_argument("--arm-id", default="armA", help="Arm 标识")
    args = ap.parse_args()

    # 连接真实机械臂
    real = litearm.Arm(endpoint=args.endpoint, arm_id=args.arm_id)
    print(f"[实臂] 已连接 · endpoint={args.endpoint}")

    # 创建仿真
    sim = PyBulletArm(render=True)
    sim.start()

    try:
        time.sleep(1.0)
        real_state = real.get_state()
        if real_state is None:
            print("[实臂] 未收到状态，检查 server 是否在运行")
            return

        q_real = real_state["q"]
        print(f"[实臂] 初始关节角: {[round(x, 3) for x in q_real]}")

        # 将仿真初始化为实臂当前姿态
        sim.set_joint_positions(q_real)
        sim._controller.set_target(q_real)

        # 开始镜像
        sim.mirror_from(real)
        print("\n镜像模式已启动 — 仿真跟随实臂运动")
        print("   在实臂上执行操作（拖动/运动），观察仿真同步")
        print("   按 Ctrl+C 退出\n")

        while True:
            time.sleep(1)
            r_state = real.get_state()
            s_state = sim.get_state()
            if r_state and s_state:
                err = max(abs(r_state["q"][i] - s_state["q"][i])
                          for i in range(7))
                print(f"  实臂 q[0]={r_state['q'][0]:.4f}  "
                      f"仿真 q[0]={s_state['q'][0]:.4f}  "
                      f"误差={err:.4f} rad", end="\r")

    except KeyboardInterrupt:
        print("\n\n用户中断")
    finally:
        sim.stop_mirroring()
        sim.close()
        real.close()
        print("[实臂+仿真] 已关闭")


if __name__ == "__main__":
    main()