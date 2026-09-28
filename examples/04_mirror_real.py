#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 04 · 镜像模式 — 仿真跟随真实机械臂同步运动

前提:
  1. 机械臂已通过 USB 接上（CDC 串口）
  2. 客户端已装 litearm-core（未上 PyPI，从源码装）:
       pip install -e ../litearm-core

运行:
  python3 examples/04_mirror_real.py                 # 自动找 CDC 串口
  python3 examples/04_mirror_real.py --port /dev/ttyACM0
"""
import argparse
import time

import litearm_core as pa

from litearm_pybullet import PyBulletArm


def main():
    ap = argparse.ArgumentParser(description="仿真镜像真实机械臂")
    ap.add_argument("--port", default=None,
                    help="真臂串口 (缺省: 自动查找唯一的 STM32 CDC 设备)")
    ap.add_argument("--rate-hz", type=float, default=50.0,
                    help="镜像轮询频率 (每帧都要过一次串口, 别调太高)")
    args = ap.parse_args()

    # 连接真实机械臂。litearm-core 直连固件，没有 server/endpoint 这一层
    real = pa.Arm(port=args.port).connect()
    print(f"[实臂] 已连接 · port={args.port or '(自动查找)'} "
          f"firmware={real.firmware} n={real.n}")

    # 创建仿真
    sim = PyBulletArm(render=True).connect()

    try:
        # 开始镜像：把实臂当前姿态搬进仿真，然后持续跟随
        sim.mirror_from(real, rate_hz=args.rate_hz)
        # 镜像在自己的线程里跑，失败不会抛到这里；等它试过第一帧再看
        time.sleep(0.5)
        if sim.mirror_error is not None:
            print(f"[镜像] 没拿到实臂状态: {sim.mirror_error}")
            print("       检查串口/固件，或 --rate-hz 调低")
            return

        print("\n镜像模式已启动 — 仿真跟随实臂运动")
        print("   在实臂上执行操作（拖动/运动），观察仿真同步")
        print("   按 Ctrl+C 退出\n")

        while True:
            time.sleep(1)
            # refresh=True: 每次都要"此刻"的一帧。refresh=False 回的是本调用方
            # 上一次读到的那帧，镜像里用它只会一直打印同一个姿态
            r_msg = real.get_state(refresh=True)
            s_msg = sim.get_state()
            if r_msg.value is None:
                print(f"  实臂无回帧 (mirror_error={sim.mirror_error})", end="\r")
                continue
            q_real, q_sim = r_msg.value.q, s_msg.value.q
            err = max(abs(q_real[i] - q_sim[i]) for i in range(min(len(q_real),
                                                                   len(q_sim))))
            print(f"  实臂 q[0]={q_real[0]:.4f}  "
                  f"仿真 q[0]={q_sim[0]:.4f}  "
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
