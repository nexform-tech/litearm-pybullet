#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 05 · 双控模式 — 同时控制真实机械臂和仿真

同一条指令发给两边：可以先在仿真上确认动作，再让它落到实臂上。

前提:
  1. 机械臂已通过 USB 接上（CDC 串口）
  2. 客户端已装 litearm-python（未上 PyPI，从源码装）:
       pip install -e ../litearm-python

运行:
  python3 examples/05_dual_control.py
  python3 examples/05_dual_control.py --port /dev/ttyACM0

互动模式（演示后保持窗口，可输入关节目标）:
  python3 examples/05_dual_control.py --interactive
"""
import argparse
import time

from litearm_pybullet import DualArm

Q_HOME = [0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0]


def _show(label, result):
    """movej 的返回值是 (实臂状态, 仿真状态)，两边都是 RobotState。

    （笛卡尔的 move_l/move_c/move_path 返回的是 (CartPlan, CartPlan)。）
    """
    real, sim = result
    print(f"     {label}: 实臂 mode={real.mode_name} "
          f"q[1]={real.q[1]:.3f} | 仿真 mode={sim.mode_name} q[1]={sim.q[1]:.3f}")


def main():
    ap = argparse.ArgumentParser(description="双控：实臂+仿真同时运动")
    ap.add_argument("--port", default=None,
                    help="真臂串口 (缺省: 自动查找唯一的 STM32 CDC 设备)")
    ap.add_argument("--interactive", "-i", action="store_true",
                    help="演示后保持仿真窗口，输入关节目标交互控制")
    args = ap.parse_args()

    dual = DualArm(real_port=args.port, render=True, mirror_first=True)
    dual.start()

    try:
        # 使能是**显式**的：构造函数不会给实体机械臂上电。
        # 这一行就是"从这里开始，实臂会动"的那一行，看得见才好 review
        print("[双控] 使能实臂 + 仿真 ...")
        dual.enable()

        time.sleep(1.0)
        state = dual.get_real_state().value
        if state is None:
            print("[实臂] 未收到状态，检查串口/固件")
            return
        print(f"[实臂] 当前关节角: {[round(x, 3) for x in state.q]}")

        # ── 双控运动 ──
        print("\n[1] 双控 movej → 舒展构型 (speed=0.2)")
        _show("结果", dual.movej(Q_HOME, speed=0.2))

        time.sleep(0.5)

        print("\n[2] 双控 movej → 回零位 (speed=0.2)")
        _show("结果", dual.movej([0.0] * 7, speed=0.2))

        print("\n双控运动完成")

        # ── 互动模式 ──
        if args.interactive:
            print("\n" + "=" * 60)
            print("互动模式：仿真窗口保持打开，你可输入关节目标控制实臂+仿真")
            print("   输入 7 个关节角 (rad)，如: 0 0.6 0 -1.2 0 0.7 0")
            print("   输入 'q' 退出, 输入 'home' 回舒展构型, 输入 'zero' 回零位")
            print("=" * 60)
            while True:
                try:
                    cmd = input("\n> ").strip()
                except (EOFError, KeyboardInterrupt):
                    print()
                    break
                if not cmd:
                    continue
                if cmd.lower() == 'q':
                    break
                if cmd.lower() == 'home':
                    q = Q_HOME
                elif cmd.lower() == 'zero':
                    q = [0.0] * 7
                else:
                    parts = cmd.split()
                    if len(parts) != 7:
                        print("  需要 7 个数字（7 个关节角），用空格分隔")
                        continue
                    try:
                        q = [float(p) for p in parts]
                    except ValueError:
                        print("  无法解析，请用空格分隔的数字")
                        continue
                print(f"  movej -> {[round(x, 2) for x in q]}  speed=0.15 ...")
                _show("结果", dual.movej(q, speed=0.15))
        else:
            print("\n仿真窗口保持打开，按 Enter 退出 ...")
            try:
                input()
            except (EOFError, KeyboardInterrupt):
                print()

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        try:
            # 双控里两边都要收力；实臂优先，仿真那边只是本进程的状态
            dual.emergency_stop()
        except Exception:
            pass
        dual.close()
        print("[双控] 已关闭")


if __name__ == "__main__":
    main()
