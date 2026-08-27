#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""样例 05 · 双控模式 — 同时控制真实机械臂和仿真

前提:
  1. 机械臂控制器上 litearm-server 已启动
  2. 客户端与控制器网络互通
  3. 客户端已安装 litearm-python (pip install litearm-pybullet[mirror])

运行:
  python3 examples/05_dual_control.py --endpoint tcp/192.168.31.139:7447

互动模式（演示后保持窗口，可输入关节目标）:
  python3 examples/05_dual_control.py --endpoint tcp/192.168.31.139:7447 --interactive
"""
import argparse
import time

from litearm_pybullet import DualArm


def main():
    ap = argparse.ArgumentParser(description="双控：实臂+仿真同时运动")
    ap.add_argument("--endpoint", default="tcp/192.168.31.139:7447",
                    help="litearm-server 的 zenoh 端点")
    ap.add_argument("--interactive", "-i", action="store_true",
                    help="演示后保持仿真窗口，输入关节目标交互控制")
    args = ap.parse_args()

    dual = DualArm(
        real_endpoint=args.endpoint,
        render=True,
        mirror_first=True,
    )
    dual.start()

    try:
        time.sleep(1.0)

        real_state = dual.get_real_state()
        if real_state is None:
            print("[实臂] 未收到状态，检查 server 是否在运行")
            return

        print(f"[实臂] 当前关节角: {[round(x, 3) for x in real_state['q']]}")

        # ── 双控运动 ──
        print("\n[1] 双控 movej → 舒展构型 (speed=0.2)")
        Q_HOME = [0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0]
        real_ok, sim_ok = dual.movej(Q_HOME, speed=0.2)
        print(f"     实臂: {'OK' if real_ok else 'FAIL'}, 仿真: {'OK' if sim_ok else 'FAIL'}")

        time.sleep(0.5)

        print("\n[2] 双控 movej → 回零位 (speed=0.2)")
        real_ok, sim_ok = dual.movej([0.0] * 7, speed=0.2)
        print(f"     实臂: {'OK' if real_ok else 'FAIL'}, 仿真: {'OK' if sim_ok else 'FAIL'}")

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
                r_ok, s_ok = dual.movej(q, speed=0.15, settle_s=0.8)
                print(f"  实臂: {'OK' if r_ok else 'FAIL'}, 仿真: {'OK' if s_ok else 'FAIL'}")
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
            dual.request_stop()
        except Exception:
            pass
        dual.close()
        print("[双控] 已关闭")


if __name__ == "__main__":
    main()