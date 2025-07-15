"""
main_launcher.py
This module controls the robotic arm movements.

Author: Wang Weijian
Date: 2025-07-15
"""
import subprocess
import sys

import keyboard  # pip install keyboard
import os
import time

# 记录当前运行的子进程（算法）
current_process = None

def run_script(script_path):
    global current_process

    # 如果已有算法在运行，先终止
    if current_process is not None and current_process.poll() is None:
        print("终止当前算法进程...")
        current_process.terminate()
        current_process.wait()

    # 启动新的算法脚本
    print(f"启动脚本: {script_path}")
    current_python = sys.executable  # 自动获取当前运行的 Python 路径
    print(f"用当前虚拟环境启动：{current_python}")
    current_process = subprocess.Popen([current_python, script_path], creationflags=subprocess.CREATE_NEW_CONSOLE)

def keyboard_listener():
    print("等待键盘输入 (1-4)，按 Esc 退出")

    while True:
        try:
            if keyboard.is_pressed('1'):
                run_script('scripts/aikit_color.py')
                time.sleep(0.5)
            elif keyboard.is_pressed('2'):
                run_script('scripts/aikit_shape.py')
                time.sleep(0.5)
            elif keyboard.is_pressed('3'):
                run_script('scripts/aikit_encode.py')
                time.sleep(0.5)
            elif keyboard.is_pressed('4'):
                run_script('scripts/aikit_img.py')
                time.sleep(0.5)
            elif keyboard.is_pressed('5'):
                run_script('scripts/yolov5_img.py')
                time.sleep(0.5)
            elif keyboard.is_pressed('esc'):
                print("退出监听")
                # 🛑 如果当前还有子进程在运行，终止它
                if current_process is not None and current_process.poll() is None:
                    print("终止当前算法脚本...")
                    current_process.terminate()
                    current_process.wait()
                break
        except:
            break

if __name__ == '__main__':
    keyboard_listener()
