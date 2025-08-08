"""
280PI_aikit_main.py
This module controls the robotic arm movements.

Author: Wang Weijian
Date: 2025-08-01
"""
with open("/home/er/aikit_log.txt", "a") as f:
    f.write("脚本在启动时运行了\n")

import subprocess
import sys
import os
import time
from pynput import keyboard

current_process = None
in_ui_mode = False  # UI 模式状态
last_ui_exit_time = 0

# 固定路径（适配当前设备）
BASE_DIR = "/home/er/aikit_V2/AiKit_260PI/scripts"  # ← 替换 XXX 为当前机型
HANDLE_DIR = "/home/er/aikit_V2/handle_control"
UI_PATH = "/home/er/AiKit_UI/main.py"
DEVICE_KEY = "280PI"  # ← 替换为当前机型编号

# 启动脚本函数
def run_script(script_path, use_sudo=False):
    global current_process, in_ui_mode, last_ui_exit_time

    if current_process is not None and current_process.poll() is None:
        print("终止当前算法进程...")
        current_process.terminate()
        current_process.wait()

    if not script_path:
        return

    print(f"启动脚本: {script_path}")
    current_python = sys.executable
    if use_sudo:
        current_process = subprocess.Popen(['sudo', current_python, script_path])
    else:
        current_process = subprocess.Popen([current_python, script_path])

# 按键监听响应
def on_press(key):
    global current_process, in_ui_mode, last_ui_exit_time

    try:
        # 在 UI 退出后的 0.5 秒内忽略所有按键
        if time.time() - last_ui_exit_time < 0.5:
            # print("忽略 UI 退出瞬间的按键残留")
            return

        if hasattr(key, 'char'):
            # UI 模式下屏蔽算法切换
            if in_ui_mode and key.char in ['1', '2', '3', '4', '5', '7']:
                # print("当前在 UI 模式，忽略数字键输入")
                return
            if key.char == '1':
                run_script(os.path.join(BASE_DIR, 'aikit_color.py'))
            elif key.char == '2':
                run_script(os.path.join(BASE_DIR, 'aikit_shape.py'))
            elif key.char == '3':
                run_script(os.path.join(BASE_DIR, 'aikit_encode.py'))
            elif key.char == '4':
                run_script(os.path.join(BASE_DIR, 'aikit_img.py'))
            elif key.char == '5':
                run_script(os.path.join(BASE_DIR, 'yolov5_img.py'))
            elif key.char == '6':
                in_ui_mode = True
                run_script(UI_PATH)
                if current_process:
                    current_process.wait()
                in_ui_mode = False
                last_ui_exit_time = time.time()  # 记录 UI 退出时间
                print("UI 模式结束，恢复数字键切换功能")
            elif key.char == '7':
                handle_script = os.path.join(HANDLE_DIR, f"{DEVICE_KEY}_wireless_keyboard_mouse_handle_control_raspi_linux.py")
                run_script(handle_script)
            else:
                print(f"无效按键：{key.char}，请按 1-7 或 Esc")

        elif key == keyboard.Key.esc:
            print("退出监听")
            if current_process is not None and current_process.poll() is None:
                print("终止当前算法脚本...")
                current_process.terminate()
                current_process.wait()
            return False
        else:
            print(f"忽略特殊按键：{key}")

    except Exception as e:
        print(f"按键监听出错: {e}")
        return False

# 主程序
if __name__ == '__main__':
    print("等待键盘输入 (1-5: 识别算法功能, 6: 启动AiKit_UI, 7: 启动手柄控制)，按 Esc 退出")
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()
