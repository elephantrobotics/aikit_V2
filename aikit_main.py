"""
aikit_main.py
This module controls the robotic arm movements.

Author: Wang Weijian
Date: 2025-08-01
"""
with open("/home/er/aikit_log.txt", "a") as f:
    f.write("脚本在启动时运行了\n")

import subprocess
import sys
import os
from pynput import keyboard

current_process = None
device_name = None
device_key = None

DEVICE_MAP = {
    '1': ('AiKit_280M5', '280M5'),
    '2': ('AiKit_270M5', '270M5'),
    '3': ('AiKit_260M5', '260M5'),
}

# 脚本路径拼接函数
def get_script_path(script_name):
    if not device_name:
        print("请先选择设备！")
        return None
    return os.path.join('/home/er/aikit_V2', device_name, 'AiKit_280M5/scripts', script_name)

# 启动脚本函数
def run_script(script_path, use_sudo=False):
    global current_process

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

# 按键响应
def on_press(key):
    global current_process

    try:
        if hasattr(key, 'char'):
            if key.char == '1':
                run_script(get_script_path('aikit_color.py'), use_sudo=False)
            elif key.char == '2':
                print("此形状识别功能已被移除或未定义")
            elif key.char == '3':
                run_script(get_script_path('aikit_encode.py'), use_sudo=False)
            elif key.char == '4':
                run_script(get_script_path('aikit_img.py'), use_sudo=False)
            elif key.char == '5':
                run_script(get_script_path('yolov5_img.py'), use_sudo=False)
            elif key.char == '6':
                run_script('/home/er/AiKit_UI/main.py', use_sudo=False)
            elif key.char == '7':
                handle_path = os.path.join('/home/er/aikit_V2/handle_control', f'{device_key}_wireless_keyboard_mouse_handle_control_raspi_linux.py')
                run_script(handle_path, use_sudo=False)

        elif key == keyboard.Key.esc:
            print("退出监听")
            if current_process is not None and current_process.poll() is None:
                print("终止当前算法脚本...")
                current_process.terminate()
                current_process.wait()
            return False

    except Exception as e:
        print(f"按键监听出错: {e}")
        return False

# 主程序
if __name__ == '__main__':
    print("请选择设备：")
    print("1 - myCobot 280 M5")
    print("2 - MechArm 270 M5")
    print("3 - MyPalletizer 260 M5")
    device_input = input("输入设备编号（1/2/3）：").strip()

    if device_input not in DEVICE_MAP:
        print("无效的设备编号！程序退出。")
        sys.exit(1)

    device_name, device_key = DEVICE_MAP[device_input]
    print(f"当前选择设备: {device_key}")

    print("等待键盘输入 (1-5: 识别算法功能, 6: 启动AiKit_UI, 7: 启动手柄控制)，按 Esc 退出")
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()