"""
offset_utils.py
This module controls the robotic arm movements.

Author: Wang Weijian
Date: 2025-07-31
"""
# 文件路径：utils/offset_utils.py
import ast

def load_offset_from_txt(file_path, default=(190, 13, 124)):
    """
    从文本文件中读取相机偏移量，返回 (camera_x, camera_y, camera_z)
    文件内容应为：['190', '13', '124']
    """
    try:
        with open(file_path, 'r') as f:
            line = f.readline().strip()
            offset_list = ast.literal_eval(line)
            if len(offset_list) >= 2:
                camera_x = int(offset_list[0])
                camera_y = int(offset_list[1])
                camera_z = int(offset_list[2]) if len(offset_list) > 2 else default[2]
                return camera_x, camera_y, camera_z
    except Exception as e:
        print(f"[偏移量读取失败] {e}")
    return default
