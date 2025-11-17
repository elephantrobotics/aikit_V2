"""
common.py
This module controls the robotic arm movements.

Author: Wang Weijian
Date: 2025-11-17
"""
# common.py

def clamp(value, low, high):
    return max(low, min(high, value))

def limit_coords(coords):
    LIMITS = {
        0: (-350, 350),   # x
        1: (-350, 350),   # y
        2: (-41, 523.9),  # z
        3: (-180, 180),   # rx
        4: (-180, 180),   # ry
        5: (-180, 180),   # rz
    }

    fixed = []
    for i, v in enumerate(coords):
        low, high = LIMITS[i]
        nv = clamp(v, low, high)
        fixed.append(nv)

        if nv != v:
            print(f"⚠️ 轴 {i} = {v} 超限，已限制到 {nv}")

    return fixed
