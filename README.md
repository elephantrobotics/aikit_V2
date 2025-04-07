# aikit_V2.0

# 1.普通机型

适用机型：

- myCobot 280 M5
- myCobot 280 PI
- myCobot 280 JetsonNano
- myCobot 320 M5
- myCobot 320 PI
- myPalletizer 260系列
- MechArm 270 系列
- ultraArm P340
- myArm 300 PI

## 介绍
使用纯python进行机械臂的颜色识别、特征点图像识别、形状识别、二维码识别、yolov5图像识别。

## 使用前安装

- 机械臂驱动库

```bash
pip install pymycobot --upgrade
```

- python opencv支持库

```bash
# 二者版本号需一致，建议使用4.6.0.66版本
pip install opencv-python==4.6.0.66
pip install opencv-contrib-python==4.6.0.66
```

## 安装

```bash
git clone https://github.com/elephantrobotics/aikit_V2.git
```
# 2. RISCV机型

适用机型：

- myCobot 280 RISCV

## 介绍
使用纯python进行机械臂的颜色识别、特征点图像识别、形状识别、二维码识别、yolov8图像识别。

## 使用前安装

- 创建虚拟环境

```bash
sudo apt install python3-virtualenv
virtualenv elephantics-venv
source elephantics-venv/bin/activate
```

- 安装依赖项

```bash
sudo apt install libopenblas-dev
```

## 安装

```bash
git clone https://github.com/elephantrobotics/aikit_V2.git
```

## 安装python依赖库

```bash
cd ~/aikit_V2/AiKit_280RISCV
pip install -r requirements.txt
```