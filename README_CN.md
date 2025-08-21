# AiKit 便捷套装

使用Python + Opencv 在机械臂上进行颜色识别、形状识别、特征点图像识别、AR二维码识别和 YOLOv5 图像识别，以及支持启动AiKit_UI图形化软件、手柄控制程序。

## 1 支持机械臂型号

- myCobot 280 M5
- myCobot 280 PI
- myPalletizer 260 M5
- myPalletizer 260 PI
- MechArm 270 M5
- MechArm 270 PI

## 2 运行环境

Linux 树莓派系统 + 套装组件 + USB一体化键盘鼠标手柄设备，末端执行器仅支持myCobot 垂直吸泵2.0

## 3 代码安装

```bash
cd ~
git clone -b Convenient_AiKit https://github.com/elephantrobotics/aikit_V2.git
git clone -b Convenient_AiKit https://github.com/elephantrobotics/AiKit_UI.git
```

## 4 依赖库安装

```bash
cd ~/aikit_V2
pip install -r requirements.txt
```

## 5 设置开机自启动

1. 将可执行脚本赋予可执行权限：

```bash
cd ~/aikit_V2
sudo chmod +x start_aikit*.sh
```

2. 将自启动脚本`.desktop文件`复制到`~/.config/autostart`文件夹中，不同设备对应不同的脚本。

- 280M5、270M5、260M5设备：
  
```bash
cd ~/aikit_V2
cp aikit_autostart.desktop $HOME/.config/autostart/aikit_autostart.desktop
```

- 280PI设备：

```bash
cd ~/aikit_V2
cp aikit_autostart_280PI.desktop $HOME/.config/autostart/aikit_autostart_280PI.desktop
```

- 270PI设备：

```bash
cd ~/aikit_V2
cp aikit_autostart_270PI.desktop $HOME/.config/autostart/aikit_autostart_270PI.desktop
```

- 260PI设备：

```bash
cd ~/aikit_V2
cp aikit_autostart_260PI.desktop $HOME/.config/autostart/aikit_autostart_260PI.desktop
```

3. 验证自启动

重启系统后，桌面会有终端弹出，如下：

- 280M5 、270M5、260M5设备：

    ![start](./img/auto_start_M5.png)

- 280/270/260 PI设备:

    ![start](./img/auto_start_PI.png)

此时代表程序开机自启动成功。

## 6 功能使用

支持启动颜色识别、形状识别、特征点图像识别、AR二维码识别、YOLOv5 图像识别、AiKit_UI程序、手柄控制程序。

![start](./img/auto_start_M5.png)

### 设备选择

根据终端信息提示，选择对应机械臂型号，如果是PI版本设备，则忽略此步骤。这里以270M5为例：

![start](./img/auto_start_M5-1.png)

### 按键功能说明

>> 注意：使用前需确保机械臂设备、USB相关设备已正确连接。

键盘按键输入：

`1`: 启动颜色识别功能。

`2`: 启动形状识别功能。

`3`: 启动AR二维码识别功能。

`4`: 启动特征点图像识别功能。

`5`: 启动yolov5识别功能。

`6`: 启动AiKit_UI图形化软件功能，可直接使用各种算法识别功能。

`7`: 启动手柄控制功能。启动后需将键鼠设备切换到手柄控制模式，手柄按键功能使用参考 [功能按键使用](https://docs.elephantrobotics.com/docs/mycobot_280_pi_cn/3-FunctionsAndApplications/6.developmentGuide/python/7.9_HandleControl.html)。

### 注意事项

1. 键盘按键 1 ~ 5 识别算法功能中的坐标抓取偏移量依赖Aikit_UI中的偏移量，所以调整坐标偏移量需输入按键 `6` 启动AiKit_UI程序进行修改保存。

2. 该程序支持识别算法功能直接切换，比如当前运行的是**颜色识别功能**，可直接输入`2`切换到**形状识别功能**。如果当前运行的是 **AiKit_UI**程序，需要在UI程序右上角手动关闭程序后， 才能按键输入切换其他识别算法功能。

3. 当摄像头没有正确自动框出识别区域，需要关闭程序，调整摄像头的位置，可将摄像头向左向右移动等操作。

4. OpenCV颜色识别会受环境的影响，若处在较为昏暗、光亮的环境下识别效果将大大降低。

5. 识别算法功能中，不同的功能对应不同的识别物料，请正确选择使用。

