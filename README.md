# Balancar

## 项目简介
通过Android端的重力感应和语音识别控制小车，采用PID算法实现自平衡，用蓝牙进行无线调参和控制。该项目旨在提供一个低成本、高效的自平衡小车方案，适用于教育和研究领域。

- 使用互补滤波器融合mpu6050的陀螺仪和加速度计得到IMU姿态的估计，减小温漂和抖动的干扰。
- 使用外部中断检测编码器的圈数，结合定时中断计算转速从而得到里程计位置和速度的估计。
- 对姿态环和速度环进行串级PID闭环运动控制，通过Android端实时无线调参。

该项目依托单片机实验室，硬件及软件设计均由我一人独立完成，作为《单片机原理与接口》的课程设计，成绩为95分。

<div align="center">
  <img src=Docs/Images/小车酷照.jpg width="30%"/>
</div>

## 硬件部分
本项目使用的主要硬件组件如下：

- **MPU6050**: 用于获取小车的姿态数据。
- **编码器**: 用于检测小车的行驶距离和速度。
- **Arduino Nano**: 作为主控板，负责数据处理和控制。
- **蓝牙模块**: 用于实现与Android端的无线通信。

<div align="center">
  <img alt=小车硬件清单.png src=Docs/Images/小车硬件清单.png/>
  <img alt=nano.png src=Docs/Images/nano.png width="30%"/>
  <img alt=Balancar_fritzing_bb.png src=Docs/Images/Balancar_fritzing_bb.png width="40%"/>
</div>

## 软件部分
项目的软件部分包括：

- **Arduino IDE**: 用于编写和上传控制小车的代码 (`Code.ino`)。
- **APK**: Android端的应用程序 (`Balancar.aia`)，用于控制和调参。

<div align="center">
  <img alt=平衡小车的软件程序流程图.PNG src=Docs/Images/平衡小车的软件程序流程图.PNG width="80%"/>
</div>

## 调参测试
在使用PID算法调试小车的过程中，通过调整参数来实现最佳的自平衡效果。

<div align="center">
  <img alt=PID原理图.PNG src=Docs/Images/PID原理图.PNG width="80%/>
</div>

<div align="center">
  <img alt=程序运行数据_offset.PNG src=Docs/Images/程序运行数据_offset.PNG width="80%"/>
  <img alt=P=30_D=0_P=0.PNG src=Docs/Images/P%3D30_D%3D0_P%3D0.PNG width="30%"/>
  <img alt=P=30_D=1.9_P=0.PNG src=Docs/Images/P%3D30_D%3D1.9_P%3D0.PNG width="30%"/>
  <img alt=P=30_D=1.9_P=-0.3.PNG src=Docs/Images/P%3D30_D%3D1.9_P%3D-0.3.PNG width="30%"/>
</div>

## 演示视频
- [Part 1](Docs/Videos/Balancar_video_1.mp4): 系统简介和硬件部分。
- [Part 2](Docs/Videos/Balancar_video_2.mp4): 软件部分和调参测试。
