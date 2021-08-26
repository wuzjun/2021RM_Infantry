# Infantry（步兵代码）
## 功能介绍

本代码为 2021RM 步兵嵌入式代码，该代码包含了底盘，云台，发射等功能。
功能名称|功能说明
-|-
底盘|底盘是基于麦克纳姆轮的运动模型，在全向移动的基础下，拥有跟随模式，反向跟随模式，自旋模式，扭腰模式。
云台|云台两轴均优先采用陀螺仪闭环控制，并能防止了云台的数据紊乱。同时配备了视觉自瞄，跟随目标装甲板以及激活能量机关的效果。
发射|发射机构的电控程序上做了防卡弹处理，通过翻转拨盘电机，能做到很好的防卡弹效果。
检测|设备检测程序，检测所有设备是否处于正常运作状态，并且通过OLED将相关信息显示出来，便于我们能够以最快的速度定位故障。

## 效果展示

### 在本赛季的步兵机器人中，我们把重心放在了云台突破上，以提高响应以及稳定性为主。
我们参考了论坛上的一些开源，采用一阶卡尔曼滤波用于消除低频抖动。
![image](https://github.com/wuzjun/2021RM_Infantry/raw/master/Photo/Kalman1.png)

二阶卡尔曼滤波+预测融合用于自瞄
![image](https://github.com/wuzjun/2021RM_Infantry/raw/master/Photo/Kalman2.png)

## 依赖工具及软硬件环境

`工具:`VSCode，Keil5

`软件环境:`Windows10

`硬件环境:`STM32F427

## 编译方式

C/C++编译

## 文件层次

* .vscode (VScode配置)
* Algorithm (各种算法层)
* Apps (用户控制层)
* BSP (用户定义外设层)
* Devices (外部设备层)
* Drivers (HAL库驱动层)
* Inc (主函数和中断层头文件)
* MDK-ARM (keil工程文件和编译文件)
* Middlewares (Freertos层)
* Src (主函数和中断层源文件)
* Tasks (用户任务层)
* User (用户头文件层)

## 系统框图

![image](https://github.com/wuzjun/2021RM_Infantry/raw/master/Photo/Infantry_System.png)

## 软件架构

![image](https://github.com/wuzjun/2021RM_Infantry/raw/master/Photo/Infantry_Software.png)

## 未来优化的方向

1.在底盘控制方面，在功率控制这一块还有可提升的空间，以功率的利用效率最大化为目标进行优化，较少功率的浪费。

2.在云台控制方面，可对响应度以及稳定性进行优化，从而给自瞄的突破打好基础。

3.在键鼠操作方面，尽可能减少操作手的误操作所来带的不利影响，以及优化半自动控制，为操作手提供更便捷的操作。