# ESP32 主控板

这是本机器人项目最重要的一个子模块，它的职责是：

- 读取板载陀螺仪数据
- 运行所有运动控制算法，包括平衡算法、腿长控制、roll/yaw控制等
- 通过CAN通信获取电机状态并发送扭矩指令
- 通过低功耗蓝牙接收上位机的遥控指令

---

## 硬件说明

![电路设计](readme-img/design.png)

本模块硬件方案的特性：

- 芯片方案
	- 主控芯片：ESP32-C3
	- 陀螺仪：MPU6050
	- CAN驱动芯片：TJA1050T
- 供电电压：12V，降压方案：LDO
- 调试接口：USB Type-C，直连芯片引脚，可用于烧录程序、JTAG调试和串口通信

### 文件说明

- `ESP32CTRL_LCEDA_SCH.json`：立创EDA原理图文件
- `ESP32CTRL_LCEDA_PCB.json`：立创EDA PCB文件
- `ESP32CTRL_AD_SCH.schdoc`：立创EDA导出的 Altium Designer 原理图文件
- `ESP32CTRL_AD_PCB.pcbdoc`：立创EDA导出的 Altium Designer PCB文件
- `ESP32CTRL_SVG.svg`：原理图矢量图文件

---

## 软件说明

程序使用PlatformIO平台进行开发，使用ESP-IDF并将Arduino框架作为组件引入，因此可同时使用Arduino第三方库和ESP-IDF的底层API(如CAN通信)

为方便程序编写和调试，本程序使用ESP-IDF自带的FreeRTOS进行多任务调度，每个模块会触发一个或多个任务进行处理

### 文件说明

代码文件均位于`src`目录下：

- `main.cpp`：主要程序文件，包含所有任务模块的逻辑代码
- `PID.c/h`：PID控制器的实现，包含单级和串级PID控制器
- `debug.c/h`：用于[Linkscope](https://gitee.com/skythinker/link-scope)软件实现蓝牙无线调试，不参与正常运行，读者可忽略
- 由MATLAB直接生成的C代码文件，没有可读性，读者可参阅本项目MATLAB程序说明：
	- `leg_pos.c/h`：腿部位置解算函数
	- `leg_spd.c/h`：腿部运动速度解算函数
	- `leg_conv.c/h`：腿部输出换算函数
	- `lqr_k.c/h`：LQR反馈矩阵计算函数

电机、陀螺仪、CAN通信、蓝牙、运动控制等模块的代码全部位于`main.cpp`中（~~其实是偷懒没分开~~），不同模块的函数名基本都以模块名开头，读者可以按此规律参照注释查看

---

## 使用说明

### 烧录程序

使用USB线连接到电脑，按下两个按键后先松开RESET再松开BOOT，芯片即会进入烧录模式，此时可使用PlatformIO进行烧录

### 调试程序

程序中使用Arduinod的Serial类即可从USB输出串口信息，可以使用电脑上的串口调试软件进行查看

此外，ESP32C3的USB还支持JTAG调试，连接到电脑后即可使用[OpenOCD]([Linkscope](https://gitee.com/skythinker/link-scope))等程序连接，笔者在调试过程中搭配[Linkscope](https://gitee.com/skythinker/link-scope)实现在线读写变量和曲线绘制，较为方便

---

## 改进方向

- MPU6050的DMP频率只有200Hz，可以考虑更换其他芯片并编写姿态解算程序
- 硬件使用LDO进行降压，由于ESP32C3功耗较高，LDO发热量较大，可以修改为DC-DC降压
- 关键算法使用MATLAB符号化简结果直接导出，运算量非常大，几乎吃满了芯片性能，无法进一步提升控制频率，可以尝试手动化简后转为C代码
