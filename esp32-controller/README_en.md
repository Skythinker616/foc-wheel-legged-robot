# ESP32 Main Controller Board

**[简体中文](README.md) | English**

This is the most crucial submodule of the robot project, and its responsibilities include:

- Reading onboard gyroscope data
- Running all motion control algorithms, including balance algorithms, leg length control, roll/yaw control, etc.
- Obtaining motor status via CAN communication and sending torque commands
- Receiving remote control commands from the upper computer through low-power Bluetooth

---

## Hardware Description

![Circuit Design](readme-img/design.png)

Features of this module's hardware solution:

- Chip Solution
  - Main control chip: ESP32-C3
  - Gyroscope: MPU6050
  - CAN driver chip: TJA1050T
- Power Supply Voltage: 12V, voltage reduction solution: LDO
- Debugging Interface: USB Type-C, directly connected to the chip pins, used for program burning, JTAG debugging, and serial communication

### File Description

Hardware-related files are located in the `hardware` directory:

- `ESP32CTRL_LCEDA_SCH.json`: LCEDA schematic file
- `ESP32CTRL_LCEDA_PCB.json`: LCEDA PCB file
- `ESP32CTRL_AD_SCH.schdoc`: Altium Designer schematic file exported from LCEDA
- `ESP32CTRL_AD_PCB.pcbdoc`: Altium Designer PCB file exported from LCEDA
- `ESP32CTRL_SVG.svg`: Schematic vector file

---

## Software Description

The program is developed using the PlatformIO platform and uses ESP-IDF, incorporating the Arduino framework as a component. Therefore, it can utilize both Arduino third-party libraries and ESP-IDF's low-level APIs (such as CAN communication).

For ease of program writing and debugging, this program utilizes ESP-IDF's built-in FreeRTOS for multitasking scheduling, and each module will trigger one or more tasks for processing.

### File Description

The PlatformIO project is located in the `software` directory, and the code files are located in the `src` directory:

- `main.cpp`: Main program file, containing the logical code for all task modules
- `PID.c/h`: Implementation of the PID controller, including single-loop and cascade PID controllers
- `debug.c/h`: Used for wireless Bluetooth debugging with [Linkscope](https://gitee.com/skythinker/link-scope) software, not involved in normal operation, readers can ignore it
- C code files generated directly from MATLAB, which are not readable. Readers can refer to the MATLAB program explanation in this project for details:
  - `leg_pos.c/h`: Leg position calculation function
  - `leg_spd.c/h`: Leg motion speed calculation function
  - `leg_conv.c/h`: Leg output conversion function
  - `lqr_k.c/h`: LQR feedback matrix calculation function

The code for motors, gyroscopes, CAN communication, Bluetooth, motion control, and other modules is all located in `main.cpp` (~~I was a bit lazy and didn't separate them~~). The function names of different modules generally start with the module name, and readers can refer to the comments following this pattern to check them.

---

## Instructions for Use

### Program Uploading

Connect the board to the computer via USB, press both buttons, release RESET first, and then release BOOT. The chip will enter the uploading mode, and you can use PlatformIO for uploading.

### Program Debugging

The program uses Arduino's Serial class to output serial information via USB, which can be viewed using serial debugging software on the computer.

In addition, ESP32C3's USB also supports JTAG debugging. After connecting it to the computer, you can use programs like [OpenOCD]([Linkscope](https://gitee.com/skythinker/link-scope)) to connect. During debugging, I combined it with [Linkscope](https://gitee.com/skythinker/link-scope) for online variable reading, writing, and curve drawing, which is quite convenient.

---

## Future Improvements

- The DMP frequency of MPU6050 is only 200Hz; consider replacing it with other chips and writing attitude calculation programs.
- The hardware uses an LDO for voltage reduction, but due to the high power consumption of ESP32C3, the LDO generates a lot of heat. Consider changing to a DC-DC voltage reduction solution.
- Key algorithms directly export results from MATLAB through symbolic simplification, which requires a significant amount of computation and almost saturates the chip's performance, making it impossible to further increase the control frequency. Consider manually simplifying and converting the code to C.

