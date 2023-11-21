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

> Note: Import the two JSON files, the schematic and PCB, into [LCEDA Standard Edition](https://lceda.cn/editor) to view and edit them. It is not recommended to use AD.

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

In addition, ESP32C3's USB also supports JTAG debugging. After connecting it to the computer, you can use programs like [OpenOCD](https://openocd.org/) to connect. During debugging, I combined it with [Linkscope](https://gitee.com/skythinker/link-scope) for online variable reading, writing, and curve drawing, which is quite convenient.

### Parameter Calibration

Several parameters in `main.c` may need to be adjusted according to the actual system; otherwise, correct computation and control may not be achieved.

#### Zero Offset and Rotation Direction Parameters

In the `Motor_InitAll` function, the `Motor_Init` function is called to set parameters for each motor, including the zero offset `offsetAngle` and rotation direction `dir`.

These two parameters are used in the `Motor_Update` function to convert the raw angle feedback from the encoder to the standard angle $\phi_i$ in the algorithm (motor `angle` member variable). The conversion formula is:

$$
\phi_i = (rawAngle_i - offsetAngle_i) \times dir_i
$$

The measurement methods for these two parameters are as follows:

1. Set the `dir` parameter (values 1/-1) so that the motor `angle` member variable increases when the motor rotates in the positive direction of $\phi_i$.

2. Set `offsetAngle=0` in the program (denoted as $offsetAngle_i^z$), rotate the motor to a certain angle, record the current calculated angle as $\phi_i^c$, and measure the current actual angle as $\phi_i^r$.

3. Substitute $\phi_i^c$, $offsetAngle_i^z$, and $dir_i$ into the above formula to obtain the value of $rawAngle_i$.

4. Substitute $\phi_i^r$, $rawAngle_i$, and $dir_i$ into the above formula to obtain the value of $offsetAngle_i$ and set it in the program.

5. At this point, the `angle` member variable should be consistent with the actual $\phi_i$.

> Note 1: The definition of $\phi_i$ can be found in the article referenced in the Matlab program documentation, where the front joint motor corresponds to $\phi_4$, and the rear joint motor corresponds to $\phi_1`.
> 
> Note 2: The descriptions above are for joint motors; wheel motors only need to set the `dir` parameter to align their `angle` variable direction with the driving wheel torque $T$ in the algorithm.

#### Torque Coefficient

> Note: If using the same motors as the author, this parameter can be left unchanged.

Similar to the above two parameters, this parameter is also set in the `Motor_InitAll` function and is used for the conversion between voltage and torque. The formula is:

$$
voltage_i = \frac{torque_i}{torqueRatio_i}
$$

Where `torqueRatio` is the torque coefficient, and its measurement method is as follows:

1. Apply different voltages to the motor and measure its torque (the author measured the stall torque), recorded as $(v_i, \tau_i)$.
2. Fit $(v_i, \tau_i)$ to a linear function $\tau=k \times v$. The value of $k$ is the `torqueRatio` and can be calculated using software like Matlab, Excel, etc.

#### Back EMF Function

> Note: If using the same motors as the author, this parameter can be left unchanged.

The back electromotive force calculation function for joint motors is `Motor_CalcRevVolt4010`, and for wheel motors, it is `Motor_CalcRevVolt2804`. It is used to calculate the back electromotive force based on the motor's real-time speed. The output voltage command will be compensated according to the calculation result to counteract the back electromotive force.

When measuring the back electromotive force, apply different voltages to the motor, measure its no-load speed, and record it as $(v_i, \omega_i)$. Fit it to a third-degree polynomial function $v = f(\omega)$, which is the back electromotive force function.

If the back electromotive force calibration is correct, when setting the motor torque to 0 (`torque` member is 0), manually rotating the motor should result in almost zero resistance (when compensating for back electromotive force, most of the mechanical resistance is also compensated).

> Note: Comment out `Ctrl_Init();` in `setup()` or comment out all calls to `Motor_SetTorque` functions to disable the control system's output. This allows for manual insertion of code to debug the motor.

---

## Future Improvements

- The DMP frequency of MPU6050 is only 200Hz; consider replacing it with other chips and writing attitude calculation programs.
- The hardware uses an LDO for voltage reduction, but due to the high power consumption of ESP32C3, the LDO generates a lot of heat. Consider changing to a DC-DC voltage reduction solution.
- Key algorithms directly export results from MATLAB through symbolic simplification, which requires a significant amount of computation and almost saturates the chip's performance, making it impossible to further increase the control frequency. Consider manually simplifying and converting the code to C.

