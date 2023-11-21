# STM32-FOC Motor Driver Board

**[简体中文](README.md) | English**

This is a submodule of the robot project, which is a brushless motor driver board based on the STM32 chip. It currently has the following features:

- Chip solution
  - Main control chip: STM32F103C6T6
  - Driver chip: DRV8313
  - Magnetic encoder: AS5600
  - CAN driver chip: TJA1050T
- Only supports voltage-controlled torque; hardware does not support current closed-loop control
- Uses CAN communication for control (UART is available on the hardware but not utilized in the program)
- Can provide feedback on position and speed with a feedback frequency of 500Hz
- Supports setting the motor ID through a button or triggering automatic calibration, and the generated parameters are saved in non-volatile memory
- PCB size: diameter of 30mm
- Power supply voltage used in this project: 12V
- Material cost: approximately ￥25

> Note: STM32F103C8T6 can be directly replaced and is fully compatible in both software and hardware, as tested.

This submodule has been open-sourced on the oshwhub platform: [https://oshwhub.com/skythinker/simplefoc103](https://oshwhub.com/skythinker/simplefoc103)

---

## Hardware Description

![Circuit Design](readme-img/design.png)

The circuit is a double-layer board design created using [LCEDA](https://lceda.cn/). The opposite side drilling is spaced at 22mm, which is the same as the 2804 motor mounting holes.

> Note: The program uses the internal RC oscillator of the microcontroller, so the external crystal X1 and load capacitors C1/C2 can be left unpopulated.

### Files Description

The schematic and PCB files are located in the `hardware` folder:

- `STM32FOC_LCEDA_SCH.json`: LCEDA schematic file
- `STM32FOC_LCEDA_PCB.json`: LCEDA PCB file
- `STM32FOC_AD_SCH.schdoc`: Altium Designer schematic file exported from LCEDA
- `STM32FOC_AD_PCB.pcbdoc`: Altium Designer PCB file exported from LCEDA
- `STM32FOC_SVG.svg`: Schematic svg file

> Note: Import the two JSON files, the schematic and PCB, into [LCEDA Standard Edition](https://lceda.cn/editor) to view and edit them. It is not recommended to use AD.

---

## Software Description

The program is developed using Keil uVision 5 as the development environment and generated using STM32CubeMX v5.6.0. It is written using the HAL library.

The FOC core algorithm is based on the C-language version of SimpleFOC, originally ported by CSDN blogger [loop222](https://blog.csdn.net/loop222). This program further integrates it into the HAL library and adds features such as CAN communication, flash memory for non-volatile storage, buzzer, buttons, LEDs, and filtering.

### Files Description

The `software` folder contains the STM32 code project, which includes the following files:

- `MDK-ARM/C6T6SimpleFoc.uvprojx`: Keil uVision 5 project file
- `C6T6SimpleFoc.ioc`: STM32CubeMX project file
- `Src/main.c`: Main program code
- `Src/stm32f1xx_it.c`: Interrupt service routine code
- `USER` folder contains other utility code:
  - `BLDCMotor.c/h`, `FOCMotor.c/h`, `foc_utils.c`: FOC algorithm related code
  - `MagneticSensor.c/h`: AS5600 magnetic encoder driver code
  - `FlashStorage.c/h`: Flash non-volatile memory storage code

---

## Usage Instructions

### Hardware Setup

1. Fix the driver board and the motor together, align the radial magnet on the motor rotor with the magnetic encoder, and keep a distance of 2-3mm.

2. The driver board uses 2.54mm pitch pin headers to expose motor three-phase lines, CAN bus, power supply, SWD debugging interface, and UART. Users can solder them as needed.

> Note: The recommended power supply voltage is around 12V, and the specific range can be referred to the voltage range of the DRV8313 chip and the maximum withstand voltage of the step-down LDO chip.

### Driver ID Setting and Automatic Calibration

**Driver ID**: This is an identifier that distinguishes multiple motors on the same CAN bus, ranging from 1 to 8. The driver IDs on the same bus must not be duplicated.

**Automatic Calibration**: The driver needs to know the pole pairs, zero offset, and sensor direction information of the motor. These pieces of information can be obtained through automatic calibration.

- In normal running state, the LED will blink to indicate the current motor ID.
- To set the ID, press and hold the button. The LED will blink at a 500ms interval. Release the button when the LED blinks for the N-th time to set the ID as N.
- If the button is held for more than 8 blinks, the LED will remain lit for 2 seconds, indicating that automatic calibration will be triggered after releasing the button.
- If the button is not released after 2 seconds, the LED will turn off, and releasing the button will not trigger any action.
- During automatic calibration, the motor will rotate slowly. Ensure that the motor is in an unloaded state and reduce rotational resistance as much as possible. Upon successful calibration, a startup sound effect will be played.

### CAN Communication Protocol

CAN communication uses standard frames with a baud rate of 1Mbps.

**Driver Feedback Data Frame Format**:

Different driver boards with distinct IDs will use different CAN data frame identifiers (StdID) to send feedback data:

| Driver ID | StdID    | Frame Type | DLC |
| :-------: | :------: | :--------: | :-: |
| N         | 0x100+N  | Standard   | 8   |

| DATA[0-3]         | DATA[4-5]          | DATA[6-7] |
| :--------------: | :---------------: | :------: |
| Accumulated angle \* 1000 | Current speed \* 10 | Reserved |
| Unit: rad       | Unit: rad/s        | /        |
| int32_t          | int16_t           | /        |

**Driver Board Receiving Voltage Command Data Frame Format**:

To save bus bandwidth, driver boards with different IDs will not use entirely different data frames to receive data. Instead, they share the same data frame with different bytes.

The four driver IDs (1-4) share the receive data frame **StdID = 0x100**:

| DATA[0-1]          | DATA[2-3]          | DATA[4-5]          | DATA[6-7]          |
| :---------------: | :---------------: | :---------------: | :---------------: |
| ID=1 Output Voltage | ID=2 Output Voltage | ID=3 Output Voltage | ID=4 Output Voltage |
| Unit: mV         | Unit: mV         | Unit: mV         | Unit: mV         |
| int16_t           | int16_t           | int16_t           | int16_t           |

The four driver IDs (5-8) share the receive data frame **StdID = 0x200**:

| DATA[0-1]          | DATA[2-3]          | DATA[4-5]          | DATA[6-7]          |
| :---------------: | :---------------: | :---------------: | :---------------: |
| ID=5 Output Voltage | ID=6 Output Voltage | ID=7 Output Voltage | ID=8 Output Voltage |
| Unit: mV         | Unit: mV         | Unit: mV         | Unit: mV         |
| int16_t           | int16_t           | int16_t           | int16_t           |

In other words, if a data frame with StdID=0x100 is sent on the bus, driver boards with IDs 1-4 will simultaneously receive the command. If a data frame with StdID=0x200 is sent, driver boards with IDs 5-8 will receive the command.

> Note: The above mentioned output voltages are RMS values, not peak values. For example, when the supply voltage is 12V, the allowable range for the output voltage is [0, 6.93V] (12 / √3 = 6.93).

---

## Areas for Improvement

- The AS5600 magnetic encoder data is not stable, and the sampling frequency is not high enough (software I2C measured at a maximum of about 2KHz), leaving little room for filtering algorithms. Consider replacing it with a better encoder.

