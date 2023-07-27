<div align=center>
	<img src="readme-img/cover.jpg"/>
	<h1>FOC Wheel Legged Balance Robot Project</h1>
	<p>
		<a href="https://gitee.com/skythinker/foc-wheel-legged-robot">
			<img src="https://gitee.com/skythinker/foc-wheel-legged-robot/badge/star.svg"/>
		</a>
		<img src="https://gitee.com/skythinker/foc-wheel-legged-robot/badge/fork.svg"/>
		<a href="https://github.com/Skythinker616/foc-wheel-legged-robot">
			<img src="https://img.shields.io/github/stars/skythinker616/foc-wheel-legged-robot?logo=github"/>
		</a>
		<img src="https://img.shields.io/github/forks/skythinker616/foc-wheel-legged-robot?logo=github"/>
		<a href="https://www.bilibili.com/video/BV1bP411k75b">
			<img src="https://img.shields.io/badge/dynamic/json?label=views&style=flat&logo=bilibili&query=data.stat.view&url=https%3A%2F%2Fapi.bilibili.com%2Fx%2Fweb-interface%2Fview%3Fbvid%3DBV1bP411k75b"/>
		</a>
		<img src="https://img.shields.io/badge/License-GPL3.0-red"/>
	</p>
	<p>
		<b><a href="README.md">简体中文</a> | English</b>
	</p>
</div>

This is a complete robot project that includes algorithm simulation, mechanical structure design, electronic hardware design, embedded software design, and upper computer software design. It has accomplished the following:

- Mechanical structure design using SolidWorks
- Algorithm design and robot physics simulation based on MATLAB / Simulink / Simscape
- Brushless motor driver board based on STM32 using CAN communication
- Motion control module (main control module) based on ESP32 and MPU6050
- Linux image transmission module based on ffmpeg / ffserver, using a low-coupling plug-and-play solution
- Android remote control APP with Bluetooth pairing support

**Demo & Introduction Video:** [https://www.bilibili.com/video/BV1bP411k75b/](https://www.bilibili.com/video/BV1bP411k75b/)

---

## Showcase

**Mechanical Structure Renderings:**

![Renderings](readme-img/mechanical.png)

**Simscape Multibody Simulation:**

![Simulation](readme-img/simulation.png)

**Robot Acceleration:**

![Acceleration](readme-img/accel.png)

**Fall Cushioning Performance:**

![Fall](readme-img/fall.png)

**Remote Control APP:**

![App](readme-img/app.png)

---

## Repo Structure

The entire robot project is divided into several parts, each located in different directories within the repository, with more detailed explanations inside. Readers can check them as needed:

- [`solidworks`](solidworks): Mechanical structure design, including all parts and assembly model files
- [`matlab`](matlab): Algorithm simulation, including model building, algorithm design, and simulation files
- [`stm32-foc`](stm32-foc): Brushless motor driver board, including hardware design files and STM32 code project
- [`esp32-controller`](esp32-controller): Motion control module, including hardware design files and ESP32 code project
- [`linux-fpv`](linux-fpv): Linux image transmission module, including related Shell scripts and Python scripts
- [`android`](android): Android remote control APP, including source code and precompiled installation package

> Note: The image transmission module is an optional module. Removing it will not affect other functions but will significantly reduce the cost and complexity of the project.

---

## Material Cost

| Item | Quantity | Unit Price | Total Price |
| :--: | :--: | :--: | :--: |
| 4010 Motor | 4 | ¥50.00 | ¥200.00 |
| 2804 Motor | 2 | ¥13.00 | ¥26.00 |
| Driver Board Components | 6 | ¥25.00 | ¥150.00 |
| Main Control Board Components | 1 | ¥20.00 | ¥20.00 |
| Li-Po Battery | 1 | ¥28.00 | ¥28.00 |
| 3D Printed Parts | - | Approx. ¥100.00 | Approx. ¥100.00 |
| Custom Acrylic | 1 | ¥5.00 | ¥5.00 |
| Bearings, Screws | - | Approx. ¥20.00 | Approx. ¥20.00 |
| Image Transmission Core Board (optional) | 1 | ¥150.00 | ¥150.00 |
| Camera (optional) | 1 | ¥20.00 | ¥20.00 |
| **Total Cost (without image transmission)** | - | - | **¥549.00** |
| **Total Cost (with image transmission)** | - | - | **¥719.00** |

> Note: The above prices are the actual purchase prices of the author and are for reference only. Please refer to the respective module's description for some purchase links.
