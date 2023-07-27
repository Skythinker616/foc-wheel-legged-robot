# SolidWorks Mechanical Model

**[简体中文](README.md) | English**

![Exploded View](readme-img/explode.jpg)

The entire mechanical model is designed using SolidWorks 2019 SP0.0. The current directory contains all parts and the assembly `总装.SLDASM`. Most parts are designed for 3D printing, and the baseplate is made from custom acrylic sheet.

---

## Component Selection

> Note: The purchase links below were used by the author at the time and are provided for reference only. Availability is not guaranteed.

### Motors

![4010](readme-img/4010.png) ![2804](readme-img/2804.png)

The articulated joint uses the 4010 motor shown on the left, and the wheel motor uses the 2804 motor shown on the right. The measured motor parameters are as follows:

| Motor Model | Rated Voltage | Stall Torque | Pole Pairs | Weight | Purchase Link | Reference Price |
| :---------: | :-----------: | :----------: | :-------: | :----: | :-----------: | :------------: |
|    4010     |      12V      |   0.22N·m   |    11     |  125g  | [Link](https://item.taobao.com/item.htm?spm=a1z09.2.0.0.67002e8djsrWfk&id=661262833408) |    ￥50.00    |
|    2804     |      12V      |   0.04N·m   |    7      |  24g   | [Link](https://item.taobao.com/item.htm?spm=a21n57.1.0.0.54cb523cFK7YX5&id=688692183367) |    ￥13.00    |

### Battery

![Battery](readme-img/battery.png)

The robot uses a high-energy-density 25C Li-Po batterie. The measured parameters are as follows:

| Voltage | Capacity | Discharge Rate | Weight | Robot Endurance | Purchase Link | Reference Price |
| :-----: | :------: | :------------: | :----: | :-------------: | :-----------: | :------------: |
| 3S (11.1-12.6V) | 800mAh |      25C       |   57g  |    20-30min     | [Link](https://item.taobao.com/item.htm?spm=a1z09.2.0.0.67002e8djsrWfk&id=679306961701) |    ￥28.00    |

### Bearings

A deep groove ball bearing and a thrust bearing are used at each joint, with the following specifications:

|     Item     | Inner Diameter | Outer Diameter | Thickness |     Purchase Link     | Reference Price |
| :----------: | :------------: | :------------: | :-------: | :-------------------: | :------------: |
| Deep Groove Ball Bearing 604 |      4mm       |      12mm      |    4mm    | [Link](https://item.taobao.com/item.htm?spm=a1z09.2.0.0.67002e8djsrWfk&id=626299304028) |    ￥1.30    |
|    Thrust Bearing F8-14M     |      8mm       |      14mm      |    4mm    | [Link](https://item.taobao.com/item.htm?spm=a1z09.2.0.0.67002e8djsrWfk&id=643067962342) |    ￥2.00    |

### Screws

Most screws used on the robot are [flat-head screws](https://detail.tmall.com/item.htm?_u=f3m84i7t8421&id=677398679720) with various specifications including M2, M2.5, M3, and M4, with lengths ranging from 6mm to 16mm. Some screws are not depicted in the model and need to be measured independently (the threaded blind holes on the motors are 2-3mm deep).

### Tires

![Tire](readme-img/tire.png)

The robot uses self-adhesive sponge strips as tires, providing good cushioning and strong grip but with relatively poor wear resistance, requiring regular replacement.

### Encoder Magnets

The magnets on the motor rotors must be radially magnetized. Since the 2804 motor does not come with magnets, they need to be designed separately. For this model, magnets with dimensions of 6mm in diameter and 2mm in thickness are used. [Purchase Link](https://item.taobao.com/item.htm?spm=a1z09.2.0.0.67002e8dunEQwK&id=596454786426) Reference Price: ￥6.00 / 5 pieces

### Structural Components

The robot's baseplate is made from custom white acrylic sheet. [Purchase Link](https://detail.tmall.com/item.htm?_u=f3m84i7t207c&id=667937893428) Reference Price: ￥5.00

All other structural components are 3D printed using white resin material from Jialichuang, with a total cost of approximately ￥100.00.

---

## Improvement Directions

- The torque of the articulated joint motor is insufficient for completing jumping actions, and the wheel motor's torque is insufficient for overcoming high obstacles. Consider replacing them with motors having higher torque.
- The current battery fixation is not sturdy enough. Consider adding additional fixation structures.
- The installation method of the wheel motor is not very convenient. Consider making modifications.

