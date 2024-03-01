# A Tutorial on the Kinova Gen 3 Robotic Arm Pick and Place Controller Theory and Implementation
The University of Calgary 2023/2024 mechanical engineering capstone students have put together this tutorial (IN PROGRESS) to help users understand and implement a pick and place controller for the Kinova Gen 3 robotic arm.

## Design and Implementation of a Pick and Place Controller for the Kinova Gen 3 Arm

#### *Created by ENME 501/502 Capstone Group 23*

This GitHub Reposotory contains a collection of all the work completed through the course of this project. This work is presented in a simple, in-depth, concise, and educational way through a set of Jupyter Notebook Tutorials. These tutorials will help allow robotic enthusiasts, students, and researchers interested in the robotics of object manipulation to gain an understanding of the robotics and mathmatics behind it all. 

We have broken down these tutorials into three sections. The tables below contain subsections for each of these parts along with direct links to the Jupyter Notebook Tutorials.

Part 1: Forwards Kinematics (**In Progress**)

Part 2: Inverse Kinematics (**Under Development**)

Part 3: Design and Implimantation of the Pick and Place Controller (**Under Development**)
<br>
<br>
|Part 1 Forward Kinematics  | Google Colab Link|
| ---                       | :---:                 |
|1. Forward and First Order Kinematics                        | [Open in Google Colab](https://colab.research.google.com/drive/1g35uhfY7_xWpHOWx1P-7ra2q78EAMfEj?usp=sharing)                   |
|2. Manipulator Jacoian                        | N/A                       |


<br>

|Part 2: Inverse Kinematics  | Google Colab Link|
| ---                        | :---:                 | 
|1. Numerical Inverse Kinematics                         | N/A                      |
|2. Resolved Rate Motion Control                         | N/A                       |
|3. Position Based Servoing                         | N/A                       |

<br>

|Part 3: Design and Implimentation of the Pick and Place Controller| Google Colab Link|
| ---                                                              | :---:                 |
|1. Implimentation of Forward Kinematics to Kinova Gen 3 Robotic Arm                                                               | N/A                       |
|2. Implimentation of Inverse Kinematics to Kinova Gen 3 Robotic Arm                                                               | N/A                       |
|3. Computer Vison Code                                                               | N/A                       |


## Synopsis

This project contains extensive information regarding mathematical calculations, code, and general information regarding the Kinova Gen 3 Robotic Arm. Due to this, it is easiest to digest this information in parts. Both Part 1 and Part 2 contain mathematic calculations for the forward kinematics and inverse kinematics respectivly. These parts must be well understood in order for an individual to comprhend Part 3, which looks at using both previous parts in conjunction with computer vison to design the pick and place program for the Kinova Gen 3 Robotic Arm. The subsections seen below give some brief insight to each of these parts. 

### Part 1: Forward Kinematics
The first part of this project was focused on the forward kinematics. Forward kinematics is the process in which known joint angles are input into a mathenmatical function which outputs the final position of the arm in the cartesian coordinate system. The forward kinematics section will cover the following topics: 
1. [Forward and First Order Kinematics](https://github.com/LoiGeral/-UNOFFICIAL-ENME501-Kinova-Gen-3-Arm-Manipulation/blob/main/Part_1/1_Forward_and_First_Order_Kinematics.ipynb)
2. [Manipulator Jacobian](https://github.com/LoiGeral/-UNOFFICIAL-ENME501-Kinova-Gen-3-Arm-Manipulation/blob/main/Part_1/2_Manipulator_Jacobian.ipynb)

### Part 2: Inverse Kinematics
The second part of this project looked at inverse kinematics. Inverse kinematics is the opposite of forward kinematics; it is when we provide a desired location in the cartesian coordinate system and, through the use of mathematical functions, are able to get the corresponding set of joint angles relating to that specifc position. The inverse kinematics section will cover the following topics: 
1. [Numerical Inverse Kinematics](https://github.com/LoiGeral/-UNOFFICIAL-ENME501-Kinova-Gen-3-Arm-Manipulation/tree/main/Part_2/Numerical_Inverse_Kinematics)
2. [Resolved Rate Motion Control](https://github.com/LoiGeral/-UNOFFICIAL-ENME501-Kinova-Gen-3-Arm-Manipulation/blob/main/Part_2/Resolved_Rate_Motion_Control.ipynb)
3. [Position Based Servoing](https://github.com/LoiGeral/-UNOFFICIAL-ENME501-Kinova-Gen-3-Arm-Manipulation/blob/main/Part_2/Position_Based_Servoing.ipynb)


### Part 3: Design and Implementation of the Pick and Place Controller
The final part of this project consisted of implementing the forward and inverse kinimatics to allow the Kinova Gen 3 Robotic Arm to maneuver in such a way that it would be capable of picking up an object, manipulating said object, and placing this object in the desired location. This section will cover the following topics:
1. Implimentation of Forward Kinematics to Kinova Gen 3 Robotic Arm
2. Implimentation of Inverse Kinematics to Kinova Gen 3 Robotic Arm
3. Computer Vison Code



## FAQ

### What is Python?
Python is a one of the most commonly used and easy to understand programming languages. Python is the primary language used throughout this entire tutorial and users will be required to download it in order to get complete access to the contents of this repository through the mean of Visual Studio and Visual Studio Code (if users choose to complete the tutorial through the Google Colab links provided, no installation is required).

 A simple step by step guide is provided [here](Installation_Tutorials/Install_Python/Install_Python.md).

### What is Kortex?
Kortex is an API for Kinova. Essentially, it is a software interface designed to allow developers to easily interact with Kinova robotic arms and related devices. It provides a set of programming tools and functions that enable users to control and communicate with Kinova robots, sensors, and peripherals. This API will allow us to work and utilize many functinoalties of the Kinova robotic arm such as basic movement control which will help with positioning the arm in a 3D space. This tool will be used in section 3 of this tutorial. 

## References

1.
``` 
@article{haviland2023dkt1,
  author={Haviland, Jesse and Corke, Peter},
  title={Manipulator Differential Kinematics: Part I: Kinematics, Velocity, and Applications},
  journal={IEEE Robotics \& Automation Magazine}, 
  year={2023},
  pages={2-12},
  doi={10.1109/MRA.2023.3270228}
}
```

2.
```py
Documentation — Kortex API documentation. (n.d.). Docs.kinovarobotics.com. Retrieved March 1, 2024, from https://docs.kinovarobotics.com/index.html
```
‌