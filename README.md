# Design and Implementation of a Pick and Place Controller for the Kinova Gen 3 Arm

## Created by University of Calgary ENME 501/502 Group 23

This GitHub Repository contains a collection of all the work completed through the course of this project. This work is presented in a simple, in-depth, concise, and educational way through a set of Jupyter Notebook Tutorials. These tutorials will help allow robotic enthusiasts, students, and researchers interested in the robotics of object manipulation to gain an understanding of the robotics and mathematics behind it all. 

We have broken down these tutorials into three sections. The tables below contain subsections for each of these parts along with direct links to the Jupyter Notebook Tutorials.

Part 1: Forwards Kinematics (Completed)

Part 2: Inverse Kinematics (Completed)

Part 3: Design and Implimantation of the Pick and Place Controller (Completed)
<br>
<br>
|Part 1 Forward Kinematics | Google Colab Link|
| ---                       | :---:                 |
|1. Position Kinematics                        | [Open in Google Colab](https://colab.research.google.com/drive/1g35uhfY7_xWpHOWx1P-7ra2q78EAMfEj?usp=sharing)                   |
|2. Manipulator Jacobian                        | [Open in Google Colab](https://colab.research.google.com/drive/1UusMWc5AYaRV8-5Zdo2Z9An0ylFIpavR?usp=sharing)                       |


<br>

|Part 2: Inverse Kinematics  | Google Colab Link|
| ---                        | :---:                 | 
|3. Resolved Rate Motion Control                     |[Open in Google Colab](https://colab.research.google.com/drive/1stDfdA4EYBy2L_KH_8mWTpmLZx1ZPEuX#scrollTo=BTOBDNN83A3n)                    |
|4. Numerical Inverse Kinematics                     |[Open in Google Colab](https://colab.research.google.com/drive/16Fr9mwFxgioye5OSf3-heZUe-vu89Pm2)                |

<br>

|Part 3: Design and Implimentation of the Pick and Place Controller| Google Colab Link|
| ---                                                              | :---:                 |
|5. Trust Constraint Method                   | [Open in Google Colab](https://colab.research.google.com/drive/1pJDImFs-ytFj88h5XOlPONxh1ScuNfLG#scrollTo=_bv8ODYZ4U52)                       |
|6. Computer Vison Code                    | [Open in Google Colab](https://colab.research.google.com/drive/105GEpzdTqPD_siRIzO0QI_cWIa34ZFCE)                      |
|7.Kinova Gen 3 Robotic Arm Pick and Place Controller                                                             | [Open in Google Colab](https://colab.research.google.com/drive/18irPSeUh2W0wV8vZxzd21Yjus2K8gEmY)                      |


## Synopsis

This project contains extensive information regarding mathematical calculations, code, and general information regarding the Kinova Gen 3 Robotic Arm. Due to this, it is easiest to digest this information in parts. Both Part 1 and Part 2 contain mathematical calculations for the forward kinematics and inverse kinematics respectively. These parts must be well understood in order for an individual to comprehend Part 3, which looks at using both previous parts in conjunction with computer vision to design the pick and place program for the Kinova Gen 3 Robotic Arm. The subsections seen below give some brief insight to each of these parts. 

### Part 1: Forward Kinematics
The first part of this project was focused on the forward kinematics. Forward kinematics is the process in which known joint angles are input into a mathematical function which outputs the final position of the arm in the cartesian coordinate system. The forward kinematics section will cover the following topics: 
1. Position Kinematics
2. Manipulator Jacobian

### Part 2: Inverse Kinematics
The second part of this project looked at inverse kinematics. Inverse kinematics is the opposite of forward kinematics; it is when we provide a desired location in the cartesian coordinate system and, through the use of mathematical functions, are able to get the corresponding set of joint angles relating to that specific position. The inverse kinematics section will cover the following topics: 
3. Resolved Rate Motion Control
4. Numerical Inverse Kinematics


### Part 3: Design and Implementation of the Pick and Place Controller
The final part of this project consisted of implementing the forward and inverse kinematics to allow the Kinova Gen 3 Robotic Arm to maneuver in such a way that it would be capable of picking up an object, manipulating said object, and placing this object in the desired location. This section will cover the following topics:
5. Implementation of Forward Kinematics to the Kinova Gen 3 Robotic Arm
6. Trust Constraint Method
7. Computer Vision Code
8. Implementation of Inverse Kinematics to the Kinova Gen 3 Robotic Arm


## FAQ

### What is Python?
Python is one of the most commonly used and easy to understand programming languages. Python is the primary language used throughout this entire tutorial and users will be required to download it in order to get complete access to the contents of this repository through the mean of Visual Studio and Visual Studio Code (if users choose to complete the tutorial through the Google Colab links provided, no installation is required).

 A simple step by step guide is provided [here](Installation_Tutorials/Install_Python/Install_Python.md).


### What is Kortex?
Kortex is an API for Kinova. Essentially, it is a software interface designed to allow developers to easily interact with Kinova robotic arms and related devices. It provides a set of programming tools and functions that enable users to control and communicate with Kinova robots, sensors, and peripherals. This API will allow us to work and utilize many functionalities of the Kinova robotic arm such as basic movement control which will help with positioning the arm in a 3D space. This tool will be used in section 3 of this tutorial. 

### Important Terms and Definitions
We will be mentioning several important terms throughout the tutorial so it is best to understand what these terms mean prior to starting the tutorial.
1. Base Frame: The base frame refers to the coordinate system of the part of the robotic arm that is attached to the ground/table. It is the end of the arm unable to move.
2. End-Effector: The end effector is the end of the arm that usually contains a gripper or some sort of hand-like mechanism to lift objects.
3. Translation: A translation refers to an offset distance. There is an offset because all of the joints are not in line. We will often use the word translation and offset interchangeably.
4. Actuator: This is a piece of equipment that converts energy to movement/force. We will often use the word joint and actuator interchangeably.
6. Jupyter Notebook: A Jupyter Notebook is a type of file where you can combine code and text. Think of it as a Google Document where you can also have code.
7. GitHub: This is a platform where individuals can make a repository which can contain various files. Think of this as a Google Drive.
8. Google Colab: This is a Jupyter Notebook file which can be accessed through a Google Drive.

### Accessing the Kinova Arm on Campus
#### (For University of Calgary Students)
For students wishing to access the avaiable Kinova gen 3 arm on campus, located in the Maker Multiplex (M2) ICT 215, please visit the following website to get started:  [https://schulich.libguides.com/m2z](https://schulich.libguides.com/m2z)

The first step is to complete the general access training, which will give university students access to the M2 D2L BRIGHTSPACE shell on their university accounts. After completing the training and getting access, students will then complete the required quizzes and safety on-boarding in the advanced skills and training modules that will expand their access.

## References

1.
We reference this tutorial in part one of our project for educational purposes:
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
Likwise for part two:
```
@article{haviland2023dkt2,
  author={Haviland, Jesse and Corke, Peter},
  title={Manipulator Differential Kinematics: Part II: Acceleration and Advanced Applications},
  journal={IEEE Robotics \& Automation Magazine}, 
  year={2023},
  pages={2-12},
  doi={10.1109/MRA.2023.3270221}
}
```
3.
```py
Documentation — Kortex API documentation. (n.d.). Docs.kinovarobotics.com. Retrieved March 1, 2024, from https://docs.kinovarobotics.com/index.html
```
‌
