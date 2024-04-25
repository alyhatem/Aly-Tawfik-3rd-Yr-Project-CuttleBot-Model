# Modelling a Soft Aquatic Robot using MuJoCo
## Introduction
The project focuses on modelling an aquatic robot using MuJoCo. The robot in question is The CuttleBot.\
A picture of the model is shown below\
<img width="154" alt="image" src="https://github.com/alyhatem/Aly-Tawfik-3rd-Yr-Project-CuttleBot-Model/assets/153009749/0b5bf30a-5d2f-4e14-938e-794531227b37">

## Usage
The '.xml' files use MuJoCo's MJCF. This builds the model using geometries and bodies.\
The '.py' files are python files that utilise MuJoCo's Python API to control the robot model.\
XML files can be loaded onto the MuJoCo app (see MuJoCo documentation on how to download this) by dragging and dropping.\
Python files can be run using any IDE that can run Python. Any libraries used in the code must be downloaded first before running.\
Make sure to correctly copy and paste the path to the XML file being used in the Pyhton code.\
An example can be shown below:\
``` xml_path = '/Users/alyhatem/Desktop/Project 127/Mujoco Python/CuttleBot With Skin/8Tendons_coupling.xml' ```
