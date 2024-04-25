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
## Files Explanation
"8tendons_coupling.xml" is the full model. If intended motion isnt achieved, change timestep to 0.004 inastaed of 0.002\
"tailTendon.xml" is one finray, you can uncomment the geoms with different masses to add a load to the finray.\
"tailTendon_Membrane.xml" is the finray with membrane attached.\
\
"motion_tests.py" is for testing different locomotion modes. ignore all code and only change "def controller": you can uncomment one of the function calls to use a different control algorithm, and you can change frequency and other parameters.
"sweep_freq_wavelength.py" is used in the Linear motion section to extract heatmap data. Will take ages to run so ignore.\
"step_response.py" used to test step response of singular finray. Dont forget to change masses as mentioned before by uncommenting.\
"frequency_response.py" used to test frequency response of singular fin for bode plot. Also change masses.\
\
