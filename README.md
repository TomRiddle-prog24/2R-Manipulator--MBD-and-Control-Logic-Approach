# 2R-Manipulator--MBD-and-Control-Logic-Approach
# Introduction- Dynamics
A 2R Manipulator is a **Planar Robotic Manipulator** containing *2 DOF*. The modelling is purely done using Python Symbolics and Numerical computations. This project showcases various elements in the design of a 2R manipulator, from its Kinematics, Dynamics and Systems Design to the control logic and manipulation. Additionally, a **Multi-body-dynamics** approach has been chosen **over the traditional approaches** (Such as D-H (Denavit-Hartenberg Mapping) and Parameters)). A *Computirized* derivation has been encouraged and appreciated in this project (Formerly, most projects focus on **Hand Derivations for the entire mechanism** earlier and treating the program built in MATLAB or Python or equivalent programming languages just as a calculator/solver. One of the major goals of the project is to enhance the Symbolic Derivations performed purely by the MBD programs in this project). 

# Required Modules 
Before running the simulation, ensure that the following modules have been set up. 
```python
import sympy as sm     
import sympy.physics.mechanics as me                   #For Symbolic computations
import numpy as np 
import matplotlib.pyplot as plt
me.init_vprinting(use_latex='mathjax') 
import scipy.integrate as sp
import scipy.optimize as so
from matplotlib.animation import FuncAnimation          #For Visualization of the mechanism
from IPython.display import HTML
import matplotlib as ilt
ilt.rcParams['animation.embed_limit'] = 50.0          #For the animation to be fit the entire time steps for a memory of 50MB 
```
# C1: Forward Kinematics (FK) 
(*In the first commit, the forward Kinematics (FK) of the robot has been built*)
Forward Kinematics: In any Mechanical Mechanism, we refer to FK as an **analysis** that **determines** the position of the **End-Effector (EF)** for a given set of Joint angles. Since the robot's DOF=2, its obvious that the Joint angles (Defined $q_1, q_2$) is *independent* of each other. The definitions of the link lengths ($l_1, l_2$) has been attached here. 

<div align="center">
    <img src="https://github.com/user-attachments/assets/3bd69b3b-fc0f-4b2e-bcb4-1913ff1dcd24"  alt="Two-link robotic manipulator mechanism" width="550">
  <br>
  <blockquote><b>Fig. 1</b> — <i>Description of the problem modeled for an FK Analysis.</i></blockquote>
</div>

As of now, currently the FK MBD Solver, solves for the following conditions:
$$
\begin{aligned}
q_1 &\in [0, 2\pi] \\
q_2 &= \frac{\pi}{3} \quad \text{fixed angle} \\
^{N}\omega_A &= \dot{q}_1\hat{n}_z = \frac{2\pi}{20} \quad \text{in rad/s} \\
^{N}\omega_B &= \dot{q}_2\hat{n}_z = 0 \\
l_1 &= 1\text{m} \\
l_2 &= 1.2\text{m}
\end{aligned}
$$
The results of the FK have been presented as well for the first commit.

# Pending Work
- Inverse Kinematics (IK)
- Dynamics of the mechanism
- Integration with the hardware 


