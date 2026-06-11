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

The Equations in question are: 
$$
r_e(q_1, q_2)=\begin{bmatrix}
l_{1}\cos{\left(q_{1}\right)}+l_{2}\left(-\sin{\left(q_{1}\right)} \sin{\left(q_{2}\right)}+\cos{\left(q_{1}\right)}\cos{\left(q_{2}\right)}\right)\\
l_{1}\sin{\left(q_{1}\right)}+l_{2} \left(\sin{\left(q_{1}\right)}\cos{\left(q_{2}\right)}+\sin{\left(q_{2}\right)}\cos{\left(q_{1}\right)}\right)
\end{bmatrix}=\begin{bmatrix}
                                    x_e\\
                                    y_e
                      \end{bmatrix}
$$
These equations can be symbolically derived. And all the solver has to do is subsitute for the joint angles and obtain the various end effector positions. 
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

# C2: Inverse Kinematics (IK) (P2P)

Without the considerations of Trajectory Planning and Trajectory Tracking, we can formulate our Inverse Kinematics Equations **Just using positions**. In such an approach, **considerations for the Jacobians** _explicitly_ (reasons provided as follows) is **NOT** essential.  

## Idea of Inverse Kinematics (Without Explicitly using Jacobians)-P2P IK

The modelling proposed here sometimes is also called a P2P IK. 
In Forward Kinemtics, our goal is to **obtain the position of the end effector in space** as the angle of the joints, change w.r.t space. In Inverse Kinematics, our idea is to **find the angle of joints ($q_1, q_2$)** for a **given end-effector position**. 
Hence we have to solve $f_h$. Which is: 
$$
f_h(q_1, q_2)=0
$$ 
Interestingly, we are **bypassing** the calclations of Jacobians **Explicitly** (as we dont define ANY Jacobians anywhere) using Numerical Iterative Techniques here primarily because we obtain the respective position of the EF. However ```fsolve()``` uses a default method which is **hybrj** which essentially uses **Jacobians** to let the values converge faster. And given that $f_e$ is a *Transcendental Equation* its roots have to be solved using Numerical Root Finding Techniques.  

## Considerations and stability

Now, if we use the same technique for some point say $(x_e, y_e)=(3,-1)$, the solver would inherently **fail**. Because the **max end effector position** when both link-1, link-2 are aligned (i.e when $q_2=0$) is: 
$$
l_1+l_2=\text{max end effector position}=1+1.2=2.2m
$$
And so, physically it'd be illogical for the links to reach anywhere beyond it. So the limits of $x_e, y_e$ can be determined as:
$$
\sqrt{x_e^2+y_e^2}=2.2=\phi(x,y)
$$
So, its obvious that for the End-Effector to perform some action, it should be well within the locii as described by the function $\phi$. Long story short, the values of $x_e$, $y_e$ have to be such that: 
$$
x_e^2+y_e^2 \leq (l_1+l_2)^2
$$
The results from the P2P IK is shown below: 

# C3: Inverse Kinematics (IK) (With Trajectory Planning)

Say a robot is confined to a specific policy, say to move along a Straight line or move along a specific curved path or say a Velocity constraint is been **effected** with the **End Effector**, in all these cases, various constraints would have to be effected in the Kinematics Equations. Mind that, Kinematics comprises of both **Position Analysis** and **Velocity Analysis** (and further analysis of derived Kinematics Quantities such as Jerks, Accelerations). 
## Trajectory Planning/Tracking 
Here, for our model, we have used a set of points which would form the locii of a line which can be mathematically be defined as: 
$$
y=\kappa=5m 
$$
Trajectory doesnt have to be required as a line alone. It can be any complex curve (Say a Bezier Curve or a Hermite Curve) etc...
## Degrees of Freedom 

Now, in a normal 2R manipulator, DOF=2. Its obvious. However, in this case, when we constrain our 2R Manipulator's end Effector to move along a specific direction (here along -ve x-axis), we introduce **1 Velocity Constraint**. Which is: 
$$
\vec{v}_e^N.\hat{n}_y=0
$$  
Hence the DOF of the mechanism **Reduces to 1**. Hence for our control/generalized speeds to be solved for, we introduce $\dot{q_1}$ as our independent variable. This also means, the **end effector's velocity magnitude CANNOT** be of our choice. Only the motion path is under our control.  
An alternate way of addressing is, I can set my $\vec{v}_e^N$. But that would mean, $\dot{q}_1$ and $\dot{q}_2$ now are **DEPENDANT** on $\vec{v}_e^N$. So, here we choose the 2nd choice of fixing, Ve. Hence we formulate this for our robotic system. The equations and Numerical Procedure have been well explained in the notebook. 

## Results of the Trajectory planning: 

>[NOTE]
>For this analysis, we have changed the link lengths, so as to not involve too much of Singularities and other issues due to Singularities. 

# Pending Work
- Motion Planning
- Dynamics of the mechanism
- Integration with the hardware 


