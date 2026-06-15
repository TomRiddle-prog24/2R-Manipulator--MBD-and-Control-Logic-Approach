# 2R-Manipulator--MBD-and-Control-Logic-Approach
# Introduction- Dynamics
A 2R Manipulator is a **Planar Robotic Manipulator** containing *2 DOF*. The modelling is purely done using Python Symbolics and Numerical computations. This project showcases various elements in the design of a 2R manipulator, from its Kinematics, Dynamics and Systems Design to the control logic and manipulation. Additionally, a **Multi-body-dynamics** approach has been chosen **over the traditional Robotics approaches** (Such as D-H (Denavit-Hartenberg Mapping) and Parameters)). A *Computirized* derivation has been encouraged and appreciated in this project (Formerly, most projects focus on **Hand Derivations for the entire mechanism** earlier and treating the program built in MATLAB or Python or equivalent programming languages just as a calculator/solver. One of the major goals of the project is to appreciate the Symbolic Derivations performed purely by the MBD programs in this project). 

## A humble note to the reader 
*This repository has been created as a part of showing my self learning journey and my deep interest in Robotics in the view of Dynamics and Control. Even though I ensure that all the modelling is consistent with the Traditional ones and existing data, there may be mistakes involved. If any mistake has been spotted, you can very well correct me (as I consider corrections as a way for me to learn something) by submitting a Pull Request* 

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
    <img width="581" height="442" alt="image" src="https://github.com/user-attachments/assets/f9fb96f5-d4e9-4373-aa08-795bbe7a665a" />
  <br>
  <blockquote><b>Fig. 1</b> — <i>Description of the problem modeled for an FK Analysis.</i></blockquote>
</div>

The Equations in question are:

$$ 
\begin{aligned}
r_e(q_1, q_2) &= \begin{bmatrix} l_{1}\cos(q_{1})+l_{2}\left(-\sin(q_{1})\sin(q_{2})+\cos(q_{1})\cos(q_{2})\right) \\ l_{1}\sin(q_{1})+l_{2}\left(\sin(q_{1})\cos(q_{2})+\sin(q_{2})\cos(q_{1})\right) \end{bmatrix} \\
&= \begin{bmatrix} x_e \\ y_e \end{bmatrix} 
\end{aligned}
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

## Idea of Inverse Kinematics

The modelling proposed here sometimes is also called a P2P IK. 
In Forward Kinemtics, our goal is to **obtain the position of the end effector in space** as the angle of the joints, change w.r.t space. In Inverse Kinematics, our idea is to **find the angle of joints ($q_1, q_2$)** for a **given end-effector position**. In this space, we're assessing the various Poses of the Manipulator for **Different** End effector Positions.
Hence we have to solve $f_h$. Which is:  

$$
\begin{aligned}
f_h(q_1, q_2) &= 0
\end{aligned}
$$

Interestingly, we are **bypassing** the calclations of Jacobians **Explicitly** (as we dont define ANY Jacobians anywhere) using Numerical Iterative Techniques here primarily because we obtain the respective position of the EF. However ```fsolve()``` uses a default method which is **hybrj** which essentially uses **Jacobians** to let the values converge faster. And given that $f_e$ is a *Transcendental Equation* its roots have to be solved using Numerical Root Finding Techniques.  

## Considerations and stability

Now, if we use the same technique for some point say $(x_e, y_e)=(3,-1)$, the solver would inherently **fail**. Because the **max end effector position** when both link-1, link-2 are aligned (i.e when $q_2=0$) is:  

$$
\begin{aligned}
l_1+l_2 &= \text{max end effector position} &= 1+1.2 &= 2.2m
\end{aligned}
$$

And so, physically it'd be illogical for the links to reach anywhere beyond it. So the limits of $x_e, y_e$ can be determined as:  

$$
\begin{aligned}
\sqrt{x_e^2+y_e^2} &= 2.2 &= \phi(x,y)
\end{aligned}
$$

So, its obvious that for the End-Effector to perform some action, it should be well within the locii as described by the function $\phi$. Long story short, the values of $x_e$, $y_e$ have to be such that:  

$$
\begin{aligned}
x_e^2+y_e^2 &\leq (l_1+l_2)^2
\end{aligned}
$$

The results from the P2P IK have also been documented. 

# C3: Inverse Kinematics (IK) (Differential)

In this section, I present my work and findings by **Explicitly Specifying** a Trajectory of the end effector. Here, the considerations of **Motion Planning** (involving obstacle mapping) and **Trajectory Generation** is NOT involved. This is because, Inverse Kinematics or Direct Kinematics or anything related to Mechanics, is **Independant** of Motion Planning if a trajectory already exists.  

Say a robot is confined to a specific policy, say to move along a Straight line or move along a specific curved path or say a Velocity constraint is been **effected** with the **End Effector**, in all these cases, various constraints would have to be effected in the Kinematics Equations. Mind that, Kinematics comprises of both **Position Analysis** and **Velocity Analysis** (and further analysis of derived Kinematics Quantities such as Jerks, Accelerations). Hence the **Path of the end effector** can be affected by other Kinematic Quantities as well. 
## Trajectory Planning/Tracking 
Here, for our model, we have used a set of points which would form the locii of a line which can be mathematically be defined as:

$$
\begin{aligned}
y&=\kappa&=5m 
\end{aligned}
$$

Trajectory doesnt have to be required as a line alone. It can be any complex curve (Say a Bezier Curve or a Hermite Curve) etc...
## Degrees of Freedom 

Now, in a normal 2R manipulator, DOF=2. Its obvious. However, in this case, when we constrain our 2R Manipulator's end Effector to move along a specific direction (here along -ve x-axis), we introduce **1 Velocity Constraint**. Which is:

$$
\begin{aligned}
\vec{v}_e^N.\hat{n}_y &= 0
\end{aligned}
$$

Hence the DOF of the mechanism **Reduces to 1**. Hence for our control/generalized speeds to be solved for, we introduce $\dot{q_1}$ as our independent variable. This also means, the **end effector's velocity magnitude CANNOT** be of our choice. Only the motion path is under our control.  
An alternate way of addressing is, I can set my $\vec{v}_e^N$. But that would mean, $\dot{q}_1$ and $\dot{q}_2$ now are **DEPENDANT** on $\vec{v}_e^N$. So, here we choose the 2nd choice of fixing, Ve. Hence we formulate this for our robotic system. The equations and Numerical Procedure have been well explained in the notebook. 

## Results of the Differential IK: 
<div align="center">
  <img width="862" height="833" alt="image" src="https://github.com/user-attachments/assets/ac45ccd3-3706-455d-91a4-52c205e825d6" alt="Two-link robotic manipulator mechanism" width="550">
  <br>
  <blockquote><b>Fig. 2</b> — <i>Results from the Trajectory Plannning.</i></blockquote>
</div>  

<div align="center">
  <img width="957" height="872" alt="image" src="https://github.com/user-attachments/assets/e745358f-7002-45ff-8be5-4ade60ecd738" />
  <br>
  <blockquote><b>Fig. 3</b> — <i>Visualization of the Trajectory Planning.</i></blockquote>
</div>

>[NOTE]
>For this analysis, we have changed the link lengths, so as to not involve too much of Singularities and other issues due to Singularities. 
# Trajectory Quality Analysis
For any given trajectory (and yes this is not just restricted to a 2R Manipulator but for any Mechanism) we generally expect the following: 
-- Jerk to be less (as much low as possible without much of sudden peaks in the plots or steep graphs) 
-- Smooth Acceleration (without discontinuities) profiles
-- Smooth Velocity Profiles (without discontinuities and sudden peaks, as its effects will be observed on the life of the Actuators involved). 

<div align="center">
  <img width="1256" height="837" alt="image" src="https://github.com/user-attachments/assets/cdc6427c-47cf-40e1-81fd-4ebd650ff270" />
  <br>
  <blockquote><b>Fig. 4</b> — <i>Trajectory Quality Analysis.</i></blockquote>
</div>
Here, we map the velocities and accelerations with the angular velocities and accelerations of the links. Its fascinating and interesting to observe the fact that even though there is a sudden jerk which is observed in the plots for the link-1, its effects seem to be minimized with the end effector link. The acceleration plots can directly imply the jerks (as per the definition of a jerk) and its continuous for both the links. The trajectory's quality can be attributed to the **Continuities** of the curve (Mathematically sometimes called as the C0, C1, C2 continuities of the curve). For the given policy and for the given **Configuration**, we can say that this is a "Good" trajectory for the end-effector. 

# Dynamics of the mechanism
To fine tune any control, we need the respective **Torques** and **Forces** from the Dynamics modelling. Here, instead of a **Lagrangian Mechanics**, we've used am extension from the **Newton-Eulerian** mechanics. Briefly all of these formulations have been discussed in the notebook. As of this current model we do not consider Gravity at all for Torque Calculations. (Its an Ideal model). However, **the Kinematics results are VALID** even for the Ideal scenarios (as Kinematics doesnt care if the modelling is done with or without gravity. It only considers the physical geometry of the mechanism). The FBD (Free-Body-Diagram) of the object has been presented below: 
<div align="center">
  <img width="341" height="357" alt="image" src="https://github.com/user-attachments/assets/4b4f3b04-11c5-4236-94f0-219e92ef765e" />

  <br>
  <blockquote><b>Fig. 5</b> — <i>Free-Body-Diagram.</i></blockquote>
</div>
(All these are mapped with considerations of Newton's 3rd law and considerations of Pin/Revolute Joints). In the modelling, we have modelled the links as a rigid rod and hence their **Moments of Inertia** is chosen accordingly in the model. The combined Kinematics and Dynamics results of the torque variation is plotted and shown below. (Considering the link Lengths to be L1=L2=10m). 
<div align="center">
<img width="1262" height="1221" alt="D" src="https://github.com/user-attachments/assets/2b14a41b-a3e0-4fd0-a3db-c6b3394d5d5e" />
  <br>
  <blockquote><b>Fig. 6</b> — <i>Multi-Body-Dynamics analysis</i></blockquote>
</div>


# Pending Work
- Tuning Methods of Dynamics with Control
- Motion Planning
- Integration with the hardware 


