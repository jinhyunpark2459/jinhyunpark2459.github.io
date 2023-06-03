---
layout: page
title: 3D Single Compound Pendulum
description: Simulation of a single compound pendulum in 3D using MATLAB.
img: assets/img/single_pendulum.png
importance: 1
category: work
---
### Overview
One of the challenges of 3D rigid body dynamics is that the angular momentum vector is not always aligned with the angular velocity vector (as it is in the 2D case). In 3D, the two quantities are related by the moment of inertia tensor. To study the complications of 3D rigid body dynamics, I simulated a compound pendulum in 3D. To simplify the math, the pendulum will not be allowed to twist about its longitudinal axis. This is equivalent to connecting the pendulum to the base via a <a href="https://www.youtube.com/watch?v=LCMZz6YhbOQ&ab_channel=Lesics">universal joint</a>.  

I took two different approaches to solving for the motion of the pendulum: (1) Newton-Euler method and (2) Lagrange method. Both methods are fully explored in the sections below.

### Newton-Euler Method
The Newton-Euler approach is the standard $$\sum \mathbf{F}=m\mathbf{a}$$ (linear moment balance) and $$\sum \mathbf{M}=I\mathbf{\alpha}$$ (angular momentum balance) approach to the problem.

In general, you need 6 coordinates to fully describe the position and orientation of a rigid body in 3D space (three for position and three for orientation). Constraining the pendulum to be rigidly attached to the base reduces this to 3 coordinates (three angles). The no-twist condition further reduces this to two angles. I defined these two angles as $$\theta$$ and $$\phi$$. Hence solving for the motion of the pendulum boils down to solving for the time evolution of these two angles.

<div class="row justify-content-sm-center">
    <div class="col-sm-8 mt-3 mt-md-0">
        {% include figure.html path="assets/img/angles.jpg" class="img-fluid rounded z-depth-1" %}
    </div>
</div>


First, we need to define the body frame. To define the body frame, we need to go through a series of two rotations. We start with the inertial frame $$I=(\hat{x},\hat{y},\hat{z})$$ and rotate it about $$\hat{x}$$ by an angle $$\theta$$ to obtain an intermediate frame $$A=(\hat{a}_1,\hat{a}_2,\hat{a}_3)$$.


<div class="row justify-content-sm-center">
    <div class="col-sm-8 mt-3 mt-md-0">
        {% include figure.html path="assets/img/rotation1.jpg" class="img-fluid rounded z-depth-1" %}
    </div>
</div>


Using simple geometric relations, we can find the transformation equation between the two frames:

$$
\hat{x}=\hat{a}_1
$$

$$
\hat{y}=cos(\theta)\hat{a}_2-sin(\theta)\hat{a}_3
$$

$$
\hat{z}=sin(\theta)\hat{a}_2+cos(\theta)\hat{a}_3
$$

We can put the three equations into a matrix $${}^IC^A$$, called the coordinate transformation matrix from frame $$I$$ to frame $$A$$:

$$
{}^IC^A=
\begin{bmatrix}
1 & 0 & 0 \\
0 & cos(\theta) & sin(\theta) \\
0 & -sin(\theta) & cos(\theta) \\
\end{bmatrix}
$$

For the second rotation, we rotate frame $$A$$ about $$\hat{a}_2$$ by an angle $$\phi$$ to obtain the body frame $$B=(\hat{b}_1,\hat{b}_2,\hat{b}_3)$$.

<div class="row justify-content-sm-center">
    <div class="col-sm-8 mt-3 mt-md-0">
        {% include figure.html path="assets/img/rotation2.jpg" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

Again, we can find the transformation equations between the two frames:

$$
{}^AC^B=
\begin{bmatrix}
cos(\phi) & 0 & -sin(\phi) \\
0 & 1 & 0 \\
sin(\phi) & 0 & cos(\phi) \\
\end{bmatrix}
$$

Like in any dynamics problem, the next step after defining the reference frame and coordinate system is to draw a free-body diagram of the system:
