---
layout: page
title: 3D Single Compound Pendulum
description: Simulation of a single compound pendulum in 3D using MATLAB.
img: assets/img/single_pendulum/single_pendulum.png
importance: 1
category: work
---
### Overview
One of the challenges of 3D rigid body dynamics is that the angular momentum vector is not always aligned with the angular velocity vector (as it is in the 2D case). In 3D, the two quantities are related by the inertia tensor. To study the complications of 3D rigid body dynamics, I simulated a compound pendulum in 3D. To simplify the math, the pendulum will not be allowed to twist about its longitudinal axis. This is equivalent to connecting the pendulum to the base via a <a href="https://www.youtube.com/watch?v=LCMZz6YhbOQ&ab_channel=Lesics">universal joint</a>.

I took two different approaches to solving for the motion of the pendulum: (1) Newton-Euler method and (2) Lagrange method. Both methods are fully explored in the sections below.

### Newton-Euler Method
The Newton-Euler approach is the standard $$\sum \mathbf{F}=m\mathbf{a}$$ (linear moment balance) and $$\sum \mathbf{M}=I\mathbf{\alpha}$$ (angular momentum balance) approach to the problem. Furthermore, we will take the minimal coordinates approach, meaning that we will take into account the constraints to reduce the degrees of freedom of the system prior to deriving the equations of motion. This is in contrast to the maximal coordinates approach where we allow the system to have maximal degrees of freedom and we introduce constraint equations only at the end, which must be solved simultaneously with the equations of motion.

In general, you need 6 coordinates to fully describe the position and orientation of a rigid body in 3D space (three for position and three for orientation). Constraining the pendulum to be rigidly attached to the base reduces this to 3 coordinates (three angles). The no-twist condition further reduces this to two angles. I defined these two angles as $$\theta$$ and $$\phi$$ (shown in the figure below). Hence solving for the motion of the pendulum boils down to solving for the time evolution of these two angles.

<div class="row justify-content-sm-center">
    <div class="col-sm-8 mt-3 mt-md-0">
        {% include figure.html path="assets/img/single_pendulum/angles.jpg" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

First, we need to define the body frame. To define the body frame, we need to go through a series of two rotations. We start with the inertial frame $$I=(\hat{x},\hat{y},\hat{z})$$ and rotate it about $$\hat{x}$$ by an angle $$\theta$$ to obtain an intermediate frame $$A=(\hat{a}_1,\hat{a}_2,\hat{a}_3)$$.

<div class="row justify-content-sm-center">
    <div class="col-sm-8 mt-3 mt-md-0">
        {% include figure.html path="assets/img/single_pendulum/rotation1.jpg" class="img-fluid rounded z-depth-1" %}
    </div>
</div>


Notice that this is a negative rotation about $$\hat{x}$$ according to the right-hand rule. Using simple geometric relations, we can find the transformation equation between the two frames:

$$
\hat{x}=\hat{a}_1
$$

$$
\hat{y}=cos(\theta)\hat{a}_2+sin(\theta)\hat{a}_3
$$

$$
\hat{z}=-sin(\theta)\hat{a}_2+cos(\theta)\hat{a}_3
$$

We can put the three equations into a matrix $${}^IC^A$$, called the coordinate transformation matrix from frame $$I$$ to frame $$A$$:

$$
{}^IC^A=
\begin{bmatrix}
1 & 0 & 0 \\
0 & cos(\theta) & -sin(\theta) \\
0 & sin(\theta) & cos(\theta) \\
\end{bmatrix}
$$

For the second rotation, we rotate frame $$A$$ about $$\hat{a}_2$$ by an angle $$\phi$$ to obtain the body frame $$B=(\hat{b}_1,\hat{b}_2,\hat{b}_3)$$.

<div class="row justify-content-sm-center">
    <div class="col-sm-8 mt-3 mt-md-0">
        {% include figure.html path="assets/img/single_pendulum/rotation2.jpg" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

Again, we can find the transformation equations between the two frames:

$$
\hat{a}_1=cos(\phi)\hat{b}_1+sin(\phi)\hat{b}_3
$$

$$
\hat{a}_2=\hat{b}_2
$$

$$
\hat{a}_3=-sin(\phi)\hat{b}_1+cos(\phi)\hat{b}_3
$$

Then the transformation matrix from frame $$A$$ to frame $$B$$ is:

$$
{}^AC^B=
\begin{bmatrix}
cos(\phi) & 0 & -sin(\phi) \\
0 & 1 & 0 \\
sin(\phi) & 0 & cos(\phi) \\
\end{bmatrix}
$$

A third rotation is unnecessary because the pendulum is not allowed to twist.

To obtain the transformation matrix that transforms a vector from frame $$I$$ to frame $$B$$, we simply multiply the two matrices above (the order matters!):

$$
{}^IC^B={}^AC^B\cdot{}^IC^A=
\begin{bmatrix}
cos(\phi) & -sin(\phi)sin(\theta) & -cos(\theta)sin(\phi) \\
0 & cos(\theta) & -sin(\theta) \\
sin(\phi) & cos(\phi)sin(\theta) & cos(\phi)cos(\theta) \\
\end{bmatrix}
$$

The first column of the matrix expresses $$\hat{x}$$ in terms of $$\hat{b}_1$$, $$\hat{b}_2$$, and $$\hat{b}_3$$. The second column expresses $$\hat{y}$$ in terms of $$\hat{b}_1$$, $$\hat{b}_2$$, and $$\hat{b}_3$$. The third column expresses $$\hat{z}$$ in terms of $$\hat{b}_1$$, $$\hat{b}_2$$, and $$\hat{b}_3$$.

Note that these matrices do not rotate a vector in space. Instead, they transform the components of the a vector in one frame to the components in another frame (i.e. change of basis). The vector remains the same vector, but we're essentially expressing the components in another coordinate system. These matrices will come in handy when we're switching between reference frames. To transform the components in the reverse direction (for example, from $$B$$ to $$A$$ instead of from $$A$$ to $$B$$), we just invert the transformation matrix.

Like in any dynamics problem, the next step after defining the reference frame and coordinate system is to draw a free-body diagram of the system:

<div class="row justify-content-sm-center">
    <div class="col-sm-8 mt-3 mt-md-0">
        {% include figure.html path="assets/img/single_pendulum/fbd.jpg" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

It is important that we include the reaction moment $$M_3$$ which prevents the pendulum from twisting about its longitudinal axis. Also, we will model the pendulum as a thin rod of length $$l$$ and mass $$m$$. Notice that the axes of the body frame coincide with the pendulum's axes of symmetry. This was not an arbitrary choice. Choosing the axes of the body frame as the axes of symmetry will greatly simplify the angular momentum balance equations as will be shown later.

The next step is to perform a linear momentum balance in each of the coordinate directions of the body frame. But before we do so, we must derive an expression for the inertial acceleration of the center of mass of the pendulum in terms of the body frame unit vectors. To do this, we differentiate the position vector in frame $$B$$, $$\mathbf{r}_G/O=\frac{l}{2}\hat{b}_3$$, twice:

$$
{}^I\mathbf{a}_{G/O}=\frac{l}{2}[(\ddot{\phi}+\dot{\theta}^2sin(\phi)cos(\phi))\hat{b}_1+(\ddot{\theta}cos(\phi)-2\dot{\theta}\dot{\phi}sin(\phi)\hat{b}_2-(\dot{\phi}^2-\dot{\theta}^2cos^2(\phi))\hat{b}_3]
$$

Now we perform linear momentum balance in each of the coordinate directions. In the $$\hat{b}_1$$ direction:

$$
\frac{l}{2}(\ddot{\phi}+\dot{\theta}^2sin(\phi)cos(\phi))=\frac{1}{m}(R_1-cos(\theta)sin(\phi)F_g)
$$

In the $$\hat{b}_2$$ direction:

$$
\frac{l}{2}(\ddot{\theta}cos(\phi)-2\dot{\theta}\dot{\phi}sin{\phi})=\frac{1}{m}(R_2-sin(\theta)F_g)
$$

In the $$\hat{b}_3$$ direction:

$$
-\frac{l}{2}(\dot{\phi}^2-\dot{\theta}^2cos^2(\phi))=\frac{1}{m}(-R_3+cos(\phi)cos(\theta)F_g)
$$

Next, we perform angular momentum balance. Because we chose the pendulum's axes of symmetry as the axes of the body frame, the inertia tensor with respect to the body frame is diagonal:

$$
I=
\begin{bmatrix}
ml^2/3 & 0 & 0 \\
0 & ml^2/3 &  \\
0 & 0 & 0 \\
\end{bmatrix}
$$

When the body frame's axes coincide with the body's axes of symmetry, these axes called the principal axes. Also, we have used the fact that, because the rod is thin ($$r<<l$$), the moment of inertia about $$\hat{b}_3$$ is negligible.

We also need the angular velocity vector in the body frame. The angular velocity vector is:

$$
\mathbf{\omega}=\dot{\theta}\hat{a}_1+\dot{phi}\hat{b}_2
$$

From here, we just use the transformation matrices we previously derived to express everything in terms of $$\hat{b}_1$$, $$\hat{b}_2$$, and $$\hat{b}_3$$:

$$
\mathbf{\omega}=\dot{theta}cos(\phi)\hat{b}_1+\dot{\phi}\hat{b}_2+\dot{\theta}sin(\phi)\hat{b}_3
$$

Now we can write the equations for the angular momentum balance about the center of mass of the pendulum in the principal-axes body frame. Like with the linear momentum balance, we perform the angular momentum balance in each of the body frame coordinate directions. In the \hat{b}_1 direction:

$$
\frac{ml^2}{3}(\ddot{\theta}cos(\phi)-\dot{\theta}\dot{\phi}sin(\phi))-\frac{ml^2}{3}\dot{\theta}\dot{\phi}sin(\phi)=\frac{l}{2}F_gsin(\theta)

In the \hat{b}_2 direction:

$$
\frac{ml^2}{3}(\ddot{\phi})+\frac{ml^2}{3}\dot{\theta}^2sin(\phi)cos(\phi)=\frac{-l}{2}F_gcos(\theta)sin(\phi)
$$

In the \hat{b}_3 direction:

$$

$$

It is important to note that, had we used a body frame that was not the principal-axes body frame, the equations for angular momentum balance would have been much more complicated due to the inertia tensor having off-diagonal terms.
