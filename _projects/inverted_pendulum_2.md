---
layout: page
title: State-Space Control of an Inverted Pendulum
description: This article provides a walk-through of controller design for an inverted pendulum using frequency domain methods state-space methods
img: assets/img/inverted_pendulum/inverted_pendulum.png
importance: 4
category: work
---

### Problem Setup
In the previous post, I gave a walk-through how we could design a controller for the inverted pendulum using frequency domain techniques. In this post, we will the design of a controller for the same system using state-space methods. Recall the linearized  equations of motion of the inverted pendulum about the upright position:

$$
\begin{bmatrix}
M_{c}+M_{R}+I_{mot}\frac{k^2}{R^2} & -\frac{1}{2}M_{R}L \\
-1 & \frac{2}{3}L \\
\end{bmatrix}
\begin{bmatrix}
\ddot{x}_{c} \\
\ddot{\theta}
\end{bmatrix}
-
\begin{bmatrix}
0 \\
g
\end{bmatrix}
\theta
=
\begin{bmatrix}
1 \\
0
\end{bmatrix}
f_{c}
$$

where $$x_{c}$$ is the position of the cart, $$\theta$$ is the angle the pendulum makes with the vertical axis, $$M_{c}$$ is the mass of the cart, $$M_{R}$$ is the mass of the pendulum rod, $$L$$ is the length of the pendulum rod, and $$I_{rot}$$ is the motor's moment of inertia. The input into the system is $$f_{c}=\alpha_{1}V+\alpha_{2}\dot{x}_{c}$$, where we get to control the voltage supplied to the cart’s motors, $$V$$.

For ease of notation, let us define the following:

$$
H\triangleq\begin{bmatrix}
M_{c}+M_{R}+I_{mot}\frac{k^2}{R^2} & -\frac{1}{2}M_{R}L \\
-1 & \frac{2}{3}L \\
\end{bmatrix}
$$

Then we can rewrite the linearized equations of motion in state-space representation as the following:

$$
\begin{bmatrix}
\dot{x}_{c} \\
\dot{\theta} \\
\ddot{x}_{c} \\
\ddot{\theta}
\end{bmatrix}
=
\begin{bmatrix}
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\begin{bmatrix} 0 \\ 0 \end{bmatrix} & H^{-1}\begin{bmatrix} 0 \\ g \end{bmatrix} & H^{-1}\begin{bmatrix} \alpha_{2} \\ 0 \end{bmatrix} & \begin{bmatrix} 0 \\ 0 \end{bmatrix}
\end{bmatrix}
\begin{bmatrix}
\dot{x}_{c} \\
\dot{\theta} \\
x_{c} \\
\theta
\end{bmatrix}
+H^{-1}\begin{bmatrix} \alpha_{1} \\ 0 \end{bmatrix}u
$$

This tells us the A and B matrices:

$$A=
\begin{bmatrix}
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\begin{bmatrix} 0 \\ 0 \end{bmatrix} & H^{-1}\begin{bmatrix} 0 \\ g \end{bmatrix} & H^{-1}\begin{bmatrix} \alpha_{2} \\ 0 \end{bmatrix} & \begin{bmatrix} 0 \\ 0 \end{bmatrix}
\end{bmatrix}
$$

$$B=
H^{-1}\begin{bmatrix} \alpha_{1} \\ 0 \end{bmatrix}
$$

We will design two controllers: (1) a SISO (single-input single-output) controller and (2) a SIMO (single-input multiple-output) controller. For controller (1), we have a scalar output $$y=x_{c}-\frac{2}{3}L\theta$$ that lumps together the two measurements $$x_{c}$$ and $$\theta$$. Note that this output is basically the x position of the 1/3 point of the rod from the base. For controller (2), the output is a 2 by 1 vector, $$\begin{bmatrix} x_{c} \\ \theta \end{bmatrix}$$. Consequently, the two controllers will have difference C matrices. For controller (1),

$$
C_{SISO}=
\begin{bmatrix}
1 & -\frac{2}{3}L & 0 & 0
\end{bmatrix}
$$

For controller (2),

$$
C_{SIMO}=
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0
\end{bmatrix}
$$

## Review of Theory

In both cases of the SISO system and the SIMO system, we do not have access to the full state. That is, we need to implement an observer (i.e. state estimator). The block diagram that implements this looks like the following:

<div class="row justify-content-sm-center">
    <div class="col-sm-8 mt-3 mt-md-0">
        {% include figure.html path="assets/img/inverted_pendulum/blockdiagram1.png" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

Or equivalently,

<div class="row justify-content-sm-center">
    <div class="col-sm-8 mt-3 mt-md-0">
        {% include figure.html path="assets/img/inverted_pendulum/blockdiagram2.png" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

the block diagram looks like the following:


### Controller Design