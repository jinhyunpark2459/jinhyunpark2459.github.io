---
layout: page
title: Frequency Domain Control of an Inverted Pendulum
description: Walk-through of controller design for an inverted pendulum using frequency domain methods
img: assets/img/inverted_pendulum/inverted_pendulum.png
importance: 4
category: work
---

### Problem Setup
In this post, I am going to walk through how I approached the design of a controller for an inverted pendulum in the frequency domain. First, let’s start by exploring the open-loop dynamics of the inverted pendulum. The linearized  equations of motion of the inverted pendulum about the upright position are given below:

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
where $$x_{c}$$ is the position of the cart, $$\theta$$ is the angle the pendulum makes with the vertical axis, $$M_{c}$$ is the mass of the cart, $$M_{R}$$ is the mass of the pendulum rod, $$L$$ is the length of the pendulum rod, and $$I_{rot}$$ is the motor's moment of inertia.

The input into the system is $$f_{c}$$, the force on the cart. But we don’t directly control $$f_{c}$$. The actual control input that we get to choose is the voltage supplied to the cart’s motors, $$V$$. $$f_{c}$$ is given by the following equation:

$$
f_{c}=\alpha_{1}V+\alpha_{2}\dot{x}_{c}
$$

where $$\alpha_{1}$$ and $$\alpha_{2}$$ are system parameters that were empirically determined and $$\dot{x}_{c}$$ is the velocity of the cart (so the second term is a drag term).

The output of the system (i.e. the quantity we measure) is the following:

$$
x=x_{c}-\frac{2}{3}Lsin(\theta)
$$

We really measured both $$x_{c}$$ and $$\theta$$ but since frequency domain analysis is limited to single-input single-output (SISO) systems, we effectively lump these two measurements to get a single output so that we can design our controller in the frequency domain. At this point, by lumping the two measurements into one, we are throwing away information because we don’t know how to use it. When we design a controller for the same system in state-space in the next post, we will see how both measurements can be used.

Now that both the input and the output have been defined, we can obtain the open-loop transfer function of the plant from $$V$$ to $$x$$. For brevity, I will skip the derivation.

$$
G(s)=\frac{-14.7}{s(0.4396s^3+4s^2-12.3126s-98)}
$$

### Controller Design
Given the open-loop transfer function, we can now begin to design the controller. Before getting started, the following is a list of performance requirements that the controller must satisfy: