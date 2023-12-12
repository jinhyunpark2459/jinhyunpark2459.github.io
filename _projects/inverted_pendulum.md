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

1. The gain of the controller C(s) should be below 85 dB for all frequencies
2. The gain of the controller C(s) at $$10^4$$ rad/s should be below 30 dB
3. Phase margin $$\phi > 10^{\circ}$$
4. Gain margin $$\vert GM \vert > 3 dB$$
5. Disturbance rejection settling time $$T^{10%}_{s} < 2s$$
6. $$\vert p \vert < 80$$ for all poles
7. The controller must be proper.

First thing we should do is look at the root locus of the open-loop transfer function to assess its stability:

<div class="row justify-content-sm-center">
    <div class="col-sm-8 mt-3 mt-md-0">
        {% include figure.html path="assets/img/inverted_pendulum/OLrlocus.jpg" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

We can see from the root locus plot that the system has an open-loop pole in the RHP and is hence unstable. Then we should pull the root locus in the RHP into the LHP by placing zeros near the origin (because zeros “attract” poles). Let us place the two zeros at $$s=-1$$ and $$s=-1.5$$. To make the controller proper while not messing up the root locus around the two zeros we just placed, we will add two poles at $$s=-80$$ far away from the rest of the pole/zeros. Then by adjusting the gain to -5, we are able to stabilize the system:

<div class="row justify-content-sm-center">
    <div class="col-sm-8 mt-3 mt-md-0">
        {% include figure.html path="assets/img/inverted_pendulum/stable_rlocus.jpg" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

<div class="row justify-content-sm-center">
    <div class="col-sm-8 mt-3 mt-md-0">
        {% include figure.html path="assets/img/inverted_pendulum/stable_bode.jpg" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

Noticed that the magnitude plot near the cross-over frequency is very flat. This makes it difficult to control the crossover frequency by adjusting the gain. Ideally, we want the magnitude graph to decrease steadily near the crossover frequency. So, we should move the poles at 80 rad/s down closer to the crossover frequency to 30 rad/s to decrease the slope near the crossover frequency (~5 rad/s).

<div class="row justify-content-sm-center">
    <div class="col-sm-8 mt-3 mt-md-0">
        {% include figure.html path="assets/img/inverted_pendulum/2.jpg" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
