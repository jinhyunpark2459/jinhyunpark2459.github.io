---
layout: page
title: 3D Double Compound Pendulum
description: Simulation of a double compound pendulum in 3D using MATLAB.
img: assets/img/double_pendulum/double_pendulum.png
importance: 2
category: work
---
### Overview
In my <a href="https://jinhyunpark2459.github.io/projects/single_pendulum/">previous post</a>, I simulated a single compound pendulum in 3D. In this post, I will be extending this to a double compound pendulum in 3D. I will be using the Lagrange method only. Although the Newton-Euler method will not be explored in this post, it is not conceptually any different than what was done for the single pendulum. The only additional complexity would be that there will be additional reaction forces and moments on the first pendulum. Just as in the single pendulum, the two pendulums will not be allowed to twist about their longitudinal axes.

### Lagrange Method
In general, it takes a total of twelve coordinates to fully describe a system with two rigid bodies. Six of these describe the position of the two bodies and the other six describe the orientation of the two bodies. Constraining the first pendulum to be rigidly attached to the base and constraining the second pendulum to be rigidly attached to the first pendulum reduces this to six coordinates. The no-twist condition further reduces this to four coordinates.The four coordinates will be defined as $$\theta_1$$, $$\phi_1$$, $$\theta_2$$, and $$\phi_2$$ (see figure below).

<div class="row justify-content-sm-center">
    <div class="col-sm-8 mt-3 mt-md-0">
        {% include figure.html path="assets/img/double_pendulum/angles.jpg" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

We need to define two body frames $$B_1=(\hat{b}_{1,1},\hat{b}_{1,2},\hat{b}_{1,3})$$ and $$B_2=(\hat{b}_{2,1},\hat{b}_{2,2},\hat{b}_{2,3})$$. Frame $$B_1$$ pertains to the first pendulum and frame $$B_2$$ pertains to the second pendulum. The process of deriving the coordinate transformation matrices is same as the <a href="https://jinhyunpark2459.github.io/projects/single_pendulum/">single pendulum case</a>.

The angular velocity of the first pendulum is given by the following:

$$
{}^I\mathbf{\omega}^{B_1}=-\dot{\theta}_1cos(\phi_1)\hat{b}_{1,1}+\dot{\phi}_1\hat{b}_{1,2}-\dot{\theta}_1sin(\phi_1)\hat{b}_{1,3}
$$

The angular velocity of the second pendulum is given by the following:

$$
{}^I\mathbf{\omega}^{B_2}=-\dot{\theta}_2cos(\phi_2)\hat{b}_{2,1}+\dot{\phi}_2\hat{b}_{2,2}-\dot{\theta}_2sin(\phi_2)\hat{b}_{2,3}
$$
