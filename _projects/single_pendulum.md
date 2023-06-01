---
layout: page
title: 3D Single Compound Pendulum
description: Simulation of a single compound pendulum in 3D using MATLAB.
img: assets/img/single_pendulum.png
importance: 1
category: work
---
### Overview
One of the challenges of 3D rigid body dynamics is that the angular momentum vector is not always aligned with the angular velocity vector (as it is in the 2D case). In 3D, the two quantities are related by the moment of inertia tensor. To study the complications of 3D rigid body dynamics, I simulated a compound pendulum in 3D.

I took two different approaches to solving for the motion of the pendulum: (1) Newton-Euler method and (2) Lagrange method. Both methods are fully explored in the sections below.

### Newton-Euler Method
The Newton-Euler approach is the standard $$ \sum \mathbf{F}=m\mathbf{a} $$ (linear moment balance) and $$ \sum \mathbf{M}=I\mathbf{\alpha} $$ (angular momentum balance) approach to the problem.

First, the position of the pendulum can be fully defined by two angles, $$ \theta $$ and $$ \phi $$. Hence solving for the motion of the pendulum boils down to solving for the time evolution of these two angles.

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/angles.png" class="img-fluid rounded z-depth-1" %}
    </div>
