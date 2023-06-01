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
The Newton-Euler approach is the standard '$$ \sum \mathbf{F}=m\mathbf{a} $$' (linear moment balance) and sum.'$$ \sum M=Ia $$' (angular momentum balance) approach to the problem.


{% raw %}
```html
<div class="row justify-content-sm-center">
    <div class="col-sm-8 mt-3 mt-md-0">
        {% include figure.html path="assets/img/6.jpg" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm-4 mt-3 mt-md-0">
        {% include figure.html path="assets/img/11.jpg" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
```
{% endraw %}
