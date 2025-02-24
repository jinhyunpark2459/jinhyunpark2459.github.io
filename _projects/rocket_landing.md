---
layout: page
title: Fuel-Optimal Rocket Landing
description: Implementation of a Sequential Convex Programming (SCP) algorithm for fuel-optimal rocket landing. 
img: assets/img/3d_trajectory.jpeg
category: work
---
I recently developed an interest in GNC applications for convex programming and implemented an SCP algorithm for a fuel-optimal rocket landing. While my implementation began using MATLAB and was almost convergent, I adopted <a href="https://github.com/EmbersArc/SCvx">Sven Niederberger's implementation</a> in Python which automates symbolic calculations. I modified his implmentation to implement lossless convexification and also include aerodynamic effects. The code is <a href="https://github.com/apark2459/SCP_python?tab=readme-ov-file">here</a> and my write-up of the project is <a href="https://jinhyunpark2459.github.io/assets/pdf/SCP_for_Fuel_Optimal_Rocket_Landing">here</a>.

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.html path="assets/video/SCvx_Simulation.mp4" class="img-fluid rounded z-depth-1" controls=true autoplay=false %}
    </div>
</div>