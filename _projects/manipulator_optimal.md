---
layout: post
title: Optimal Control of a Two-link Planar Manipulator
description: Still a work in progress!
img: assets/img/robot_arm.png
category: work
importance: 4
redirect: /assets/pdf/two_link_manipulator.pdf
---
To get a deeper understanding of optimal control, I aim to compute an optimal control policy to control a two-link manipulator to a target pose. Since the system is highly nonlinear, linear methods such as LQR cannot be used on the system as is. However, feedback linearization may be used to linearize the system, after which LQR can be used. Using this approach, I was able to successfully track a target pose.

However, this approach is not truly optimal with respect to the original system. In an attempt to further optimize, so far, I have attempted to generate an optimal open-loop control policy using Pontryagin's maximum principle and control parameterization, both of which were unsuccessful. 

In another project where I generated an fuel-optimal trajectory for powered descent guidance, I used sequential convex programming (SCP) to generate a sequence of optimal thrusts. SCP is a widely applicable method since it can work with nonlinear systems with