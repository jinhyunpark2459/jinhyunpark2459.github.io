---
layout: page
title: Path Tracking Control of a Small Mobile Robot
description: Designed and implemented a path tracking controller of the Thymio robot using ROS.
img: assets/img/path_tracking.png
category: work
---
For my senior design project, I designed and implemented a path tracking controller on a mobile robot using the Robot Operating System (ROS). More specifically, I implemented the pure pursuit algorithm to get the robot to follow various paths by providing a list of waypoints to the algorithm. <a href="https://jinhyunpark2459.github.io/assets/pdf/senior_design.pdf">Here</a> is a comprehensive report for the project and the code can be found <a href="https://github.com/apark2459/thymio-path-tracking">here</a>. Here is a video of the robot following a star-shaped path:
<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.html path="assets/video/thymio_star.mp4" class="img-fluid rounded z-depth-1" controls=true autoplay=false %}
    </div>
</div>