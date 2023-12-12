---
layout: page
title: Inverted pendulum Control: Controller Design in the Frequency Domain
description: Designing a controller to stabilize an inverted pendulum using frequency domain techniques.
img: assets/img/inverted_pendulum/inverted_pendulum.png
importance: 4
category: work
---
### Problem Setup
In this post, I am going to walk through how I approached the design of a controller for an inverted pendulum in the frequency domain. First, let’s start by exploring the open-loop dynamics of the inverted pendulum. The dynamics of the inverted pendulum are given by the following equation:

$$
\begin{bmatrix}
M_{c}+M_{R}+I_{mot}\frac{k^2}{R^2} & -\frac{1}{2}M_{R}L \\
-1 & frac{2}{3}L \\
\end{bmatrix}
\begin{bmatrix}
$$

The input into the system is $$f_{c}$$ – the force on the cart. But we don’t directly control $$f_{c}$$. The actual control input that we get to choose is the voltage supplied to the cart’s motors, $$V$$. $$f_{c}$$ is given by the following equation:

$$
f_{c}=\alpha_{1}V+\alpha_{2}x_dot_{c}
$$

where $$\alpha_{1}$$ and $$\alpha_{2}$$ are system parameters that were empirically determined and $$x_dot_{c}$$ is the velocity of the cart (so the second term is a drag term).

The output of the system (i.e. the quantity we measure) is the following:

$$
x=x_c-\frac{2}{3}Lsin(\theta)
$$

We really measured both $$x_{c|$$ and $$\theta$$ but since frequency domain analysis is limited to single-input single-output (SISO) systems, we effectively lump these two measurements to get a single output so that we can design our controller in the frequency domain. At this point, by lumping the two measurements into one, we are throwing away information because we don’t know how to use it. When we design a controller for the same system in state-space in the next post, we will see how both measurements can be used.

Now that both the input and the output have been defined, we can obtain the open-loop transfer function of the plant from V to x. For brevity, I will skip the derivation.

$$
G(s)=\frac{-14.7}{s(0.4396s^3+4s^2-12.3126s-98)}
$$

### Controller Design
Given the open-loop transfer function, we can now begin to design the controller. Before getting started, the following is a list of performance requirements that the controller must satisfy:




<div class="row justify-content-sm-center">
    <div class="col-sm-8 mt-3 mt-md-0">
        {% include figure.html path="assets/img/single_pendulum/fbd.jpg" class="img-fluid rounded z-depth-1" %}
    </div>
</div>



$$
{}^I\mathbf{a}_{G/O}=\frac{l}{2}[(\ddot{\phi}+\dot{\theta}^2sin(\phi)cos(\phi))\hat{b}_1+(\ddot{\theta}cos(\phi)-2\dot{\theta}\dot{\phi}sin(\phi)\hat{b}_2-(\dot{\phi}^2-\dot{\theta}^2cos^2(\phi))\hat{b}_3]
$$

Now we write the linear momentum balance equations in each of the basis vector directions.

In the $$\hat{b}_1$$ direction:

$$
\frac{l}{2}(\ddot{\phi}+\dot{\theta}^2sin(\phi)cos(\phi))=\frac{1}{m}(R_1-cos(\theta)sin(\phi)F_g)
$$

In the $$\hat{b}_2$$ direction:

$$
\frac{l}{2}(\ddot{\theta}cos(\phi)-2\dot{\theta}\dot{\phi}sin{\phi})=\frac{1}{m}(R_2-sin(\theta)F_g)
$$

In the $$\hat{b}_3$$ direction:

$$
-\frac{l}{2}(\dot{\phi}^2+\dot{\theta}^2cos^2(\phi))=\frac{1}{m}(-R_3+cos(\phi)cos(\theta)F_g)
$$

Next, we perform the angular momentum balance. Because we chose the pendulum's axes of symmetry as the axes of the body frame, the inertia tensor is diagonal:

$$
I=
\begin{bmatrix}
ml^2/12 & 0 & 0 \\
0 & ml^2/12 &  \\
0 & 0 & 0 \\
\end{bmatrix}
$$

When the body frame's axes coincide with the body's axes of symmetry, these axes are called the principal axes. Also, we have used the fact that, because the rod is thin ($$r<<l$$), the moment of inertia about $$\hat{b}_3$$ is negligible.

We also need the angular velocity vector in the body frame. The angular velocity of frame $$B$$ with respect to frame $$I$$ is:

$$
{}^I\mathbf{\omega}^B=-\dot{\theta}\hat{x}+\dot{\phi}\hat{a}_2
$$

From here, we just use the transformation matrices we previously derived to express the angular velocity vector in terms of $$\hat{b}_1$$, $$\hat{b}_2$$, and $$\hat{b}_3$$:

$$
{}^I\mathbf{\omega}^B=-\dot{\theta}cos(\phi)\hat{b}_1+\dot{\phi}\hat{b}_2-\dot{\theta}sin(\phi)\hat{b}_3
$$

Now we can write the equations for the angular momentum balance about the center of mass of the pendulum with respect to the body frame. This implies that we must compute the moment exerted by external forces about the center of mass. Like with the linear momentum balance, we consider the angular momentum in each of the basis vector directions separately.

In the $$\hat{b}_1$$ direction:

$$
\frac{ml^2}{3}(-\ddot{\theta}cos(\phi)+\dot{\theta}\dot{\phi}sin(\phi))+\frac{ml^2}{3}\dot{\theta}\dot{\phi}sin(\phi)=\frac{l}{2}R_2
$$

In the $$\hat{b}_2$$ direction:

$$
\frac{ml^2}{3}\ddot{\phi}+\frac{ml^2}{3}\dot{\theta}^2sin(\phi)cos(\phi)=\frac{-l}{2}R_1
$$

In the $$\hat{b}_3$$ direction:

$$
0=M_3
$$

It is important to note that, had we used a body frame that was not the principal-axes body frame, the equations for angular momentum balance would have been much more complicated due to the inertia tensor having nonzero off-diagonal terms.

Now we have system of 6 equations (3 from LMB, 3 from AMB) for 6 unknowns ($$R_1$$, $$R_2$$, $$R_3$$, $$M_3$$, $$\ddot{\theta}$$, $$\ddot{\phi}$$). Unlike the maximal coordinates approach, we do not have a separate constraint equation. The constraint that the rod is rigidly attached to the base was already taken into account when we derived an expression for $${}^I\mathbf{a}_{G/O}$$. Had we approached this problem using the maximal coordinates approach, we would have had an additional degree of freedom from a third rotation about $$\hat{b}_3$$ by an angle $$\psi$$ and added a constraint equation $$\dot{\psi}=0$$ to our system of equations. Also, we would not have assumed that the pendulum had a fixed length when deriving an expression for $${}^I\mathbf{a}_{G/O}$$ and instead would have added a second constraint equation $$x^2+y^2+z^2=l^2$$ to our system of equations.

Next step is to solve for the unknowns. To do this, I used MATLAB's symbolic toolbox to write down the six equations symbolically, then used the `solve` function to solve for the six unknowns. Then I stored $$\ddot{\theta}$$, $$\ddot{\phi}$$, $$\dot{\theta}$$, $$\dot{\phi}$$ into a vector, turned it into a MATLAB function using `matlabFunction`, and plugged it into the `ode45` solver to solve for the motion of the pendulum.

The resulting animation of the pendulum with various initial conditions are below:

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include video.html path="assets/video/NE_single_pendulum_1.mp4" class="img-fluid rounded z-depth-1" controls=true autoplay=false %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include video.html path="assets/video/NE_single_pendulum_2.mp4" class="img-fluid rounded z-depth-1" controls=true %}
    </div>
</div>

As a sanity check, I plotted the energy versus time plot for the pendulum for the initial conditions corresponding to the second video:

<div class="row justify-content-sm-center">
    <div class="col-sm-8 mt-3 mt-md-0">
        {% include figure.html path="assets/img/single_pendulum/energy.png" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

As you can see in the plot, the total mechanical energy stays approximately constant. It does not exactly stay constant due to numerical errors. I also noticed that giving the pendulum a large initial angular velocity amplifies the numerical error that accumulates over time.

### Lagrange Method

The Lagrange method is much simpler than the Newton-Euler method. The Lagrangian of a system is defined as follows:

$$
L = T - V
$$

where $$L$$ is the Lagrangian of the system, $$T$$ is the kinetic energy of the system, and $$V$$ is the potential energy of the system.

For the single compound pendulum system, the kinetic energy is given by:

$$
T = \frac{1}{2}m(v_x^2+v_y^2+v_z^2)+\frac{1}{2}(I_1\omega_1^2+I_2\omega_2^2+I_3\omega_3^2)
$$

$$I_1$$, $$I_2$$, $$I_3$$ are the moments of inertia of the pendulum about the $$\hat{b}_1$$,$$\hat{b}_2$$,and $$\hat{b}_3$$ directions, respectively. $$\omega_1$$, $$\omega_2$$, $$\omega_3$$ are the angular velocity components in the $$\hat{b}_1$$,$$\hat{b}_2$$,and $$\hat{b}_3$$ directions, respectively. $$v_x$$, $$v_y$$, $$v_z$$ are the velocity components of the center of mass of the pendulum in the $$\hat{x}$$, $$\hat{y}$$, $$\hat{z}$$ directions, respectively. The Cartesian coordinates of the center of mass of the pendulum are:

$$
x = \frac{l}{2}sin(\phi)
$$

$$
y = \frac{l}{2}cos(\phi)sin(\theta)
$$

$$
z = \frac{l}{2}cos(\phi)cos(\theta)
$$

We can differentiate the three equations to get the three velocity components:

$$
v_x = \frac{l}{2}\dot{\phi}cos(\phi)
$$

$$
v_y = -\frac{l}{2}\dot{\phi}sin(\theta)sin(\phi)+\frac{l}{2}\dot{\theta}cos(\theta)cos(\phi)
$$

$$
v_z = -\frac{l}{2}\dot{\phi}cos(\theta)sin(\phi)-\frac{l}{2}\dot{\theta}sin(\theta)cos(\phi)
$$

The three moments of inertia terms and the angular velocity components were already discussed in the previous section.

Next, we need to obtain an expression for the potential energy of the pendulum:

$$
V = -mgz=-\frac{mgl}{2}cos(\phi)cos(\theta)
$$

Now we have an expression for the Lagrangian:

$$
L=\frac{ml^2\dot{\phi}^2}{6}+\frac{ml^2\dot{\theta}^2cos(\phi)^2}{6}+\frac{mglcos(\theta)cos(\phi)}{2}
$$

Once we have an expression for the Lagrangian, the equations of motion can be obtained by plugging in $$L$$ into the Euler-Lagrange equation:

$$
\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{q}_i}\right)-\frac{\partial L}{\partial q_i}=0
$$

where $$q_i$$ are the generalized coordinates of the system. For our system, we have two generalized coordinates $$q_1=\theta$$ and $$q_2=\phi$$. Plugging $$L$$ into the Euler-Lagrange equation will give us two equations (one containing $$\ddot{\theta}$$ and one containing $$\ddot{\phi}$$).

Again, we can use MATLAB to do all of the heavy lifting for us. As we've done before, we can take the two equations and solve for $$\ddot{\theta}$$ and $$\ddot{\phi}$$ using the `solve` function. Then store $$\ddot{\theta}$$, $$\ddot{\phi}$$, $$\dot{\theta}$$, $$\dot{\phi}$$ into a vector, turn it into a MATLAB function using `matlabFunction` and plug it into `ode45` to solve for the motion of the pendulum.

I was able to confirm that the equations of motion obtained using the Lagrange method matched the ones I obtained using the Newton-Euler method. As a result, the animation rendered using the Lagrange method was identical to the one rendered using the Newton-Euler method.

### Tips for Animating the Solution
To animate the solution, I wrote a function that creates a cylindrical patch object at the desired orientation $$(\theta$$,$$\phi$$). This function takes five arguments: (1) Starting point of the cylinder, (2) radius of the cylinder, (3) length of the cylinder (4) $$\theta$$, and (5) $$\phi$$. My advice is to first create the cylinder using the `cylinder` function (which basically generates a matrix containing the coordinates of the surface points of the cylinder), rotate it by the two angles $$\theta$$ and $$\phi$$, then turn it into a patch object using the `patch` function. You may want to consider writing a separate helper function for rotating the surface points of the cylinder. This will involve using the <a href="https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula">Rodrigues' rotation formula</a>.
