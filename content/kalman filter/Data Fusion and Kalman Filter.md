
# Intro example
when given different measurement devices, how can we obtain the diameter of a coin?

### Test 1: Measuring tape 
we can use the measuring tape to measure the diameter of the radius twice, measurements $z_1$ and $z_2$. Taking the average gives the estimate $$
z_e=25\ \text{mm}
$$
![[1a.png]]
### Test 2: measuring tape + ruler of different resolutions
To obtain the estimate $z_e$ would we still average the measurements?
![[1b.png]]
### Test 3: measuring tape and micro meter
In this case, would we still take one measurement each and take the average? Even though the micrometer's resolution is much higher than the measuring tape? (1mm vs 0.001mm)
![[1c.png]]
## Intuition: No
Obviously, since sensor precisions differ, we can no longer simply average the readings from different sensors. Because, neglecting systematic error, a micrometer’s measurement is clearly more accurate than that of a tape measure.

So how can we obtain the best estimate of the coin’s diameter from measurements made by different sensors? That’s where the data‐fusion methods described later come in!

# Data fusion

## What does Data fusion do?
Using different sensors to achieve the best estimate of the system. In the example above, it would be to use different sensors (measuring devices) to give a best estimation of the coin's diameter.

## Prerequisite for Data Fusion — Uncertainty  
No sensor is perfectly accurate, nor is there any measurement process that is completely error‑free. In other words, every single measurement comes with some uncertainty. When we ignore systematic errors, a higher‑precision sensor will have a smaller measurement uncertainty. For example, in the case above, the measurement uncertainty of the micrometer is much smaller than that of the tape measure.

In statistics, uncertainty is expressed in terms of standard deviation, variance, and the covariance matrix. Clearly, in the previous example, when computing a weighted average of measurements from different sensors, we should take each sensor’s uncertainty into account: the smaller the uncertainty, the larger the weight its measurement should carry, because it is more accurate.
![[2a.png]]
If we know the standard deviation of the measurements produced by each sensor, we should be able to derive a weight $k$ in order to assign different weights (importance/trustworthiness) to each measurement.

## **Result of Data Fusion: The Statistically Optimal Estimate**  
Every measurement carries uncertainty, so from a statistical standpoint our measurement is a [random variable](https://math.stackexchange.com/questions/240673/what-exactly-is-a-random-variable), and the final estimate of the system state is also a random variable. The optimal data‐fusion result, therefore, should minimize variance; for a multivariate random variable this means achieving the smallest possible [trace](https://en.wikipedia.org/wiki/Trace_(linear_algebra)) of the covariance matrix.

Hence, when we derive the weighted average of the measurements from different sensors below, our goal is to choose weights so that the resulting weighted average has the minimum variance.

- **Optimal Estimation**: fusion result with the **minimum uncertainty**, i.e minimum $\sigma,\sigma^2,P$
### Example
Suppose we have measurements:
$$
\begin{aligned}
z_1 =& 24\ \text{mm}, \sigma_1=1\ \text{mm}\\
z_2 =& 25.003\ \text{mm}, \sigma_2=0.001\ \text{mm}
\end{aligned}
$$
**goal**: to find value $k$ such that the estimation has minimal uncertainty
$$
\begin{aligned}
z_e = (1-k)z_1+kz_2
\end{aligned}
$$
**ans:** recall that 
$$
\begin{aligned}
\sigma^2_{z_e}=& Var[(1-l)z_1+kz_2]\\
=& Var[(1-k)z_1]+ Var[kz_2]\\
=& (1-k)^2Var[z_1]+ k^2Var[z_2]\\
\end{aligned}
$$
due to properties of [variance](https://en.wikipedia.org/wiki/Variance#Properties), we have
$$
\begin{aligned}

 \sigma^2_{z_e}=& (1-k)^2\sigma_1^2 + k^2\sigma^2_2
\end{aligned}
$$
since $\sigma_1$ and $\sigma_2$ are constants in this case, we can differentiate $\sigma_{z_e}$ with respect to $k$ to find the minima.
$$
\begin{aligned}
\frac{d\sigma^2_{z_e}}{dk}=& -2 (1-k)\sigma^2_1+2k\sigma^2_2\\
0 =& -2 (1-k)\sigma^2_1+2k\sigma^2_2\\
k=&\frac{\sigma_1^2}{\sigma_1^2+\sigma_2^2}
\end{aligned}
$$

The result is easy to interpret in two scenarios:
* If $\sigma_1^2$ is very large, then $k$ tends toward 1, and the fused result approaches $z_2$. This makes sense: measurement 1 has a large variance (i.e., is less accurate), so we lean on measurement 2.
* If $\sigma_2^2$ is very large, then $k$ tends toward 0, and the fused result approaches $z_1$. The reasoning is analogous.
Plugging our derived formula into the previous example of measuring the coin’s diameter with a tape measure and a micrometer, we find that the final fused estimate lies very close to the micrometer’s reading, which is exactly the desirable outcome.
![[2b.png]]


# State Space Representation 
In robotic state‑estimation problems, it’s not simply a matter of taking two sensor measurements and fusing them. Instead, one usually builds a motion model of the system that lets us predict its state. When new sensor data arrives, we update that prediction based on the measurements, yielding a more accurate estimate of the system state.

This process is precisely the system’s state‑space formulation. In the Kalman Filter, it’s broken into two parts: the **state equation** and the **observation (measurement) equation**.
## State equation
We construct a mathematical model of the system’s physics. For example, in SLAM, if we assume the robot travels at constant velocity or under constant acceleration, then our models become the constant‑velocity model or the constant‑acceleration model, respectively.

In short, the state equation is something we calculate or derive. Given the system state at the previous time step, we use the state equation to compute the current system state, this constitutes our prediction.

Another point to note, as mentioned in the Data Fusion section, is that both measurements and our mathematical model carry uncertainty. For instance, our model may be imperfect, so the state equation is subject to noise. In the Kalman Filter, we assume this noise is Gaussian, an essential premise for deriving the Kalman Gain.
$$
x_k = A\,x_{k-1} + B\,u_{k-1} + w_{k-1},
$$

* **$A$**: state‑transition matrix 
* **$B$**: control‑input matrix
* **$u_{k-1}$**: control vector
* **$w_{k-1}$**: process noise, assumed to be drawn from multivariate normal distribution with zero‑mean and covariance $Q$


### $A$: State‑Transition Matrix
* **What it is**: A matrix that encodes your system’s built‑in dynamics—how the state moves forward in time if there were no external inputs or noise.
* **Role in the equation**: In
$$
x_k = A\,x_{k-1} + B\,u_{k-1} + w_{k-1},
$$
the term $A\,x_{k-1}$ is your *prediction* of the new state $x_k$ based solely on the old one.
* **Intuition**:
* If your state is just a 1D position and you assume it doesn’t change by itself, $A=1$.
* If your state is $\begin{bmatrix} \text{position}\\\text{velocity}\end{bmatrix}$ under a constant‑velocity model, then with timestep $\Delta t$:

$$
A = \begin{bmatrix}
1 & \Delta t\\
0 & 1
\end{bmatrix},
$$

because (from kinematics) 
$$
\text{pos}_k = \text{pos}_{k-1} + \Delta t\,\text{vel}_{k-1},\quad
$$
and (from constant velocity assumption)
$$
\text{vel}_k = \text{vel}_{k-1}.
$$
- **Details:** suppose our state $x_i=\begin{bmatrix} p_i\\ v_i\end{bmatrix}$ at some timestep $i$, then
$$
\begin{aligned}
x_k=&A\cdot x_{k-1} \\
 =& \begin{bmatrix}
1 & \Delta t\\
0 & 1
\end{bmatrix} \cdot \begin{bmatrix} p_{k-1}\\ v_{k-1}\end{bmatrix} \\
=& \begin{bmatrix}
1\cdot p_{k-1} + \Delta t\cdot v_{k-1}\\
0\cdot p_{k-1}+  1 \cdot v_{k-1}
\end{bmatrix} \\
=& \begin{bmatrix}
p_{k-1} + \Delta t v_{k-1}\\
v_{k-1}
\end{bmatrix}
\end{aligned}
$$
which matches above.

Q: So we have to derive $A$ ourselves every time we need to apply a kalman filter?

**$B$: Control‑Input Matrix**
* **What it is**: A matrix that describes *how* your known inputs or controls $u$ (e.g. commanded acceleration, wheel‑encoder speeds) push the state forward.
* **Role in the equation**: In
$$
x_k = A\,x_{k-1} + B\,u_{k-1} + w_{k-1},
$$
the term $B\,u_{k-1}$ injects the effect of your control actions.
* **Intuition**:
* If your control $u$ is a direct velocity command in a 1D position‑only model, you might set
$$
A=1,\quad B=\Delta t,
$$
so that $B\,u = \Delta t\cdot(\text{velocity})$ updates your position.

* In the 2‑state $[\text{pos};\text{vel}]$ constant‑acceleration model, if $u$ is acceleration $a$, you’d choose

$$

B = \begin{bmatrix}

\tfrac12\,\Delta t^2\\[6pt]

\Delta t

\end{bmatrix},

$$
because
$$

\text{pos increment} = \tfrac12\,a\,\Delta t^2,\quad

\text{vel increment} = a\,\Delta t.

$$

#### Summary
* **$A$** tells you “where the system would go on its own.”
* **$B$** tells you “how external commands or controls nudge it.”


### Observation Equation
This is just like our earlier example of measuring the coin’s diameter: it describes how we observe the system’s state. In SLAM, however, the observation is often indirect and mediated by an observation model. For instance, in visual‑inertial odometry (VIO) we want to estimate the system’s 6 DOF pose, but what the camera actually gives us are pixel coordinates of feature points, not the pose itself. Those pixel measurements are related to the 6 DOF state through the camera’s projection model. In other words, the projection model ties the _actual_ observed pixel values to the _true_ pose, and that relationship is our observation equation.

In summary, although the observation equation doesn’t measure the system state directly, it _indirectly_ measures it via the observation model, so it still constitutes a measurement of the state.

Likewise, observations are noisy: feature‐point locations may be imprecise, and the projection model itself might not be perfect. Therefore we add Gaussian noise to the observation equation, and it must be Gaussian, because that assumption is necessary for deriving the Kalman Gain.


**Intuition:**
At each time step $k$, you get a new sensor reading $z_k$. The Kalman‐filter observation equation

$$
z_k = H_k\,x_k \;+\; v_k
$$

says that:

1. **$x_k$** is the true system state at time $k$ (e.g.\ position/velocity, 6‑DOF pose, etc.).

2. **$H_k$** is the **observation matrix** (or model) that tells you *how* the state maps into whatever your sensor actually measures.

* If your state is $\begin{bmatrix}\text{pos}\\\text{vel}\end{bmatrix}$ but your sensor only reads position, then

$$

H = \begin{bmatrix}1 & 0\end{bmatrix}

$$

so that $z_k = [1\;\;0]\begin{bmatrix}\text{pos}_k\\\text{vel}_k\end{bmatrix} = \text{pos}_k$.

* In a camera case, $H_k$ would be the projection from 3‑D pose into pixel coordinates.

3. **$v_k$** is the **measurement noise**, capturing all the ways your sensor might err (electronic noise, quantization, feature‐extraction error, etc.). We model it as

$$

v_k \sim \mathcal{N}(0,\,R_k),

$$

meaning zero‑mean Gaussian with covariance $R_k$. The size and shape of $R_k$ encode how much trust you place in that sensor—and whether different measurements are correlated.