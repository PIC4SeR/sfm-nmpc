# Cost Function Formulation

The controller solves a nonlinear least-squares optimisation problem at each control cycle using the Ceres Solver. The decision variables are $N_b$ velocity blocks $\mathbf{u}_b = [v_b,\, \omega_b]^\top$, where each block governs $B$ consecutive timesteps (block length), yielding a control horizon of $H = N_b \times B$ steps at interval $\Delta t$. The total cost minimised by Ceres is:

$$J(\mathbf{u}) = \frac{1}{2}\sum_{i}\, r_i(\mathbf{u})^2$$

where each residual $r_i = w_i \cdot f_i(\mathbf{u})$ is a weighted scalar. The predicted robot state at timestep $t$ is obtained by forward-integrating a unicycle kinematic model from the current pose $\mathbf{p}_0$:

$$x_{t+1} = x_t + v_b \cos\theta_t \,\Delta t, \quad y_{t+1} = y_t + v_b \sin\theta_t \,\Delta t, \quad \theta_{t+1} = \theta_t + \omega_b \,\Delta t$$

For social critics, agent trajectories are co-predicted using the Social Force Model (SFM) with parameters $\lambda = 2.0$, $\gamma = 0.35$, $n = 2.0$, $n' = 3.0$, and $A_{\text{social}} = 2.1$, applied iteratively through the function `computeSFMState`.

All critics below are instantiated **once per timestep** $t \in \{0, \ldots, H{-}1\}$ unless stated otherwise.

---

## Navigation Critics

### 1. Path Distance Cost (DistanceCost)

Penalises the squared Euclidean distance between the predicted robot position and a reference waypoint on the global path:

$$r_{\text{dist}}^{(t)} = w_{\text{dist}} \cdot \left\lVert \mathbf{p}_t - \mathbf{p}_{\text{ref}}^{(t)} \right\rVert^2$$

This term is used in two modes controlled by separate enable flags: *path follow* (endpoint tracking toward the farthest reachable waypoint) and *path align* (per-waypoint lateral tracking). Both share the same functional form but differ in which reference point $\mathbf{p}_{\text{ref}}^{(t)}$ is used.

### 2. Path Angle Cost (AngleCost)

Penalises the squared, wrapped angular deviation between the robot's predicted heading $\theta_t$ and the heading of the reference path segment $\phi_{\text{ref}}^{(t)}$:

$$r_{\text{angle}}^{(t)} = w_{\text{angle}} \cdot \left(\mathrm{atan2}\!\big(\sin(\phi_{\text{ref}} - \theta_t),\, \cos(\phi_{\text{ref}} - \theta_t)\big)\right)^2$$

The $\mathrm{atan2}(\sin(\cdot), \cos(\cdot))$ wrapping ensures the difference remains in $[-\pi, \pi]$, avoiding discontinuities at $\pm\pi$.

### 3. Obstacle Cost (ObstacleCost)

Queries the Nav2 costmap at a point offset 0.25 m ahead of the robot (approximating the front bumper of a Jackal platform), using bi-cubic interpolation for sub-cell accuracy:

$$r_{\text{obs}}^{(t)} = w_{\text{obs}} \cdot \mathcal{C}\!\left(\mathbf{p}_t + 0.25\,[\cos\theta_t,\, \sin\theta_t]^\top\right)$$

where $\mathcal{C}(\cdot) \in [0, 254]$ is the interpolated costmap value. The large raw range necessitates a small weight.

---

## Goal-Reaching Critics

### 4. Goal Alignment Cost (GoalAlignCost)

Penalises squared angular deviation between the predicted heading and the bearing toward the final goal:

$$r_{\text{ga}}^{(t)} = w_{\text{ga}} \cdot \left(\mathrm{atan2}\!\big(\sin(\phi_{\text{goal}} - \theta_t),\, \cos(\phi_{\text{goal}} - \theta_t)\big)\right)^2$$

where $\phi_{\text{goal}}$ is the desired goal heading.

### 5. Goal Proximity Cost (GoalProximityCost)

Implements a logarithmic attractive potential field toward the global goal:

$$r_{\text{gp}}^{(t)} = w_{\text{gp}} \cdot \log\!\left(1 + \frac{d_t}{\varepsilon}\right)$$

where $d_t = \lVert \mathbf{p}_t - \mathbf{p}_{\text{goal}} \rVert$ and $\varepsilon$ (`decay_distance`) controls the transition from linear to logarithmic behaviour. This potential has the following properties:

- $U(0) = 0$: the global minimum lies exactly at the goal;
- $\nabla U \propto 1/(d + \varepsilon)$: the gradient always points toward the goal and is strongest near it;
- For $d \ll \varepsilon$, $U \approx d/\varepsilon$ (linear, strong pull);
- For $d \gg \varepsilon$, $U \approx \log(d/\varepsilon)$ (logarithmic saturation, preventing domination over path-tracking costs at large distances);
- Smooth everywhere with no activation barrier.

---

## Speed and Smoothness Critics

### 6. Velocity Cost (VelocityCost)

Penalises the squared deviation between the commanded linear velocity and the desired cruising speed $v_{\text{des}}$:

$$r_{\text{vel}}^{(t)} = w_{\text{vel}} \cdot \left(v_{\text{des}} - v_b\right)^2$$

where $b = \lfloor t / B \rfloor$ is the block index for timestep $t$.

### 7. Velocity Feasibility Cost (VelocityFeasibilityCost)

Enforces smooth transitions between adjacent velocity blocks by penalising squared jumps in both linear and angular velocity. This critic is instantiated **once per block boundary** ($N_b - 1$ times total, not per timestep):

$$r_{\text{feas}}^{(b)} = w_{\text{feas}} \cdot \left[\left(v_b - v_{b+1}\right)^2 + \left(\omega_b - \omega_{b+1}\right)^2\right]$$

---

## Social Navigation Critics

The following critics use the SFM-predicted agent states. Agent positions, orientations, and velocities are co-evolved through `computeSFMState`, which applies the extended SFM interaction forces to each agent at every prediction step.

### 8. Social Work Cost (SocialWorkCost)

Computes the magnitude of SFM social forces exerted on both the robot by the agents and on the agents by the robot, and penalises the total:

$$r_{\text{sw}}^{(t)} = w_{\text{sw}} \cdot \left(\left\lVert \mathbf{F}_{\text{social}}^{\text{robot}}\right\rVert^2 + \sum_{j=1}^{N_a} \left\lVert \mathbf{F}_{\text{social}}^{(j \leftarrow \text{robot})}\right\rVert\right)$$

The social force $\mathbf{F}_{\text{social}}$ follows the anisotropic formulation of Helbing and Molnár, with:

$$\mathbf{F}_{ij} = A_{\text{social}} \left(\mathbf{f}_v + \mathbf{f}_\theta\right)$$

where $\mathbf{f}_v$ is the velocity-aligned repulsive component and $\mathbf{f}_\theta$ is the angular (lateral) component, both decaying exponentially with inter-agent distance scaled by $B = \gamma \lVert \lambda\,\Delta\mathbf{v} + \hat{\mathbf{d}} \rVert$.

### 9. Proxemics Cost (ProxemicsCost)

Enforces a distance-based personal space penalty using a sum of exponential decay functions over all valid agents:

$$r_{\text{prox}}^{(t)} = w_{\text{prox}} \cdot \sum_{j=1}^{N_a} \alpha \cdot \exp\!\left(-\frac{\lVert \mathbf{p}_t - \mathbf{p}_j \rVert^2}{d_0^2}\right)$$

where $d_0$ is the characteristic proxemic distance and $\alpha$ is a scaling factor. The summation formulation ensures smooth gradient propagation through Ceres automatic differentiation (unlike a $\min$-based formulation which would break Jet-type gradient flow).

### 10. Agent Angle Cost (AgentAngleCost)

Penalises the robot for heading toward or travelling in the same direction as the nearest moving agent. It has two components, both modulated by exponential distance decay:

$$r_{\text{aa}}^{(t)} = w_{\text{aa}} \cdot e^{-d^2/d_s^2} \cdot \Big(\underbrace{\mathrm{softplus}\!\big(\cos(\theta_t - \phi_{j})\big)}_{\text{position alignment}} + w_v \cdot \underbrace{\mathrm{softplus}\!\big(\cos(\theta_t - \psi_{j})\big)}_{\text{velocity alignment}}\Big)$$

where $\phi_j = \mathrm{atan2}(y_j - y_t,\, x_j - x_t)$ is the bearing to the agent and $\psi_j$ is the agent's heading. The softplus function $\mathrm{softplus}(x) = \frac{1}{k}\ln(1 + e^{kx})$ with $k=5$ acts as a smooth ReLU, activating the penalty only when the robot points toward the agent ($\cos > 0$) or travels in the same direction.

### 11. Crossing Cost (CrossingCost)

Specifically addresses perpendicular crossing encounters with two components:

$$r_{\text{cross}}^{(t)} = w_{\text{cross}} \cdot e^{-d^2/d_s^2} \cdot \Big(\underbrace{v_b \cdot \sin^2(\theta_t - \psi_j)}_{\text{speed penalty}} + w_{\text{bear}} \cdot \underbrace{\mathrm{softplus}\!\big(\mathbf{c} \cdot \omega_b \cdot s\big) \cdot \sin^2(\theta_t - \psi_j)}_{\text{steering penalty}}\Big)$$

where $\sin^2(\Delta\theta)$ gates the cost to peak at 90° crossings and vanish for same/opposite headings. The scalar $\mathbf{c}$ is the 2D cross product of the robot-to-agent vector with the agent's heading direction, determining whether the agent approaches from the left or right. The softplus of $\mathbf{c} \cdot \omega$ penalises angular velocity in the wrong rotational direction (i.e., turning in front of rather than behind the crossing agent). Both components provide **direct gradients** on the velocity parameters $v_b$ and $\omega_b$, yielding faster convergence than position-chain-rule-based costs alone.
