# Critics Overview

This document describes the custom critics used in the `sfm_nmpc` package. Critics are cost functions that guide the robot's behavior during navigation, ensuring safety, efficiency, and social compliance.

---
## Optimization variables

The variables to optimize are the angular and linear speed of the robot in the subsequent time steps.
This is because the future position of the robot is optimized by inserting the subsequent speeds to project the state of the robot 
using the differential drive model.

---

## General passed parameters

**Weights:**
Each critic possesses its weight to determine its influence on the overall robot behavior.

**Robot params:**
The initial position and orientation of the robot is passed for the critics that require forward projection of the state.

**Path params:**
Some critics feature parameters containing the trajectorized path points, either positions or headings.

**Social params:**
The two social critics feature the information about the predicted agents positions and velocities.

---

## Agent Angle Cost Function

**Purpose:**  
Enforces an angular speed for the robot in certain situations.  
**Behavior:**  
If an agent is on the left side of the robot, the critic encourages steering to the right to follow social norms.

---

## Distance Cost Function

**Purpose:**  
Pushes the robot to follow points on the path.

**Usage:**  
- **Path Align:** If all trajectory points for different time steps are provided, aligns the robot to the path.
- **Path Follow:** If only the final point is given, acts as a path-follow critic.

---

## Goal Align Cost Function

**Purpose:**  
Reduces the angular difference between the robot's orientation and the goal pose orientation.

---

## Goal Proximity Cost Function

**Purpose:**  
Adds an exponential attraction toward the final global navigation goal once the robot is within a configurable activation radius.

**Behavior:**  
When the predicted future pose lies inside the activation radius, the residual grows with `exp(distance / decay)` so the optimizer prefers solutions that drive the robot directly into the goal region, even if the trajectorized path has already flattened out.

---

## Obstacle Cost Function

**Purpose:**  
Keeps the robot away from high-cost zones.

**Details:**  
Uses a bicubic interpolator to estimate the cost of a robot's position in future time steps using the local costmap.

---

## Social Work Cost Function

**Purpose:**  
Uses the Social Force Model (SFM) to consider social work as a cost, aiming to minimize the social impact of the robot.

---

## Proxemics Cost Function

**Purpose:**
Consideres the distance with neighboring agents as a cost, aiming to maximize it.

## Velocity Cost Function

**Purpose:**  
Keeps the robot's velocity terms near the desired values.

---

## Velocity Feasibility Cost Function

**Purpose:**  
Prevents the optimizer from computing drastically different velocity terms in subsequent time steps.

---
## Example Configuration: `FollowPath`

Below is an example YAML configuration for the `FollowPath` behavior using the `sfm_nmpc::MPCSFMMotionModel` plugin:

```yaml
FollowPath:

    plugin: sfm_nmpc::MPCSFMMotionModel
    transform_tolerance: 0.5
    max_robot_pose_search_dist: 6.0
    max_linear_vel: 0.6
    min_linear_vel: -0.2
    max_angular_vel: 1.4
    trajectorizer:
      waypoint_dist_tol: 0.05
      desired_linear_vel: 0.6
      omnidirectional: false
      lookahead_dist: 2.0
      allow_reverse: false
      reverse_heading_threshold: 2.56
      max_reverse_speed: 0.2
      base_frame: "base_link"
      time_step: 0.10
      max_time: 2.0
      motion_model_type: "unicycle"
      max_angular_accel: 3.0
      max_linear_accel: 2.5
      min_approach_linear_velocity: 0.05
      
      rotate_to_heading_angular_vel: 0.4
      rotate_to_heading_min_angle: 1.57
      use_interpolation: true
      use_rotate_to_heading: true
      # Obstacle avoidance (regulated pure pursuit)
      use_cost_regulated_linear_velocity_scaling: true
      cost_scaling_dist: 0.6
      cost_scaling_gain: 1.0
      inflation_cost_scaling_factor: 3.0
      use_collision_detection: false
      max_allowed_time_to_collision_up_to_carrot: 1.0
      projection_lookahead_resolution: 0.1
      
    optimizer:
      linear_solver_type: "DENSE_SCHUR"
      param_tol: 1.0e-9
      fn_tol: 1.0e-5
      gradient_tol: 1.0e-8
      max_iterations: 40
      control_horizon: 18
      parameter_block_length: 6
      discretization: 1
      debug_optimizer: false
      current_path_weight: 0.5
      current_cmds_weight: 0.5
      goal_proximity_activation_radius: 0.75
      goal_proximity_decay_distance: 0.25
      agent_velocity_bound: 0.6
      stationary_agent_velocity_bound: 0.01
      max_agents: 3
      critics:
        enable_social_work: false
        enable_path_align: false
        enable_path_follow: false
        enable_proxemics: false
        enable_angle: false
        enable_crossing: false
      weights:
        # Navigation (path following)
        distance_weight: 9.0          # Path tracking — squared dist, moderate raw value
        angle_weight: 1.5             # 
        goal_align_weight: 8.0        # Heading 
        goal_proximity_weight: 5.0   # Log 
        velocity_weight: 4.5          # Desired 
        velocity_feasibility_weight: 8.0  # 
        # Safety
        obstacle_weight: 0.1          # Costmap 
        social_weight: 40.0           # SFM force 
        proxemics_weight: 8.0         # Proximity 
        agent_angle_weight: 5.0      # Don't point 
        velocity_alignment_weight: 8.0  # Don't 
        crossing_weight: 20.0         # Speed 
        crossing_bearing_weight: 5.0   # 
```

This configuration sets parameters for trajectory generation and optimization, including weights for each cost function described above.

---


## License

See the repository for license information.
