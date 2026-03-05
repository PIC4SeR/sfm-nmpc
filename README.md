[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Nav2 Controller](https://img.shields.io/badge/Nav2-Controller-orange.svg)](https://navigation.ros.org/)
[![Ceres Solver](https://img.shields.io/badge/Ceres-Solver-9cf.svg)](http://ceres-solver.org/)
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache--2.0-green.svg)](package.xml)

<h1 align="center">SFM-NMPC: Social-Force-Aware Nav2 Controller</h1>

<p align="center">
  <img src="docs/images/sfm_nmpc_open_crowded_scenario.gif" alt="SFM-NMPC open crowded scenario" width="650" />
</p>

`sfm_nmpc` brings a nonlinear model predictive controller to Nav2 that explicitly reasons about human crowds via the Social Force Model (SFM). The plugin optimizes a block of linear/angular velocities with Ceres, fusing classical path-following critics with social-aware penalties so the robot can plan short-horizon motions that obey proxemic conventions while staying close to the global plan.

## Key Features

- **Ceres-based NMPC** with configurable control horizon and block discretization (`optimizer` section).
- **11 custom critics** covering path tracking, obstacle avoidance, social force minimization, proxemics, and velocity smoothness. Mathematical details live in [`docs/mpc_critics_methodology.md`](../../docs/mpc_critics_methodology.md).
- **Social Force Model integration** to co-predict pedestrian trajectories and penalize socially undesirable interactions.
- **Nav2 drop-in**: exported as `sfm_nmpc::MPCSFMMotionModel`, so it plugs into `controller_server` alongside other controllers.
- **Ready-to-use parameter presets** (`params/params.yaml`, `params/obst_only_parameters_in_benchmark.yaml`, `params/soc_work_obst_parameters_in_benchmark.yaml`).
- **Documentation assets** (Gazebo snapshots, report configs) shared with the larger HuNavSim workspace for reproducible benchmarks.

## Package Layout

| Path | Notes |
| --- | --- |
| `CMakeLists.txt` | Builds `sfm_nmpc` and `sfm_nmpc_critics` shared libraries, exporting the plugin description `sfm_nmpc.xml`. |
| `include/sfm_nmpc/` | Headers for critics, optimizer, trajectorizer, interfaces, and helper utilities. |
| `src/` | Implementations for each critic, the optimizer, interfaces, and the Nav2 plugin entry point. |
| `params/` | Controller parameter presets used in the paper benchmarks. |
| `critics.md` | High-level overview of every critic, weights, and example YAML usage. |
| `EXAMPLE.md` | Reference README formatting template (the file requested by the template task). |

## Installation

`sfm_nmpc` depends on ROS 2 Humble, Nav2, and the `lightsfm` library (installed by the workspace dev container). To build the package inside the workspace:

```bash
cd /workspaces/hunavsim_devcontainer
colcon build --packages-select sfm_nmpc
source install/setup.bash
```
Run `rosdep install --from-paths src --ignore-src -r -y` to pull any missing dependencies.

## Usage

1. **Include the plugin in Nav2 bringup** by referencing the exported class in your controller configuration:

   ```yaml
   controller_server:
     ros__parameters:
       use_sim_time: True
       controller_frequency: 20.0
       controller_plugins: ["FollowPath"]
       FollowPath:
         plugin: "sfm_nmpc::MPCSFMMotionModel"
         trajectorizer:
           desired_linear_vel: 0.6
           lookahead_dist: 1.0
           max_angular_vel: 1.4
           time_step: 0.05
           max_time: 1.5
         optimizer:
           control_horizon: 18
           parameter_block_length: 6
           linear_solver_type: "DENSE_SCHUR"
           weights:
             distance_weight: 20.0
             social_weight: 400.0
             proxemics_weight: 80.0
             obstacle_weight: 0.13
             velocity_feasibility_weight: 5.0
   ```

2. **Launch Nav2** with the above params. Example:

   ```bash
   ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True \
       params:=$PWD/src/sfm-nmpc/params/params.yaml
   ```

3. **Tune critics** using the YAML weights or by editing the defaults in `optimizer.hpp`. `critics.md` explains the influence of each critic and shows a `FollowPath` configuration snippet.

4. **Benchmark** the controller using the HuNavSim Gazebo harness in `src/gazebo_test/` (see the root workspace README for instructions). Metrics and plots can be generated with `src/social_evaluation_graphs`.

## Parameter Presets

- `params/params.yaml` – General navigation stack with the SFM critics enabled alongside standard path/velocity critics.
- `params/obst_only_parameters_in_benchmark.yaml` – Ablation that disables the social costs to highlight obstacle-only behavior.
- `params/soc_work_obst_parameters_in_benchmark.yaml` – Variant emphasizing social work penalties for crowd-heavy scenes.

Each preset configures both the trajectorizer (lookahead, time step) and optimizer (solver tolerances, weights). Use them as starting points for your robots or experiments.

## Development Notes

- **Build type**: defaults to `Release` and forces `-fPIC` to satisfy pluginlib.
- **Testing**: enable `BUILD_TESTING` and add gtests under `test/` to integrate with Nav2’s CI configuration. Currently the package relies on simulation benchmarks for validation.
- **Formatting**: adhere to Nav2 coding conventions (ament linters are listed in `package.xml`).

## Citation

If you build upon this controller, please cite the sfm-nmpc and any related publications describing SFM-NMPC :

```
@misc{sfm_nmpc_controller,
  title        = {SFM-NMPC: Social-Force-Aware Nav2 Controller},
  year         = {2026},
  howpublished = {\url{https://github.com/<org>/hunavsim_devcontainer/tree/master/src/sfm-nmpc}}
}
```

This package is distributed under Apache-2.0; see `package.xml` for the full license header. Individual dependencies retain their respective licenses.
