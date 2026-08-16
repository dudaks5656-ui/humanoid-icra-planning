#!/usr/bin/env bash
set -eo pipefail
run_root=${1:?usage: run_paper_main_simulation_dataset_v1.sh RUN_ROOT}
source /opt/ros/humble/setup.bash
source /home/openarm/humanoid_sim_ws/install/setup.bash
source /home/openarm/humanoid_sim_ws/validation/paper_main_simulation_dataset_v1/build/install/humanoid_extraction_experiments/share/humanoid_extraction_experiments/local_setup.bash
set -u
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
printf '%s\n' "$$" > "${run_root}/runner.pid"
ros2 launch humanoid_extraction_experiments paper_main_simulation_dataset_v1.launch.py run_root:="${run_root}" >> "${run_root}/logs/runner.log" 2>&1 || true
python3 /home/openarm/humanoid_sim_ws/src/humanoid_extraction_experiments/scripts/finalize_paper_main_simulation_dataset_v1.py "${run_root}"
