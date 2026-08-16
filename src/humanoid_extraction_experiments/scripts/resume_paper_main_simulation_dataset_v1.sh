#!/usr/bin/env bash
set -euo pipefail
run_root=${1:?usage: resume_paper_main_simulation_dataset_v1.sh RUN_ROOT}
if test -f "${run_root}/runner.pid" && kill -0 "$(<"${run_root}/runner.pid")" 2>/dev/null; then
  echo "runner already active" >&2; exit 2
fi
exec /home/openarm/humanoid_sim_ws/src/humanoid_extraction_experiments/scripts/run_paper_main_simulation_dataset_v1.sh "${run_root}"
