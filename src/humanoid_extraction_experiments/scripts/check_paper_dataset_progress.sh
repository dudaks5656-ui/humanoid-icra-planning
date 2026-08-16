#!/usr/bin/env bash
set -euo pipefail
run_root=${1:?usage: check_paper_dataset_progress.sh RUN_ROOT}
date --iso-8601=seconds
cat "${run_root}/progress.json"
cat "${run_root}/resume_state.yaml"
printf 'completed_task_rows='
awk 'END {print (NR > 0 ? NR - 1 : 0)}' "${run_root}/logs/task_status.csv"
printf 'raw_case_rows='
awk 'END {print (NR > 0 ? NR - 1 : 0)}' "${run_root}/all_case_results.csv"
printf 'recorded_runner_pid='
grep -o 'runner_started pid=[0-9]*' "${run_root}/heartbeat.log" | tail -1 | cut -d= -f2 || true
tail -n 10 "${run_root}/heartbeat.log"
sha256sum --check "${run_root}/protected_before.sha256" >/dev/null
echo 'protected_sha256=OK'
