#!/usr/bin/env bash
# 저장된 Pick/Extract 경계 탐색 결과를 STL/MoveIt 모델 위에 표시한다.
# 기존 validation CSV는 읽기만 하며 경계 탐색을 다시 실행하지 않는다.

set -eo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="$WORKSPACE_ROOT/install/setup.bash"

if [[ ! -f "$ROS_SETUP" || ! -f "$WORKSPACE_SETUP" ]]; then
  echo "오류: ROS 2 Humble 또는 humanoid_sim_ws가 준비되지 않았습니다." >&2
  exit 1
fi

source "$ROS_SETUP"
source "$WORKSPACE_SETUP"
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1

echo "저장된 작업반경 XY 단면과 MoveIt RViz를 실행합니다."
echo "주의: 이 표시는 z=1.25 m의 기존 샘플 단면이며 전체 3D 안전영역이 아닙니다."
echo "종료: Ctrl+C"

exec ros2 launch humanoid_extraction_experiments workspace_boundary.launch.py
