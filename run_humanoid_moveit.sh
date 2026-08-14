#!/usr/bin/env bash
# STL 시각화, 관절 범위, 충돌 검사와 MoveIt RViz를 한 번에 실행한다.
# 시뮬레이션 전용이며 ros2_control, 컨트롤러, 실제 하드웨어를 실행하지 않는다.

set -eo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="$WORKSPACE_ROOT/install/setup.bash"
OMPL_PROFILE="${1:-baseline}"

case "$OMPL_PROFILE" in
  baseline|fine)
    ;;
  *)
    echo "사용법: $0 [baseline|fine]" >&2
    exit 2
    ;;
esac

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "오류: ROS 2 Humble 설정 파일을 찾을 수 없습니다: $ROS_SETUP" >&2
  exit 1
fi

if [[ ! -f "$WORKSPACE_SETUP" ]]; then
  echo "오류: 작업공간이 빌드되지 않았습니다: $WORKSPACE_SETUP" >&2
  echo "먼저 다음 명령을 실행하세요:" >&2
  echo "  cd $WORKSPACE_ROOT && source $ROS_SETUP && colcon build --symlink-install" >&2
  exit 1
fi

source "$ROS_SETUP"
source "$WORKSPACE_SETUP"

# 실제 OpenArm DDS 그래프와 분리된 로컬 시뮬레이션 설정이다.
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1

echo "휴머노이드 MoveIt 시뮬레이션을 실행합니다."
echo "  작업공간: $WORKSPACE_ROOT"
echo "  OMPL 프로필: $OMPL_PROFILE"
echo "  종료: Ctrl+C"

exec ros2 launch humanoid_sim_moveit_config planning_only.launch.py \
  ompl_profile:="$OMPL_PROFILE"
