#!/bin/bash

set -e

echo "===== SQLite 라이브러리 설치 ====="
sudo apt-get update
sudo apt-get install -y libsqlite3-dev sqlite3

echo "===== 소스 설정 ====="
source ~/.bashrc
source /opt/ros/humble/setup.bash
cd ~/liorf_ws2

echo "===== 빌드 시작 ====="
colcon build --symlink-install --packages-select liorf

echo "===== 빌드 완료 ====="
source install/setup.bash

echo "===== LIORF 실행 ====="
echo "데이터베이스 모드가 활성화된 LIORF를 실행합니다."
echo "메모리 키프레임 제한: 100개, 공간 쿼리 반경: 50m"
echo "Ctrl+C로 종료하세요."

# 데이터베이스 모드 파라미터 추가
ros2 launch liorf run_lio_sam_livox_mid360.launch.py --debug use_database_mode:=true database_path:=/root/liorf_maps/slam_map.db active_keyframes_window_size:=100 spatial_query_radius:=50.0 