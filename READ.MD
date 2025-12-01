# 0. 컨테이너 안
cd ~/forza_ws

# 1. 빌드
colcon build --packages-select simple_perception_cpp
source install/setup.bash

# 2. (다른 터미널) 시뮬레이터 실행
ros2 launch stack_master base_system_launch.xml \
  racecar_version:=OrinNano \
  map_dir:=small_hall \
  map_name:=small_hall_orig \
  sim:=true

# 3. 인지 노드 실행
ros2 run simple_perception_cpp detect_cpp_node   # 또는 simple_detector_node
