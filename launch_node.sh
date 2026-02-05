source install/setup.bash

ros2 run cubemars_v2_ros motor_node --ros-args \
  -p can_interface:=can0 \
  -p can_id:=1 \
  -p motor_type:=AK10-9 \
  -p control_hz:=50.0 \
  -p joint_name:=joint1 \
  -p auto_start:=true \
  -p reverse_polarity:=false
