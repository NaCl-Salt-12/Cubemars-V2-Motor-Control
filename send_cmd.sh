source install/setup.bash 
# CMD: [pos, vel, kp, kd, torque_ff]
ros2 topic pub /joint1/mit_cmd std_msgs/msg/Float64MultiArray "{data: [1.0, 0.0, 15.0, 1.0, 0.0]}" 
