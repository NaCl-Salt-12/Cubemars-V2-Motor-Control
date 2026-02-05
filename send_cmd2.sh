source install/setup.bash 
# CMD: [pos, vel, kp, kd, torque_ff]
ros2 topic pub --rate 820 /joint1/mit_cmd std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 12.0, 1.0, 0.0]}" 

