# MA-HiWonder-MPC
Dieses Rep dient zur Entwicklung auf dem HiWonder Mentor PI der mit einem rasberry betrieben wird.

Der HiWonder Pi wurde mit folgender Anleitung aufgesetzt: https://github.com/Matzefritz/HiWonder_MentorPi

ros2 topic pub --once -w 2 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

ros2 run Trajektorienfolgeregelung circle_trajectory_p_regler
