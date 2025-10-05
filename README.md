# Turtlesim launch execution
## 1) Open first terminal write
```bash
ros2 run turtlesim turtlesim_node 
```
## 2) Open second terminal write
```bash
mkdir hi_ws
cd hi_ws
git clone git@github.com:RiteshSureshAshtkar/Turtlesim.git
colcon build
source install/setup.bash
ros2 run hi_ws auto
```
## 3)Open third terminal write
```bash
ros2 topic pub /turtle1/target turtlesim/msg/Pose '{x: 5.5, y: 8.0, theta: 0.0, linear_velocity: 0.0, angular_velocity: 0.0}' --once
```
## 4)Make sure to hit ctrl+C on the second terminal and kill the auto node !!!.DO NOT FORGET
## 5)Open fourth terminal write to run the server
```bash
cd hi_ws
source install/setup.bash
ros2 run hi_ws c_server
```
## 6)Open sixth terminal write to run the client
```bash
cd hi_ws
source install/setup.bash
ros2 run hi_ws c_client
```
## 7)After thre completion of the circle In the second terminal write
```bash
ros2 run hi_ws auto
```
## 8)Open third terminal write
```bash
ros2 topic pub /turtle1/target turtlesim/msg/Pose '{x: 8.0, y: 8.0, theta: 0.0, linear_velocity: 0.0, angular_velocity: 0.0}' --once
```
Hence exercise is completed successfully 
