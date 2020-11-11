## sd_sample_pkg

## 使い方

```shell
cd ~/catkin_ws/src
git clone https://github.com/sskitajima/sd_sample_pkg.git
cd ~/catkin_ws
catkin_make
```

## サンプルプログラム

### トピック通信のサンプルプログラム

- ```src/listener.cpp```
- ```src/talker.cpp```

```shell
# terminal 1
roscore

# terminal 2
rosrun sd_sample_pkg talker_node

# termianl 3
rosrun sd_sample_pkg listner_node
```

### レーザースキャナとカメラのサンプルプログラム

- ```src/SD_sample_laser.cpp```
- ```src/SD_sample_camera.cpp```

```shell
# terminal 1
roslaunch sd_sample_pkg robot_simulation.launch

# terminal 2
rosrun sd_sample_pkg SD_sample_laser

# terminal 3
rosrun sd_sample_pkg SD_sample_camera
```

## シミュレーション

- ロボットモデル: ```urdf/robot.urdf.xacro```
- サンプルのworld: ```worlds/sample.world```