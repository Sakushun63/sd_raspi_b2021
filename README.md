## sd_sample_pkg

## 使い方

```shell
cd ~/catkin_ws/src
git clone https://github.com/sskitajima/sd_sample_pkg.git
cd ~/catkin_ws
catkin_make
```

## サンプルプログラム

- 各処理の詳細や内容についてはソースコードを見てほしい

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

### 遠隔操作のサンプルプログラム

- ```scripts/teleop.py```

```shell
# terminal 1
roscore

# terminal 2
rosrun sd_sample_pkg teleop.py
```

### 画像処理のサンプルプログラム

- ```src/metch_templete.cpp```

```shell
# teminal 1
roscore

# teminal 2
rosrun sd_samp@le_pkg matching_node
```



## シミュレーション

- ロボットモデル: ```urdf/robot.urdf.xacro```
- サンプルのworld: ```worlds/sample.world```