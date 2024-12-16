# File player for HeLiPR dataset in ''ROS2''

This program is a file player for the [HeLiPR dataset](https://sites.google.com/view/heliprdataset) in ROS2.

## 0. Prerequisite

Novatel GPS Driver Installation:
```
$ sudo apt-get install ros-VERSION-novatel-gps-driver
```

Livox ROS Driver2 Installation:

Follow the official reopository of [Livox-ros-driver2](https://github.com/Livox-SDK/livox_ros_driver2)

## 1. How to install, Build, and Run
```
$ mkdir -p HeLiPR_File_Player_ROS2_ws/src
$ cd HeLiPR_File_Player_ROS2_ws/src
$ git clone -b [ros2] https://github.com/wooseongY/HeLiPR-File-Player-ROS2.git
$ cd .. && colcon build
$ source install/setup.bash
$ ros2 launch helipr_file_player helipr_file_player.launch
```
- This version is tested in ROS-humble (Ubuntu 22.04)


## 2. Prepare the data and timestamps

If your data directory is represented as follows, you are now ready to enjoy the HeliPR dataset!
```
📂 Sequence_name/
├── 📂 LiDAR/
│   ├── 📂 Aeva/
│   │   └── 📝 timestamp.bin
│   ├── 📂 Avia/
│   │   └── 📝 timestamp.bin
│   ├── 📂 Ouster/
│   │   └── 📝 timestamp.bin
│   ├── 📂 Velodyne/
│   │   └── 📝 timestamp.bin
│   ├── 📂 Inertial_data/
│   │   └── 📝 inspva.csv
│   │   └── 📝 xsens_imu.csv
└── 📝 stamp.csv
```

## 3. Load data files and play

1. Click the "Load" button.
2. Choose Sequence_name folder including sensor_data folder and data_stamp.csv.
3. The "Play" button starts publishing data in the ROS message.
4. The "Pause/Resume" button pauses and resumes publishing data.
5. The "Save" button saves all topics into the rosbag file.
6. The "Loop" checkbox resumes when playback is finished.

Enjoy it:) 

## Maintainer

Wooseong Yang (yellowish@snu.ac.kr)
Minwoo Jung (moonshot@snu.ac.kr)
