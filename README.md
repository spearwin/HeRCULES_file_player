# File Player for HeRCULES Dataset

 <div align="center">
    
  ![overview](https://github.com/user-attachments/assets/c3b71b0e-3a5f-4c9c-91e3-bc6c23870f03)

 </div>

## News
- 2025/02/05: Our dataset is available via [https://sites.google.com/view/herculesdataset](https://sites.google.com/view/herculesdataset).
- 2025/02/05 : ROS based fileplayer of the HeRCULES dataset is released.
- 2025/03/08 : PR_GT has been updated with a unified local coordinate system.
- 2025/03/10 : Stereo Camera Data is uploaded.
- 2025/05/30 : HeRCULES_Pointcloud_Toolbox is released [https://github.com/hanjun815/HeRCULES_Pointcloud_Toolbox](https://github.com/hanjun815/HeRCULES_Pointcloud_Toolbox).
- 2025/07/11 : ROS2 fileplayer of HeRCULES is released.
  
## What is File player?
This program is a file player for the complex urban data set. If a user installs the ROS using "Desktop-Full version", there is only one additional dependent package, except for the ROS default package. First, clone this package into the src folder of your desired ROS workspace.

## 1. Pre-requisites
Before utilizing the file player, it's crucial to have both the novatel-gps-msgs and livox custom messages. Ensure you install these drivers:

Novatel GPS Driver Installation:
Replace 'version' with your appropriate ROS version (e.g., melodic, noetic).
```
$ sudo apt-get install ros-'version'-novatel-gps-driver
```

## 2. How to install
```
$ mkdir -p catkin_ws/src
$ cd catkin_ws/src
$ git clone https://github.com/hanjun815/HeRCULES_file_player.git
$ cd ~/catkin_ws
$ catkin_make
```

## 3. How to Execute the File Player

```
$ source devel/setup.bash
$ roslaunch file_player file_player.launch
```
- Then, you need to select a sequence directory via GUI.
- For the correct load, first you need to place the GPS, IMU, LiDAR, and radar files in a directory following this structure (see this [guide video](https://youtu.be/uU-FC-GmHXA?t=45)) 
- IMU and GPS files (.csv) must be located at the same directory of "datastamp.csv"

## 4. Load Data Files and Play
Here's a step-by-step guide:

1. Click the 'Load' button.
2. Navigate and select the desired dataset folder.
3. Hit the player button to commence publishing data as ROS messages.
4. Use the 'Stop skip' button to skip intervals when the vehicle remains stationary. This feature enhances the user experience by focusing on significant data.
5. The loop button ensures that the data resumes playback from the beginning once completed

```
awk '{print $1",pose_gt"}' PR_GT/Continental_gt.txt > pose_gt_stamp.csv
cat sensor_data/datastamp.csv pose_gt_stamp.csv | sort > sensor_data/merged_datastamp.csv
mv sensor_data/merged_datastamp.csv sensor_data/datastamp.csv
```

## License and Citation
- When using the dataset or code, please cite our paper:
```
@INPROCEEDINGS { hjkim-2025-icra,
    AUTHOR = { Hanjun Kim and Minwoo Jung and Chiyun Noh and Sangwoo Jung and Hyunho Song and Wooseong Yang and Hyesu Jang and Ayoung Kim },
    TITLE = { HeRCULES: Heterogeneous Radar Dataset in Complex Urban Environment for Multi-session Radar SLAM },
    BOOKTITLE = { Proceedings of the IEEE International Conference on Robotics and Automation (ICRA) },
    YEAR = { 2025 },
    MONTH = { May. },
    ADDRESS = { Atlanta },
}
```

## Radar Development Tools
- Our Radar development tools are available via [Polar2X](https://github.com/hanjun815/Polar2X).

## Contributors
- Maintainer: Hanjun Kim (hanjun815@snu.ac.kr)
- Jinyong Jeong: The original author
- Minwoo Jung: made the player system compatible with LIO-SAM input (i.e., supports ring information of a lidar scan)


