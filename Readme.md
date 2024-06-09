![image](https://github.com/aliaxam153/ORB-SLAM3-MINOS-Installation-Integration-in-WSL/assets/146977640/6d014486-c34d-4673-9385-fbfff2547797)

## Installation of ORB-SLAM3:
Following are the instruction to install ORB-SLAM3, I have referenced this [repo](https://github.com/UZ-SLAMLab/ORB_SLAM3/tree/master) for the installation in Ubuntu 20.04 along with further changes added by me.
### Install Pangolin
> For installation procedure of Pangolin refer to this link: 
[Pangolin.md](https://github.com/aliaxam153/ORB-SLAM3-MINOS-Installation-Integration-in-WSL/blob/main/Pangolin.md)

### Install OpenCV 4.4.0
You must switch from default g++ & gcc version 11 the version 9 using the following command.
```sudo update-alternatives --config g++```
This will prompt you to select which version of G++ you wish to be the default by typing the selection number. Select g++-9 by pressing 2.

Similarly, we use the following command for GCC:
```sudo update-alternatives --config gcc```
Select gcc-9 by pressing 2 in prompt.
> For installation procedure of OpenCV refer to this link: 
[OpenCV.md](https://github.com/aliaxam153/ORB-SLAM3-MINOS-Installation-Integration-in-WSL/blob/main/OpenCV.md)

### Installation of ROS Noetic
Follow this link for installation: [ROS-Noetic-TwoLineInstall](https://wiki.ros.org/ROS/Installation/TwoLineInstall/)
After installing ROS melodic, we need to create a catkin workspace, which will be used later for ORB-SLAM3 integration with ROS.
[Catkin Workspace](https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
Now that your environment is now setup.

## Install ORB-SLAM3
Now, we install ORB-SLAM3. Use the ORB_SLAM3 file given in the repository, it is a modded version which works with ROS Noetic
```
cd ~/dev
cd ORB_SLAM3
```
Before installation change compiler back to default C++11 compiler.
Now Simply just run (if you encounter compiler, try to run the this shell script 2 or 3 more time. It works for me.)
```
chmod +x build.sh
./build.sh
```
### Integrate with  ORB_SLAM3 ROS
Add the path including Examples/ROS/ORB_SLAM3 to the ROS_PACKAGE_PATH environment variable. Open .bashrc file:
```
gedit ~/.bashrc
```
and add at the end the following line. Replace PATH by the folder where you cloned ORB_SLAM3:
```
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM3/Examples/ROS
```
e.g. ```export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/user/dev/ORB_SLAM3/Examples/ROS```
After editing, save the file and run: ```source ~/.bashrc```
Execute build_ros.sh script:
```
chmod +x build_ros.sh
./build_ros.sh
```
## Run Example on ROS
Running RGB_D Node
For an RGB-D input from topics /camera/rgb/image_raw and /camera/depth_registered/image_raw, run node ORB_SLAM3/RGBD. You will need to provide the vocabulary file and a settings file. See the RGB-D example above.
```rosrun ORB_SLAM3 RGBD PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE```

Running ROS example: Download a rosbag (e.g. V1_02_medium.bag) from the [EuRoC dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) and paste in the directory ```~/```.

Open 3 tabs on the terminal and run the following command at each tab for a Stereo-Inertial configuration:

- Terminal_1: ```roscore```
- Terminal_2: ``` rosbag play --pause ~/MH_01_easy.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw /imu0:=/imu```
- Terminal_3:
```
cd ~/Packages/ORB_SLAM3/
rosrun ORB_SLAM3 Stereo_Inertial ~/dev/ORB_SLAM3/Vocabulary/ORBvoc.txt ~/dev/ORB_SLAM3/Examples/Stereo-Inertial/EuRoC.yaml true
```

