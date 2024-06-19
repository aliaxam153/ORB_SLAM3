### a) Install C++11 Compiler:
We require C++11 compiler to build some dependencies. So, refer to this link to install the compiler: 
[C++11_Compiler.md](https://github.com/aliaxam153/ORB_SLAM3/blob/main/C++11_Compiler.md)
### b) Install Pangolin
For installation procedure of Pangolin refer to this link:
[Pangolin.md](https://github.com/aliaxam153/ORB_SLAM3/blob/main/Pangolin.md)
### c) Install OpenCV 4.4.0
You must switch from default g++ & gcc version 11 the version 9 using the following command.
```
sudo update-alternatives --config g++
```
This will prompt you to select which version of G++ you wish to be the default by typing the
selection number.

Select g++-9 by pressing 2.
 
Similarly, we use the following command for GCC:
```
sudo update-alternatives --config gcc
```
 
Select gcc-9 by pressing 2 in prompt.

For installation procedure of OpenCV refer to this link:
[OpenCV.md](https://github.com/aliaxam153/ORB_SLAM3/blob/main/OpenCV.md)
### d) Installation of ROS Noetic
Follow this link for installation: [ROS-Noetic-TwoLineInstall](https://wiki.ros.org/ROS/Installation/TwoLineInstall/)

After installing ROS melodic, we need to create a catkin workspace, which will be used later
for ORB-SLAM3 integration with ROS.
[Catkin Workspace](https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

### e) Setup ORB_SLAM3 Environment
Now that your environment is now setup. Download the modded version ORB_SLAM3 from this repo.
```
cd ~/dev
git clone https://github.com/aliaxam153/ORB_SLAM3.git
```

Before installation change compiler back to default C++11 compiler.
You must switch from default g++ & gcc version 11 the version 9 using the following command.
```
sudo update-alternatives --config g++
```
This will prompt you to select which version of G++ you wish to be the default by typing the selection number. 
Select g++-9 by pressing 1.
 
Similarly, we use the following command for GCC:
```
sudo update-alternatives --config gcc
```
Select gcc-9 by pressing 1 in prompt.

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
