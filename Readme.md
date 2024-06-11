## ORBSLAM3 on WSL-Ubuntu Demonstration:
<p align="center">
  <img src="https://github.com/aliaxam153/ORBSLAM3-WSL/assets/146977640/d71efab3-655d-4c48-823a-de32ea22a349" width="800" height="400" alt="ORB-SLAM3 in action">
</p>


> ### Disclaimer:
>
> The code and materials provided in this repository are not owned by me. They are sourced from various external contributors, publicly available resources, or other repositories. All credit for the original
> work goes to the respective authors and contributors. I do not claim any ownership or rights over the code and materials within this repository.
> If you are the rightful owner of any content and wish to have it removed or properly attributed, please contact me, and I will address your concerns promptly.

> ### Original Sources:
> 
> ORB-SLAM3: https://github.com/UZ-SLAMLab/ORB_SLAM3/

## Video Guide:
If you have trouble understanding the script based process for setting up ORB_SLAM3 in your local device, then watch and follow the process shown in this video:

https://www.youtube.com/watch?v=1JnIU3kLH0c

## Pre-requisities: 
Following are the instruction to install ORB-SLAM3, I have referenced this [repo](https://github.com/UZ-SLAMLab/ORB_SLAM3/tree/master) for the installation in Ubuntu 20.04 along with further changes added by me.
### Install and setup WSL2 in Windows Machine:
Here are the steps to install a specific Ubuntu distribution on WSL2 from the Microsoft Store:
- **Enable WSL2:** Before you install any Linux distribution, make sure you have WSL2 enabled on your Windows machine. You can do this by following the instructions in the official Microsoft documentation ([WSL2](https://learn.microsoft.com/en-us/windows/wsl/install)).
- **Open Microsoft Store:** Go to the Microsoft Store on your Windows machine.
- **Search for Ubuntu:** In the search bar at the top right corner of the Microsoft Store, type "Ubuntu" and press Enter.
- **Select Ubuntu Distro:** You will see various versions of Ubuntu available. Choose the specific version you want to install. For example, if you want Ubuntu 20.04 LTS, select that version.
- **Install:** Click on the Ubuntu distribution you've selected, then click on the "Get" or "Install" button to begin the installation process.
- **Launch Ubuntu:** Once the installation is complete, you can launch the Ubuntu distribution from the Start menu or by typing "Ubuntu" in the Windows search bar and selecting it from the results.
- **Set up Ubuntu:** The first time you launch the Ubuntu distribution, it will take a few moments to set up. Follow the on-screen instructions to set up your username and password.
- **Update Packages (Recommended):** It's a good practice to update the package lists and upgrade installed packages using the following commands:
  ```
  sudo apt-get update && sudo apt-get upgrade
  ```
  After completion of ubuntu update, reboot it.
  ```
  sudo reboot
  ```
Tool for editing scripts:
```
sudo apt install gedit
```

### Bash Installation Guide for Dependencies:
> Go to this directory:
>```
>mkdir -p ~/dev && cd ~/dev
>```
>Download this script file: 
>```
>wget https://raw.githubusercontent.com/aliaxam153/ORB_SLAM3/main/build_prerequisities.sh
>```
>Make the script executable:
>```
>chmod +x build_prerequisities.sh
>```
>Run the script:
>```
>./build_prerequisities.sh
>```
> This script will automate the installation process for Pangolin, OpenCV 4.4.0, and
> ROS Noetic on an Ubuntu 20.04 system, including switching the GCC and G++ versions, installing
> necessary dependencies, and verifying the installations.


###  OR Manual Installation Guide for Dependencies:
> Alternative method is to install by manual running the commands on the ubuntu terminal. Follow
> the instruction below:
> ### a) Install C++11 Compiler:
> We require C++11 compiler to build some dependencies. So, refer to this link to install the compiler: 
[C++11_Compiler.md](https://github.com/aliaxam153/ORB_SLAM3/blob/main/C++11_Compiler.md)
> ### b) Install Pangolin
> For installation procedure of Pangolin refer to this link:
> [Pangolin.md](https://github.com/aliaxam153/ORB_SLAM3/blob/main/Pangolin.md)
> ### c) Install OpenCV 4.4.0
> You must switch from default g++ & gcc version 11 the version 9 using the following command.
> ```
> sudo update-alternatives --config g++
> ```
> This will prompt you to select which version of G++ you wish to be the default by typing the
> selection number.
> 
> Select g++-9 by pressing 2.
> 
> Similarly, we use the following command for GCC:
> ```
> sudo update-alternatives --config gcc
> ```
> 
> Select gcc-9 by pressing 2 in prompt.
> 
> For installation procedure of OpenCV refer to this link:
> [OpenCV.md](https://github.com/aliaxam153/ORB_SLAM3/blob/main/OpenCV.md)
> ### d) Installation of ROS Noetic
> Follow this link for installation: [ROS-Noetic-TwoLineInstall](https://wiki.ros.org/ROS/Installation/TwoLineInstall/)
> 
> After installing ROS melodic, we need to create a catkin workspace, which will be used later
> for ORB-SLAM3 integration with ROS.
> [Catkin Workspace](https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
> 
> Now that your environment is now setup.


## Install ORB-SLAM3
Now, we install ORB-SLAM3. Use the ORB_SLAM3 file given in the repository, it is a modded version which works with ROS Noetic
```
cd ~/dev/ORB_SLAM3
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

Now simply just run (if you encounter compiler, try to run the this shell script 2 or 3 more time. It works for me.)
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
## Test Run an example on ROS-ORBSLAM3
Running RGB_D Node
For an RGB-D input from topics /camera/rgb/image_raw and /camera/depth_registered/image_raw, run node ORB_SLAM3/RGBD. You will need to provide the vocabulary file and a settings file. See the RGB-D example above.
```rosrun ORB_SLAM3 RGBD PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE```

Running ROS example: Download a rosbag (e.g. V1_02_medium.bag) from the [EuRoC dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) and paste in the directory ```~/```.

Open 3 tabs on the terminal and run the following command at each tab for a Stereo-Inertial configuration:

- Terminal-1:
```
roscore
```
- Terminal-2:
```
rosbag play --pause ~/MH_01_easy.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw /imu0:=/imu
```
- Terminal-3:
```
cd ~/dev/ORB_SLAM3/
rosrun ORB_SLAM3 Stereo_Inertial ~/dev/ORB_SLAM3/Vocabulary/ORBvoc.txt ~/dev/ORB_SLAM3/Examples/Stereo-Inertial/EuRoC.yaml true
```

