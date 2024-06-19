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

## Comments: 
Here are the instructions to install ORB-SLAM3. I referred to the original repository  in the "Original Source" section. However, since the original repository has become deprecated, there were significant conflicts with package versions and dependencies. Therefore, I have made modifications and re-uploaded the modified version of "ORB_SLAM3" in this repository.

## Installation Procedure:
I have described two methods to deal with ORB_SLAM3 dependencies:

 1) Bash-script Installation Guide.
 2) Manual Script-by-Script Installation Guide.

You can follow either one.

### 1) Bash-script Installation Guide:

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

### 2) Manual Script-by-Script Installation Guide:
> Alternative method is to install by manual running the commands on the ubuntu terminal. Refer to this guide: [manual_install_orbslam3.md](https://github.com/aliaxam153/ORB_SLAM3/blob/main/manual_install_orbslam3.md)

## Build & Install ORB-SLAM3
Now, we install ORB-SLAM3. Use the ORB_SLAM3 file given in the repository, it is a modded version which works with ROS Noetic
```
cd ~/dev/ORB_SLAM3
```
Now simply just run (if you encounter compiler, try to run the this shell script 2 or 3 more time. It works for me.)
```
chmod +x build.sh
./build.sh
```
### Integrate with  ORB_SLAM3 ROS

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

