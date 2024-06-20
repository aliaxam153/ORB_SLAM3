#!/bin/bash


echo "#######################################################################################################################"
echo ""
echo ">>> {Starting ORBSLAM3 Installation}"
echo ""
echo ">>> {Checking your Ubuntu version} "
echo ""
#Getting version and release number of Ubuntu
version=`lsb_release -sc`
relesenum=`grep DISTRIB_DESCRIPTION /etc/*-release | awk -F 'Ubuntu ' '{print $2}' | awk -F ' LTS' '{print $1}'`
echo ">>> {Your Ubuntu version is: [Ubuntu $version $relesenum]}"
#Checking version is focal, if yes proceed othervice quit
case $version in
  "focal" )
  ;;
  *)
    echo ">>> {ERROR: This script will only work on Ubuntu Focal (20.04).}"
    exit 0
esac

echo ""
echo ">>> {Ubuntu Focal 20.04 is fully compatible with Ubuntu Focal 20.04}"
echo ""
echo "#######################################################################################################################"
echo ">>> {Step 1: Handling Required Dependencies to build ORBSLAM3 and its packages}"
echo ""
# Function to show a loading animation
show_loading_animation() {
     local pid=$1
     local delay=0.1
     local spinstr='|/-\'
     while ps -p $pid > /dev/null 2>&1; do
         local temp=${spinstr#?}
         printf " [%c]  " "$spinstr"
         spinstr=$temp${spinstr%"$temp"}
         sleep $delay
         printf "\b\b\b\b\b\b"
     done
     printf "    \b\b\b\b"
 }

echo ">> Installing build-essential package..."
(sudo apt-get install -y build-essential gedit pv dialog cmake > /dev/null 2>&1 ) &
pid=$!
show_loading_animation $pid
wait $pid
if [ $? -eq 0 ]; then
    echo ">> build-essential packages are installed."
else
    echo ">> Failed to install build-essential packages."
    exit 1
fi

echo "#######################################################################################################################"
echo ">>> {Step 2: Install and Setup C++11 Compiler}"
echo ""
echo ">> Adding PPA repository to get the latest G++ version for Current Distro..."
sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y && sudo apt update
echo ">> PPA repository added."
echo ""

echo ">> Installing g++-11..."
sudo apt install -y g++-11
if [ $? -eq 0 ]; then
    echo ">> g++-11 compiler are installed."
else
    echo ">> Failed to install g++-11 compiler."
    exit 1
fi
echo ">> g++-11 installation completed."
echo ""

echo ">> Setup default G++ to alternatives system..."
echo ""
echo "> Adding G++-9 to alternatives..."
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 20
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 20
echo ""
# Add other versions to alternatives
echo "> Adding G++-11 to alternatives..."
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 50
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 50
echo ""


# Switch to G++ and GCC version 9
echo ">> Switching from Default G++/GCC V11 --> V9..."
sudo update-alternatives --config g++ <<EOF
2
EOF
sudo update-alternatives --config gcc <<EOF
2
EOF
echo ""
echo ">> Verifying G++ & GCC Version..."
g++ --version && gcc --version
echo ""

echo ">> Adding Misc Repositories..."
sudo add-apt-repository universe
sudo add-apt-repository restricted
sudo add-apt-repository multiverse
echo ""

echo ">> Refresh all installed packages..."
sudo apt update
echo ""

echo "#######################################################################################################################"
echo ">>> {Step 3: Build and Install All Pre-requisite Packages}" 
echo ""
echo ">> Created dev folder to store all builds"
mkdir -p ~/dev && cd ~/dev

echo "#######################################################################################################################"
echo ">> Pangolin Installation"
echo ""

# Check if Pangolin is already installed
if [ -d "/usr/local/include/pangolin" ]; then
    echo "> Pangolin is already installed."
else
    echo "> Pangolin is not installed. Proceeding with installation."
    echo "> Cloning Pangolin..."
    cd ~/dev
    # Check if the repository already exists
    if [ -d "Pangolin" ]; then
        echo "Repository already exists."

    else
        # Clone the repository
        REPO_URL="https://github.com/stevenlovegrove/Pangolin.git"
        git clone $REPO_URL
        if [ $? -eq 0 ]; then
            echo "> Cloning Completed"  
        else
            echo "> Failed to clone the repository."
            exit 1
        fi
    fi

    # Verify the repository using git fsck
    echo "Verifying the repository..."
    cd ~/dev/Pangolin
    ( git fsck > /dev/null 2>&1 ) &
    pid=$!
    show_loading_animation $pid
    wait $pid

    if [ $? -eq 0 ]; then
        echo "Repository verification completed successfully."
    else
        echo "Repository verification failed."
        # Clone the repository
        echo "Re-cloning Pangolin..."
        git clone $REPO_URL
        if [ $? -eq 0 ]; then
            echo "Repository cloned successfully."
        else
            echo "Failed to clone the repository."
            exit 1
        fi
    fi

    echo "> Running dry-run for Pangolin prerequisites..."
    cd ~/dev/Pangolin
    (./scripts/install_prerequisites.sh --dry-run recommended > /dev/null 2>&1 ) &
    pid=$!
    show_loading_animation $pid
    wait $pid
    echo ""
    echo "> Fixing catch2 check from install_prerequisites.sh..."
    sed -i '/catch2/d' ./scripts/install_prerequisites.sh
    echo ""
    
    # Installation of Catch2

    cd ~/dev
    # Check if the repository already exists
    if [ -d "Catch2" ]; then
        echo "Repository already exists."

    else
        # Clone the repository
        REPO_URL="https://github.com/catchorg/Catch2.git"
        git clone $REPO_URL
        if [ $? -eq 0 ]; then
            echo "> Cloning Completed"  
        else
            echo "> Failed to clone the repository."
            exit 1
        fi
    fi

    # Verify the repository using git fsck
    echo "Verifying the repository..."
    cd Catch2
    ( git fsck > /dev/null 2>&1 ) &
    pid=$!
    show_loading_animation $pid
    wait $pid

    if [ $? -eq 0 ]; then
        echo "Repository verification completed successfully."
    else
        echo "Repository verification failed."
        REPO_URL="https://github.com/catchorg/Catch2.git"
        git clone $REPO_URL
        if [ $? -eq 0 ]; then
            echo "> Cloning Completed"
        else
            echo "> Failed to clone the Catch2 repository."
            exit 1
        fi
    fi

    echo ""
    echo "> Building Catch2..."
    cd ~/dev/Catch2
    ( cmake -Bbuild -H. -DBUILD_TESTING=OFF > /dev/null 2>&1 ) &
    pid=$!
    show_loading_animation $pid
    wait $pid
    if [ $? -eq 0 ]; then
        echo "> Catch2 build configuration completed."
    else
        echo "> Failed to configure Catch2 build."
        exit 1
    fi

    echo ""
    echo "> Installing Catch2..."
    ( sudo cmake --build build/ --target install > /dev/null 2>&1 ) &
    pid=$!
    show_loading_animation $pid
    wait $pid
    if [ $? -eq 0 ]; then
        echo "> Catch2 installation completed."
    else
        echo "> Failed to install Catch2."
        exit 1
    fi
    
    # Install dependencies for Pangolin
    echo "> Installing Pangolin dependencies..."
    cd ~/dev/Pangolin
    sudo ./scripts/install_prerequisites.sh recommended
    if [ $? -eq 0 ]; then
        echo "> Dependencies installation completed."
    else
        echo "> Failed to install Pangolin dependencies."
        exit 1
    fi

    # Build & Install Pangolin
    echo "> Building and installing Pangolin..."
    mkdir -p build && cd build
    
    # Run CMake with loading animation
    ( cmake .. -DCMAKE_BUILD_TYPE=Release > /dev/null 2>&1 ) &
    pid=$!
    show_loading_animation $pid
    wait $pid
    if [ $? -eq 0 ]; then
        echo "> CMake configuration completed."
    else
        echo "> CMake configuration failed."
        exit 1
    fi
    
    # Run make with loading animation
    ( make -j8 > /dev/null 2>&1 ) &
    pid=$!
    show_loading_animation $pid
    wait $pid
    if [ $? -eq 0 ]; then
        echo "> Make completed."
    else
        echo "> Make failed."
        exit 1
    fi

    # Run make install with loading animation
    (sudo make install > /dev/null 2>&1 ) &
    pid=$!
    show_loading_animation $pid
    wait $pid
    if [ $? -eq 0 ]; then
        echo "> Pangolin installation completed successfully."
    else
        echo "> Pangolin installation failed."
        exit 1
    fi
fi
    
# Updating package list
echo "> Updating package list..."
(sudo apt update > /dev/null 2>&1 ) &
pid=$!
show_loading_animation $pid
wait $pid
if [ $? -eq 0 ]; then
    echo "> Package list updated."
else
    echo "> Failed to update package list."
    exit 1
fi


echo "#######################################################################################################################"
echo ">> OpenCV Installation"
echo ""

# Check if OpenCV is already installed
if pkg-config --modversion opencv4 > /dev/null 2>&1; then
    echo "> OpenCV is already installed."
else
    echo "> OpenCV is not installed. Proceeding with build and installation."

    # Ensure the dev directory exists
    mkdir -p ~/dev
    cd ~/dev

    # Check if the repository already exists
    if [ -d "opencv" ]; then
        echo "Repository already exists."
    else
        # Clone the repository
        REPO_URL="https://github.com/opencv/opencv.git"
        git clone $REPO_URL
        if [ $? -eq 0 ]; then
            echo "> Cloning Completed"
        else
            echo "> Failed to clone the repository."
            exit 1
        fi
    fi

    # Verify the repository using git fsck
    echo "Verifying the repository..."
    cd ~/dev/opencv
    ( git fsck > /dev/null 2>&1 ) &
    pid=$!
    show_loading_animation $pid
    wait $pid

    if [ $? -eq 0 ]; then
        echo "Repository verification completed successfully."
    else
        echo "Repository verification failed."
        # Clone the repository again
        echo "Re-cloning OpenCV..."
        rm -rf ~/dev/opencv
        git clone $REPO_URL
        if [ $? -eq 0 ]; then
            echo "Repository cloned successfully."
        else
            echo "Failed to clone the repository."
            exit 1
        fi
    fi

    # Checkout the specified version
    cd ~/dev/opencv
    git checkout 4.4.0
    if [ $? -eq 0 ]; then
        echo "> Checked out OpenCV version 4.4.0"
    else
        echo "> Failed to checkout OpenCV version 4.4.0"
        exit 1
    fi

    # Install necessary dependencies
    echo "> Installing dependencies for OpenCV..."
    ( sudo apt install -y build-essential cmake git pkg-config libgtk-3-dev \
        libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
        libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
        gfortran openexr libatlas-base-dev python3-dev python3-numpy \
        libtbb2 libtbb-dev libdc1394-22-dev libopenexr-dev \
        libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev > /dev/null 2>&1 ) &
    pid=$!
    show_loading_animation $pid
    wait $pid
    if [ $? -eq 0 ]; then
        echo "> Dependencies installed successfully"
    else
        echo "> Failed to install dependencies"
        exit 1
    fi

    # Create a build directory and switch into it
    echo "> Creating build directory for OpenCV..."
    mkdir -p ~/dev/opencv/build && cd ~/dev/opencv/build

    # Configure the build with CMake
    echo "> Configuring OpenCV build with CMake..."
    ( cmake -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/usr/local .. > /dev/null 2>&1 ) &
    pid=$!
    show_loading_animation $pid
    wait $pid
    if [ $? -eq 0 ]; then
        echo "> CMake configuration completed successfully"
    else
        echo "> CMake configuration failed"
        exit 1
    fi

    # Build and install OpenCV
    echo "> Building and installing OpenCV..."
    ( make -j8 > /dev/null 2>&1 ) &
    pid=$!
    show_loading_animation $pid
    wait $pid
    if [ $? -eq 0 ]; then
        echo "> OpenCV built successfully"
    else
        echo "> OpenCV build failed"
        exit 1
    fi

    ( sudo make install > /dev/null 2>&1 ) &
    pid=$!
    show_loading_animation $pid
    wait $pid
    if [ $? -eq 0 ]; then
        echo "OpenCV installation completed successfully."
    else
        echo "OpenCV installation failed."
        exit 1
    fi
fi
echo "#######################################################################################################################"
echo ">>ROS Noetic Installation" 
echo ""
# Check if ROS Noetic is already installed
if dpkg-query -W ros-noetic-desktop-full > /dev/null 2>&1; then
    echo "> ROS Noetic is already installed."
else
    echo "> ROS Noetic is not installed. Proceeding with installation."
    # Install ROS Noetic
    echo "Downloading and installing ROS Noetic..."
    wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh
    chmod +x ./ros_install_noetic.sh
    ./ros_install_noetic.sh
fi

source /opt/ros/noetic/setup.bash
source ~/.bashrc

# echo "#######################################################################################################################"
# echo ">>> {Step 4: Installing realsense2 and catkin dependencies}"
# echo ""

# # Check if librealsense2 is installed using a marker file
# LIBREALSENSE2_MARKER="$HOME/.librealsense2_installed"
# if [ -f "$LIBREALSENSE2_MARKER" ]; then
#     echo "> librealsense2 is already installed"

# else
#     echo "> Installing dependencies for librealsense2..."
#     (sudo apt-get update 2>&1 ) &
#     pid=$!
#     show_loading_animation $pid
#     wait $pid
#     (sudo apt-get install -y git cmake libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev 2>&1 ) &
#     pid=$!
#     show_loading_animation $pid
#     wait $pid
    
#     if [ $? -eq 0 ]; then
#         echo "> Dependencies for librealsense2 installed successfully"
#     else
#         echo "> Failed to install dependencies for librealsense2"
#         exit 1
#     fi

#     echo "> Cloning and installing librealsense2 from source..."
#     cd ~/dev
#     git clone https://github.com/IntelRealSense/librealsense.git
#     cd librealsense
#     ./scripts/setup_udev_rules.sh
#     ./scripts/patch-realsense-ubuntu-lts.sh
    
#     rm -rf build
#     mkdir build && cd build
#     (cmake .. -DCMAKE_BUILD_TYPE=Release 2>&1 ) &
#     pid=$!
#     show_loading_animation $pid
#     wait $pid
#     (make -j4 2>&1 ) &
#     pid=$!
#     show_loading_animation $pid
#     wait $pid
#     (sudo make install 2>&1 ) &
#     pid=$!
#     show_loading_animation $pid
#     wait $pid
#     if [ $? -eq 0 ]; then
#         echo "> librealsense2 installed successfully"
#         touch "$LIBREALSENSE2_MARKER"
#     else
#         echo "> Failed to install librealsense2"
#         exit 1
#     fi
# fi

# # Check and install catkin if not already installed
# if dpkg-query -W ros-noetic-catkin > /dev/null 2>&1; then
#     echo "> ros-noetic-catkin is already installed"
# else
#     echo "> Installing ros-noetic-catkin..."
#     sudo apt-get install -y ros-noetic-catkin
#     if [ $? -eq 0 ]; then
#         echo "> ros-noetic-catkin installed successfully"
#     else
#         echo "> Failed to install ros-noetic-catkin"
#         exit 1
#     fi
# fi

echo "#######################################################################################################################"
echo ">>> {Step 4: Setting ORBSLAM3 Environment}" 
echo ""

cd ~/dev
# Check if the repository already exists
if [ -d "ORB_SLAM3" ]; then
    echo "ORB-SLAM3 repository already exists."
    cd ~/dev/ORB_SLAM3
else
    # Clone the repository
    echo "Cloning ORB-SLAM3..."
    REPO_URL="https://github.com/aliaxam153/ORB_SLAM3.git"
    git clone $REPO_URL
    if [ $? -eq 0 ]; then
        echo "Repository cloned successfully."
        cd ORB_SLAM3
    else
        echo "Failed to clone the repository."
        exit 1
    fi
fi

# Verify the repository using git fsck
echo "Verifying the repository..."
( git fsck > /dev/null 2>&1 ) &
pid=$!
show_loading_animation $pid
wait $pid

if [ $? -eq 0 ]; then
    echo "Repository verification completed successfully."
else
    echo "Repository verification failed."
    # Clone the repository
    echo "Cloning ORB-SLAM3..."
    REPO_URL="https://github.com/aliaxam153/ORB_SLAM3.git"
    git clone $REPO_URL
    if [ $? -eq 0 ]; then
        echo "Repository cloned successfully."
        cd ~/dev/ORB_SLAM3
    else
        echo "Failed to clone the repository."
        exit 1
    fi
fi

cd ~/dev/ORB_SLAM3
rm -rf build_prerequisities.sh

# Define the lines to add
LINE1_TO_ADD="source /opt/ros/noetic/setup.bash"
LINE3_TO_ADD="export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:$HOME/dev/ORB_SLAM3/Examples/ROS"
LINE2_TO_ADD="export CMAKE_PREFIX_PATH=/opt/ros/noetic:$CMAKE_PREFIX_PATH"

# Add ROS setup to ~/.bashrc
if ! grep -Fxq "$LINE1_TO_ADD" ~/.bashrc; then
   echo "$LINE1_TO_ADD" >> ~/.bashrc
   echo "Added ROS setup to ~/.bashrc"
else
   echo "ROS setup already in ~/.bashrc"
fi

if ! grep -Fxq "$LINE2_TO_ADD" ~/.bashrc; then
   echo "$LINE2_TO_ADD" >> ~/.bashrc
   echo "Added ORBSLAM3 ROS Package setup to ~/.bashrc"
else
   echo "ORBSLAM3 ROS Package setup already in ~/.bashrc"
fi

if ! grep -Fxq "$LINE3_TO_ADD" ~/.bashrc; then
   echo "$LINE3_TO_ADD" >> ~/.bashrc
else
   echo ""
fi

# Source the ~/.bashrc to apply changes
source ~/.bashrc
echo "Sourced ~/.bashrc"


# Switch to G++ and GCC version 11
echo ">> Switching from Default G++/GCC V9 --> V11..."
sudo update-alternatives --config g++ <<EOF
1
EOF
sudo update-alternatives --config gcc <<EOF
1
EOF
echo ""
echo ">> Verifying G++ & GCC Version..."
g++ --version && gcc --version
echo ""

echo "Setup completed successfully!"


