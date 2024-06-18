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
sudo apt update && sudo apt upgrade -y
OUTPUT_SIZE=$(sudo apt-get install -y gedit pv dialog --print-uris | wc -c)
( sudo apt-get install -y gedit pv dialog ) 2>&1 | pv -s $OUTPUT_SIZE > /dev/null
echo ""

# Install build-essential package
echo ">> Installing build-essential package..."
OUTPUT_SIZE=$(sudo apt update && sudo apt-get install -y build-essential --print-uris | wc -c)
( sudo apt update && sudo apt-get install -y build-essential ) 2>&1 |pv -s $OUTPUT_SIZE > /dev/null
echo ">> build-essential packages are installed."
echo ""

echo "#######################################################################################################################"
echo ">>> {Step 2: Install and Setup C++11 Compiler}"
echo ""
# Add the repository for the latest G++ version (if needed for C++11)
echo ">> Adding PPA repository to get the latest G++ version for Current Distro..."
sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y && sudo apt update
echo ">> PPA repository added."
echo ""

# Install g++-11
echo ">> Installing g++-11..."
OUTPUT_SIZE=$(sudo apt install -y g++-11 --print-uris | wc -c)
( sudo apt install -y g++-11 ) 2>&1 | pv -s $OUTPUT_SIZE > /dev/null
echo ">> g++-11 installation completed."
echo ""

echo ">> Setup default G++ to alternatives system..."
# Add the default G++ to alternatives system
echo ">> Adding G++-9 to alternatives..."
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 20
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 20
echo ""
# Add other versions to alternatives
echo ">> Adding G++-11 to alternatives..."
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
echo ">> Added Misc Repositories."
echo ""

echo ">> Refresh all installed packages..."
sudo apt update
echo ""

echo "#######################################################################################################################"
echo ">>> {Step 3: Build and Install All Pre-requisite Packages}" 
echo ""
mkdir -p ~/dev && cd ~/dev

echo "#######################################################################################################################"
echo ">> Pangolin Installation"
echo ""
echo "> Cloning Pangolin..."
REPO_URL="https://github.com/stevenlovegrove/Pangolin.git"
OUTPUT_SIZE=$(git clone $REPO_URL --print-uris | wc -c)
( git clone $REPO_URL ) 2>&1 | pv -s $OUTPUT_SIZE > /dev/null > /dev/null
if [ $? -eq 0 ]; then
    echo "> Cloning Completed"  
else
    echo "> Failed to clone the Pangolin repository."
    exit 1
fi

echo "> Running dry-run for Pangolin prerequisites..."
cd Pangolin
./scripts/install_prerequisites.sh --dry-run recommended
echo ""
echo "> Fixing catch2 check from install_prerequisites.sh..."
sed -i '/catch2/d' ./scripts/install_prerequisites.sh
echo ""
    
# Manual Installation of Catch2############################################################
echo "> Cloning Catch2 repository..."
REPO_URL="https://github.com/catchorg/Catch2.git"
OUTPUT_SIZE=$(git clone $REPO_URL --print-uris | wc -c)
( git clone $REPO_URL ) 2>&1 | pv -s $OUTPUT_SIZE > /dev/null
if [ $? -eq 0 ]; then
    echo "> Cloning Completed"
    echo ""
    echo "> Building Catch2..."
    echo ""
    cd Catch2
    OUTPUT_SIZE=$(cmake -Bbuild -H. -DBUILD_TESTING=OFF --print-uris | wc -c)
    ( cmake -Bbuild -H. -DBUILD_TESTING=OFF ) 2>&1 | pv -s $OUTPUT_SIZE > /dev/null
    OUTPUT_SIZE=$(cmake --build build/ --target install --print-uris | wc -c)
    ( sudo cmake --build build/ --target install ) 2>&1 | pv -s $OUTPUT_SIZE > /dev/null
    echo ""
else
    echo "> Failed to clone the Catch2 repository."
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
mkdir build && cd build
OUTPUT_SIZE=$(cmake .. -DCMAKE_BUILD_TYPE=Release --print-uris | wc -c)
cmake .. -DCMAKE_BUILD_TYPE=Release 2>&1 | pv -s $OUTPUT_SIZE > /dev/null
if [ $? -eq 0 ]; then
    echo "> CMake configuration completed."
else
    echo "> CMake configuration failed."
    exit 1
fi
OUTPUT_SIZE=$(make -j8 --print-uris | wc -c)
make -j8 2>&1 | pv -s $OUTPUT_SIZE > /dev/null
if [ $? -eq 0 ]; then
    echo "> Make completed."
else
    echo "> Make failed."
    exit 1
fi

sudo make install
if [ $? -eq 0 ]; then
    echo "> Pangolin installation completed successfully."
else
    echo "> Pangolin installation failed."
    exit 1
fi


echo "> Updating package list..."
sudo apt update
echo ""

# OpenCV Installation Guide on Ubuntu 20.04
echo "#######################################################################################################################"
echo ">> OpenCV Installation"
echo ""
# Ensure the dev directory exists
mkdir -p ~/dev
cd ~/dev

# Clone OpenCV with a progress bar
echo "> Cloning OpenCV..."
# Define the repository URL for OpenCV
REPO_URL="https://github.com/opencv/opencv.git"
OUTPUT_SIZE=$(git clone $REPO_URL --print-uris | wc -c)
( git clone $REPO_URL ) 2>&1 | pv -s $OUTPUT_SIZE > /dev/null
if [ $? -eq 0 ]; then
    echo "> Cloning Completed"
else
    echo "> Failed to clone the OpenCV repository."
    exit 1
fi

# Checkout the specified version
cd opencv
git checkout 4.4.0
if [ $? -eq 0 ]; then
    echo "> Checked out OpenCV version 4.4.0"
else
    echo "> Failed to checkout OpenCV version 4.4.0"
    exit 1
fi

# Install necessary dependencies
echo "> Installing dependencies for OpenCV..."
sudo apt install -y build-essential cmake git pkg-config libgtk-3-dev \
    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
    libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
    gfortran openexr libatlas-base-dev python3-dev python3-numpy \
    libtbb2 libtbb-dev libdc1394-22-dev libopenexr-dev \
    libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev
if [ $? -eq 0 ]; then
    echo "> Dependencies installed successfully"
else
    echo "> Failed to install dependencies"
    exit 1
fi

# Create a build directory and switch into it
echo "> Creating build directory for OpenCV..."
mkdir build && cd build

# Configure the build with CMake
echo "> Configuring OpenCV build with CMake..."

OUTPUT_SIZE=$(cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local .. --print-uris | wc -c)
    
( cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local .. ) 2>&1 | pv -s $OUTPUT_SIZE > /dev/null
if [ $? -eq 0 ]; then
    echo "> CMake configuration completed successfully"
else
    echo "> CMake configuration failed"
    exit 1
fi

# Build and install OpenCV
echo "> Building and installing OpenCV..."
OUTPUT_SIZE=$(make -j8 --print-uris | wc -c)
make -j8 2>&1 | pv -s $OUTPUT_SIZE > /dev/null
if [ $? -eq 0 ]; then
    echo "> OpenCV built successfully"
else
    echo "> OpenCV build failed"
    exit 1
fi

sudo make install
if [ $? -eq 0 ]; then
    echo "OpenCV installation completed successfully."
else
    echo "OpenCV installation failed."
    exit 1
fi

echo "#######################################################################################################################"
echo ">>ROS Noetic Installation" 
echo ""
# Install ROS Noetic
echo "Downloading and installing ROS Noetic..."
wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh
chmod +x ./ros_install_noetic.sh
./ros_install_noetic.sh

echo "#######################################################################################################################"
echo ">>> {Step 4: Setting ORBSLAM3 Environment}" 
echo ""

cd ~/dev

# Clone the repository
echo "Cloning ORB-SLAM3..."
REPO_URL="https://github.com/aliaxam153/ORB_SLAM3.git"
OUTPUT_SIZE=$(git clone $REPO_URL --print-uris | wc -c)
( git clone $REPO_URL ) 2>&1 | pv -s $OUTPUT_SIZE > /dev/null > /dev/null
if git clone $REPO_URL ; then
    echo "Repository cloned successfully."
    cd ORB_SLAM3

    # Define the lines to add
    LINE1_TO_ADD="source /opt/ros/noetic/setup.bash"
    LINE2_TO_ADD="export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:/home/$user_name/dev/ORB_SLAM3/Examples/ROS"

    # Add ROS setup to ~/.bashrc
    if ! grep -Fxq "$LINE1_TO_ADD" ~/.bashrc; then
        echo "$LINE1_TO_ADD" >> ~/.bashrc
        echo "Added ROS setup to ~/.bashrc"
    else
        echo "ROS setup already in ~/.bashrc"
    fi

    # Source the ~/.bashrc to apply changes
    source ~/.bashrc
    echo "Sourced ~/.bashrc"

    # Add ORBSLAM3 ROS Package setup to ~/.bashrc
    if ! grep -Fxq "$LINE2_TO_ADD" ~/.bashrc; then
        echo "$LINE2_TO_ADD" >> ~/.bashrc
        echo "Added ORBSLAM3 ROS Package setup to ~/.bashrc"
    else
        echo "ORBSLAM3 ROS Package setup already in ~/.bashrc"
    fi

    # Source the ~/.bashrc to apply changes
    source ~/.bashrc
    echo "Sourced ~/.bashrc"
 
else
    echo "Failed to clone the repository."
    exit 1
fi

# Switch to G++ and GCC version 9
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

# Build ORB_SLAM3 Package
echo "Building ORB_SLAM3 Package..."
chmod +x build.sh
./build.sh
if [ $? -eq 0 ]; then
    echo "ORB_SLAM3 built successfully."
else
    echo "Failed to build ORB_SLAM3."
    exit 1
fi

# Build ORB_SLAM3 ROS Package
echo "Building ORB_SLAM3 ROS Package..."
chmod +x build_ros.sh
./build_ros.sh
if [ $? -eq 0 ]; then
    echo "ORB_SLAM3 ROS Package built successfully."
else
    echo "Failed to build ORB_SLAM3 ROS Package."
    exit 1
fi

echo "Setup completed successfully!"

