#!/bin/bash
# Update package list and install gedit
echo "Installing gedit..."
sudo apt update
sudo apt install -y gedit 
# Install build-essential package
echo "Installing build-essential package..."
sudo apt update && sudo apt install -y build-essential
# Add the repository for the latest G++ version (if needed for C++11)
echo "Adding PPA repository for latest G++ version..."
sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y
sudo apt update
# Install specific version of G++ (e.g., G++-11)
echo "Installing G++-11..."
sudo apt install -y g++-11
# Add the default G++ to alternatives system
echo "Adding G++-9 to alternatives..."
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 20
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 20
# Add other versions to alternatives
echo "Adding G++-11 to alternatives..."
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 50
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 50

# Switch to G++ and GCC version 9
echo "Switching to G++ version 9..."
sudo update-alternatives --config g++ <<EOF
2
EOF

echo "Switching to GCC version 9..."
sudo update-alternatives --config gcc <<EOF
2
EOF

# Verify installation
echo "Verifying G++ installation..."
g++ --version
echo "Verifying GCC installation..."
gcc --version

# Pangolin Installation Guide for Ubuntu 20.04
echo "Cloning Pangolin..."
git clone https://github.com/stevenlovegrove/Pangolin.git

echo "Running dry-run for Pangolin prerequisites..."
cd Pangolin
./scripts/install_prerequisites.sh --dry-run recommended

echo "Removing catch2 check from install_prerequisites.sh..."
sed -i '/catch2/d' ./scripts/install_prerequisites.sh

# Manual Installation of Catch2
echo "Cloning Catch2 repository..."
cd ~
mkdir -p dev && cd dev
git clone https://github.com/catchorg/Catch2.git

echo "Building Catch2..."
cd Catch2
cmake -Bbuild -H. -DBUILD_TESTING=OFF
sudo cmake --build build/ --target install

echo "Verifying Catch2 installation..."
ls /usr/local/include/catch2

# Install dependencies for Pangolin
echo "Installing Pangolin dependencies..."
cd ~/dev/Pangolin
sudo ./scripts/install_prerequisites.sh recommended

# Build & Install Pangolin
echo "Building and installing Pangolin..."
mkdir build && cd build
cmake .. -D CMAKE_BUILD_TYPE=Release
make -j 8
sudo make install

# OpenCV 4.4.0 Installation Guide on Ubuntu 20.04
echo "Updating package list..."
sudo apt update

# Install necessary dependencies
echo "Installing dependencies for OpenCV..."
sudo apt install -y build-essential cmake git pkg-config libgtk-3-dev \
    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
    libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
    gfortran openexr libatlas-base-dev python3-dev python3-numpy \
    libtbb2 libtbb-dev libdc1394-22-dev libopenexr-dev \
    libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev

# Download and unpack the OpenCV sources
echo "Cloning OpenCV repository..."
cd ~
mkdir -p dev && cd dev
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout 4.4.0

# Create a build directory and switch into it
echo "Creating build directory for OpenCV..."
mkdir build && cd build

# Configure the build with CMake
echo "Configuring OpenCV build with CMake..."
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local ..

# Build and install OpenCV
echo "Building and installing OpenCV..."
make -j4
sudo make install

# Install ROS Noetic
echo "Downloading and installing ROS Noetic..."
wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh
chmod +x ./ros_install_noetic.sh
./ros_install_noetic.sh

# Install ORB-SLAM3
echo "Cloning ORB-SLAM3..."
cd ~/dev
git clone https://github.com/aliaxam153/ORBSLAM3-WSL.git
cd ORB_SLAM3

echo "Switching compiler back to default C++11 compiler..."
sudo update-alternatives --config g++ <<EOF
1
EOF

echo "Switching to GCC version 11..."
sudo update-alternatives --config gcc <<EOF
1
EOF

echo "Building ORB-SLAM3..."
chmod +x build.sh
./build.sh || ./build.sh || ./build.sh

# Integrate with ORB-SLAM3 ROS
echo "Integrating ORB-SLAM3 with ROS..."
echo "Please add the following line to your .bashrc file:"
echo 'export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/user/dev/ORB_SLAM3/Examples/ROS'
echo 'You can use the following command to edit your .bashrc file:'
echo 'gedit ~/.bashrc'

# Provide instruction to source .bashrc
echo "After editing .bashrc, run the following command to source it:"
echo "source ~/.bashrc"

# Build ROS integration
echo "Building ROS integration for ORB-SLAM3..."
chmod +x build_ros.sh
./build_ros.sh

# Verify all installations
echo "Verifying all installations..."

# Verify Pangolin
if [ -d "/usr/local/include/pangolin" ]; then
    echo "Pangolin installed successfully."
else
    echo "Pangolin installation failed."
fi

# Verify OpenCV
if pkg-config --modversion opencv4 | grep -q '4.4.0'; then
    echo "OpenCV 4.4.0 installed successfully."
else
    echo "OpenCV 4.4.0 installation failed."
fi

# Verify ROS Noetic
if dpkg -l | grep -q "ros-noetic"; then
    echo "ROS Noetic installed successfully."
else
    echo "ROS Noetic installation failed."
fi

# Verify ORB-SLAM3
if [ -d "~/dev/ORB_SLAM3" ]; then
    echo "ORB-SLAM3 installed successfully."
else
    echo "ORB-SLAM3 installation failed."
fi

echo "Setup completed successfully!"

