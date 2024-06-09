## Pangolin Installation Guide for Ubuntu 20.04
**1) Clone Pangolin along with it's submodules**
```
git clone https://github.com/stevenlovegrove/Pangolin.git
```
**2) See what package manager and packages are recommended**
```
cd Pangolin
./scripts/install_prerequisites.sh --dry-run recommended
```
> You will see that catch2 is not found in apt . So we will build it separately.
> Remove the catch2 check from``install_prerequisites.sh``, by editing the script:
> ```
> gedit ./scripts/install_prerequisites.sh
> ```
> See line number 124, and ``remove catch2``

**3) Manual Installation of Catch2**

The Catch2 repository is hosted on GitHub, you can follow these steps:

a. **Clone the Catch2 repository**: Use the `git clone` command to clone the Catch2 repository from GitHub¹:
```bash
cd ~/dev
git clone https://github.com/catchorg/Catch2.git
```
b. **Navigate to the cloned directory**: Change your current directory to the cloned Catch2 directory¹:
```bash
cd Catch2
```
c. **Build the files**: Use the `cmake` command to configure and build the files¹:
```bash
cmake -Bbuild -H. -DBUILD_TESTING=OFF
```
d. **Install Catch2**: Finally, install Catch2 by running the following command¹:
```bash
sudo cmake --build build/ --target install
```
e. **Verify Catch2**: To verify if Catch2 is installed, you can check the `/usr/local/include/catch2` directory to see if the Catch2 header files are there. You can do this by running the following command:

```bash
ls /usr/local/include/catch2
```

If Catch2 is installed correctly, you should see a list of header files.

Please note that you need to have `git` and `cmake` installed on your system to perform these steps¹. If you don't have them installed, you can install them using the `sudo apt install git cmake` command.

**4) Install dependencies**
```
cd ~/dev/Pangolin
sudo ./scripts/install_prerequisites.sh recommended
```
**5) Build & Install**
Now that all dependencies are installed, build and install pangolin, as follows:
```
mkdir build && cd build 
cmake .. -D CMAKE_BUILD_TYPE=Release 
make -j 8 
sudo make install
```
