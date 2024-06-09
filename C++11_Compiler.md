## G++ Compiler Installation Steps on Ubuntu System
### 1. Install the Build-Essential Package
To install G++ along with all necessary compilers and libraries, the best approach is to install the build-essential package. This package includes G++ and other tools necessary for compiling C++ and other programming languages.
```
sudo apt update && sudo apt install build-essential
```
> ORB-SLAM3 requires C++11 to compile. But for ubuntu 20.04, it is not a stable release, so we need to add the following repo:
> ```
> sudo add-apt-repository ppa:ubuntu-toolchain-r/test
> ```

### 2. Installing a Specific Version of G++

If a specific version of G++ is required, perhaps for compatibility or testing purposes, you can install it alongside the default version:

**a) Check for Available G++ Versions:** To find out which versions of G++ are available for installation.
```
$ apt search '^g\+\+-[0-9]+$'
```
This command lists all the available G++ versions. Choose the version that suits your needs.

**b) Install a Specific Version of G++:** After deciding which version you need, you can install it.
```
sudo apt install g++-11
```
Replace ‘11’ with whichever version number you need.

**c) List all installed GCC versions:** If you have installed multiple versions of GCC, you can list all of them using the dpkg command3. This command will list all installed compilers and their versions3:
```
dpkg --list | grep compiler
```

**d) Add the Default G++ to Alternatives:** First, add the default version of G++ to the alternatives system.
```
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 20
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 20
```
Here, ‘g++-9’ is the default version, and ‘20’ is its priority. Adjust the version and priority according to your needs.

Add Other Versions: Add any other installed versions.
```
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 50
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 50
```
This command adds G++ 11 to alternatives and sets it as a higher priority, making it the default version.

**e) Switch Between G++ & GCC Compiler Versions:** You can switch between the versions using the following command.
```
sudo update-alternatives --config g++
```
This will prompt you to select which version of G++ you wish to be the default by typing the selection number.
Similarly, we use the following command for GCC:
```
sudo update-alternatives --config gcc
```

**f) Verify Installation:** Ensure the specified version of G++ is correctly installed.
```
g++ --version && gcc --version
```
This command checks which version is currently active.
