mt6592 kernel source
=========================================
# To see a list of typical targets execute "make help"
# More info can be located in ./README
# Comments in this file are targeted only to the developer, do not
# expect to learn how to build the kernel reading this file.

=========================================
cd ~/j608_kernel
export ARCH=arm
export ARCH_MTK_PLATFORM=mt6592
make clean
make fly_j608_defconfig
./build.sh

=========================================
* Not Working
  * MAIN IMGSENSOR
  * SUB IMGSENSOR
