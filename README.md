mt6592 kernel source
=========================================
# To see a list of typical targets execute "make help"
# More info can be located in ./README
# Comments in this file are targeted only to the developer, do not
# expect to learn how to build the kernel reading this file.

=========================================
cd ~/linux_kernel
export ARCH=arm
make mrproper
make fly_j608_defconfig
./build.sh

=========================================
* Not Working
  * MAIN IMGSENSOR
  * SUB IMGSENSOR
  * Dual SIM (Maybe working. Not tested.)
