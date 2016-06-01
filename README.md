mt6592 kernel source
=========================================
# To see a list of typical targets execute "make help"
# More info can be located in ./README
# Comments in this file are targeted only to the developer, do not
# expect to learn how to build the kernel reading this file.

=========================================
cd ~/j608_kernel
mkdir out
make ARCH=arm ARCH_MTK_PLATFORM=mt6592 O=out fly_j608_defconfig
make ARCH=arm ARCH_MTK_PLATFORM=mt6592 O=out

=========================================
* Not Working
  * autofocus

