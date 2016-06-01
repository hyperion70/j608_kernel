kernel source for Fly Tornado One
=========================================
Basic   | Spec Sheet
-------:|:-------------------------
CPU     | 1.4GHz Octa-Core MT6592
GPU     | Mali - 450 MP4
Memory  | 1GB RAM
Shipped Android Version | 5.1
Storage | 8GB
Battery | 1920 mAh
Display | 5" 720 x 1280 px
Camera  | 13MPx + 5Mpx, LED Flash

![Fly](http://mobiltelefon.ru/i/other/september14/22/iq4511_2.jpg "Fly Tornado One")

=========================================
cd ~/j608_kernel
mkdir out
make ARCH=arm ARCH_MTK_PLATFORM=mt6592 O=out fly_j608_defconfig
make ARCH=arm ARCH_MTK_PLATFORM=mt6592 O=out

=========================================
* Not Working
  * DT2WAKE
