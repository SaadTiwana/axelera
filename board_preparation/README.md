Note: Information here has been gathered from Axelera help pages, forum posts and also from information shared by the very helpful [Shabaz](https://github.com/shabaz123) .


# Flashing the board

These instructions hold for the Aetina RK3588 board. 

The instructions are taken from this page:

https://support.axelera.ai/hc/en-us/articles/25556440050194-System-Imaging-Guide-Aetina-RK3588

Ubuntu x86/x64 system is needed as host machine. I used ubuntu 22 "live" usb disk. VM worked too.

Inside the host ubuntu, download the image (download latest from link above, if available)

https://axelera-public.s3.eu-central-1.amazonaws.com/host-images/v1.0.0-a6-0-g544580a49/Aetina-ubuntu-22.04-v1.1.tgz

After downloading the file, do this:
```
sudo tar -xvzf Aetina-ubuntu-22.04-v1.1.tgz 
cd rockdev
chmod +x upgrade_tool
```

Install jumpter between pins 2,3 of DEBUG3 header on board, connect USB-C port of board to host system (using USB-A to USB-C cable). Then power on the board. Then remove the jumper from the board. 

Run following commands to check board is connected and status:

`./upgrade_tool LD` 

Link said it should appear like this:

>List of rockusb connected(1)
>
>DevNo=1 Vid=0x2207,Pid=0x350b,LocationID=31     Mode=Loader     SerialNo=1da8b74954605d1d

However, my board was going into Maskrom mode only. However it didn't matter actually. I was able to proceed as normal.

>List of rockusb connected(1)
>
>DevNo=1 Vid=0x2207,Pid=0x350b,LocationID=11     Mode=Maskrom     SerialNo=

Start the flash command:

`./flash.sh`

The process will finish like this (last few lines only)
>Download image ok.
>
>---- reset device ----
>
>Reset Device OK. 


Now, remove USB-C cable and unplug power, then plug it back in.

Connect to Mouse/Keyboard, HDMI cable and power on.




# Getting Wifi + BT to work

Before going further, it makes things easy to connect to internet with an Ethernet cable (can also share internet from the Ubuntu or windows host PC if router is far). 

Another easy alternative for getting internet access is using the RTL8188 USB dongle provided by Axelera to connect to internet. Use instructions provided by Shabaz:

https://github.com/shabaz123/rtl8188eus/blob/master/README.md 

Note that speed from this dongle is not great (works on 2.4Ghz only) but ok to get internet quickly.

To get better speed, I installed a RTL8822CE based M.2 card (5.8Ghz+ 2.4Ghz Wifi, Bluetooth) - Thanks to Shabaz for the idea!

Download the latest driver files from linux repository (right click, save as):

https://git.kernel.org/pub/scm/linux/kernel/git/firmware/linux-firmware.git/tree/rtw88

We only need two files:
>rtw8822c_fw.bin
>
>rtw8822c_wow_fw.bin

Next, open terminal in the directory where files were downloaded and run following commands:
```
sudo mkdir -p /lib/firmware/rtw88
sudo mv rtw88*.bin /lib/firmware/rtw88
sudo modprobe rtw88_8822ce
ifconfig -a
```
Last command should show the new wifi interface. The wifi should be functioning now and you can search and connect to a network using the toolbar on top-right of Ubuntu GUI.

For more detailed instructions, refer to:

https://github.com/shabaz123/axelera/tree/main/rtw88



# Set up SD card to extend storage and swap file
The RK3588 board comes with 32GB of Flash storage. After the ubuntu 22 image is installed on the board it only leaves ~10GB storage available, which is quite small and fills up really quickly. Hence extending it with a fast SD card or a M.2 SSD is very important. 

Main article to refer to:

https://support.axelera.ai/hc/en-us/articles/25556514922514-Solving-Storage-Space-Issues-on-Aetina-Evaluation-System

Some commands have been taken from information provided by Shabaz

Setting up SWAP file is also recommended for the 8GB RAM version of the Aetina RK3588 board.


# Install Docker



# Install Voyager SDK

