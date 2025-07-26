# Introduction

This file details the full procedure to (re-)image an Aetina RK3588 board fitted with Axelera Metis M.2 accelerator card and prepare it for usage. Information here has been gathered from Axelera help pages, community posts and also from information shared by the very helpful [Shabaz](https://github.com/shabaz123) . I had the 16GB RAM version of the Aetina RK3588 board. All the instructions will also hold for the 8GB version, except that swapfile should also be enabled for the 8GB version.


# Flashing the board

These instructions hold for the Aetina RK3588 board. The instructions are taken from the Axelera page:

https://support.axelera.ai/hc/en-us/articles/25556440050194-System-Imaging-Guide-Aetina-RK3588

A Ubuntu x86/x64 "host" system is needed for this procedure (the RK3588 board cannot itself be used for this). I used ubuntu 22 "live" usb disk running on a laptop (this prevents needing to install the OS to the machine). I also tried with a Virtual Machine (Vmware), which worked fine as well.

Inside the host ubuntu, download the image (get latest link from page above, in case there is a newer version). Below is link to image as of July 2025:

https://axelera-public.s3.eu-central-1.amazonaws.com/host-images/v1.0.0-a6-0-g544580a49/Aetina-ubuntu-22.04-v1.1.tgz

`wget https://axelera-public.s3.eu-central-1.amazonaws.com/host-images/v1.0.0-a6-0-g544580a49/Aetina-ubuntu-22.04-v1.1.tgz`

After downloading the file, do this:
```
sudo tar -xvzf Aetina-ubuntu-22.04-v1.1.tgz 
cd rockdev
chmod +x upgrade_tool
```

Install jumpter between pins 2,3 of DEBUG3 header on board, connect USB-C port of board to host system (using USB-A to USB-C cable). Then power on the board. After several seconds, remove the jumper from the board. 

Run following commands to check board is connected and status:

`./upgrade_tool LD` 

Axelera link mentioned the board should appear like this, in "Loader" mode:

>List of rockusb connected(1)
>
>DevNo=1 Vid=0x2207,Pid=0x350b,LocationID=31     Mode=Loader     SerialNo=1da8b74954605d1d

However, my board was going into "Maskrom" mode only. 

>List of rockusb connected(1)
>
>DevNo=1 Vid=0x2207,Pid=0x350b,LocationID=11     Mode=Maskrom     SerialNo=

However, I found out that it didn't matter actually. I was able to proceed with firmware update as normal. To start the flashing, use command:

`./flash.sh`

The process will finish like this (last few lines only)
>Download image ok.
>
>---- reset device ----
>
>Reset Device OK. 


Now, remove USB-C cable and unplug power, then plug it back in (power-cycle, and not just restart).

Connect to Mouse/Keyboard, HDMI cable and power on.


# Setting Default User and Auto-login

By default, the image comes with two users "firefly" and "aetina", and the board boots into "firefly" user on power up. Since I noticed that a lot of documentation uses the "aetina" user, I decided it's best to set "aetina" as the default user so that the board conveniently boots into it on power up.

The relevant setting is in the file `/etc/lightdm/lightdm.conf.d/20-autologin.conf`. Open this file using any text editor (vi/vim/nano/gEdit, etc) and look for the line `autologin-user=firefly`. Change it to `autologin-user=aetina`, and then reboot the board `sudo reboot`.

Once the board reboots, it will auto-login into the "aetina" user, which can be confirmed by executing the command `whoami` in a terminal. At this point, if you want to remove the firefly user completely, type following command in the terminal:

`sudo deluser --remove-home firefly`

This will remove (delete) the user "firefly" and also remove the home folder for the user (this helps recover some space on the flash).



# Getting Wifi + BT to work

Before going further, it makes things easy to connect to internet with an Ethernet cable (can also share internet from the Ubuntu or windows host PC if router is far). 

Another easy alternative for getting internet access is using the RTL8188 USB dongle provided by Axelera to connect to internet. Use instructions provided by Shabaz:

https://github.com/shabaz123/rtl8188eus/blob/master/README.md 

Note that speed from this dongle is not great (works on 2.4Ghz only) but ok to get internet quickly.

To get better speed, I installed a RTL8822CE based M.2 card (5.8Ghz+ 2.4Ghz Wifi, Bluetooth) on the back side of the board - Credit to Shabaz for the idea!

The driver for this is available in the "linux-firmware" package inside the linux git. However ,the whole package is more than a GB in size and not suitable to download to an embedded board like this. Instad, we can manually download and install the driver files.

Download the latest driver files from the linux repository (click file name, then download raw/plain - very important to do like this. Right-click->save does not download file correctly):

https://git.kernel.org/pub/scm/linux/kernel/git/firmware/linux-firmware.git/tree/rtw88

We only need two files:
>rtw8822c_fw.bin
>
>rtw8822c_wow_fw.bin

Alternatively, we can download via wget in terminal (link used are raw/plain file links, which downloads the files correctly):
```
wget https://git.kernel.org/pub/scm/linux/kernel/git/firmware/linux-firmware.git/plain/rtw88/rtw8822c_fw.bin
wget https://git.kernel.org/pub/scm/linux/kernel/git/firmware/linux-firmware.git/plain/rtw88/rtw8822c_wow_fw.bin
```

Next, with a terminal open in the directory where files were downloaded, run following commands:
```
sudo mkdir -p /lib/firmware/rtw88
sudo mv rtw88*.bin /lib/firmware/rtw88/
sudo modprobe rtw88_8822ce
sudo reboot
```

The wifi card should be functioning now and you can search and connect to a network using the toolbar on top-right of Ubuntu GUI. 

For more detailed instructions, refer to:

https://github.com/shabaz123/axelera/tree/main/rtw88

For this card, I did not have to do anything to get the bluetooth to work - it worked out of the box.


# Set up SD card to extend storage and swap file
The RK3588 board comes with 32GB of Flash storage. After the ubuntu 22 image is installed on the board it only leaves ~10GB storage available, which is quite small and fills up really quickly. Hence extending it with a fast SD card or a M.2 SSD is very important. 

Main article to refer to:

https://support.axelera.ai/hc/en-us/articles/25556514922514-Solving-Storage-Space-Issues-on-Aetina-Evaluation-System

Some commands have been modified, though. 

Insert sdcard, and confirm presence by using command:

`lsblk -f'

The SD card will appear as mmcblk1p1 (or something similar?). In instructions below we will assume it is "mmcblk1p1".

```
sudo mkfs.ext4 -F -L microsd /dev/mmcblk1p1

cd /mnt
sudo mkdir microsd
sudo bash -c "echo \"LABEL=microsd /mnt/microsd ext4 defaults,user,exec,nofail 0 2\" >> /etc/fstab"
sudo mount /mnt/microsd
sudo chown aetina:aetina /mnt/microsd
```

Next, we want to link some of the main folders used by Axelera sdk, etc to the SD card so that the they dont end up using up the EMMC.

First we remove the /axelera folder and recreate as a symbolic link to the SD card
```
sudo rm -rf /axelera
mkdir /mnt/microsd/axelera
sudo ln -s /mnt/microsd/axelera /axelera

```

Next, to prevent Axelera SDk from filling up the /home/aetina/.cache/axelera/data folder, we link it to SD card also

```
sudo mkdir /mnt/microsd/aetina_cache_axelera
sudo chmod aetina:aetina /mnt/microsd/aetina_cache_axelera

ln -s /mnt/microsd/aetina_cache_axelera /home/aetina/.cache/axelera
mkdir /mnt/microsd/aetina_cache_axelera/data

```

# Setting up SWAP file

Setting up SWAP file is recommended for the 8GB RAM version of the Aetina RK3588 board. However, I found that the board shipped to me by Axelera is the 16GB RAM Aetina RK3588 boards, hence this was not needed. This can be confirmed by typing the following in a console:

`free -h`

Look at the 'total' under command output. In my case, it showed "15Gi" which points to it having 16GB RAM.

For the 8GB version board, follow the procedure at this link:
https://support.axelera.ai/hc/en-us/articles/25556440050194-System-Imaging-Guide-Aetina-RK3588



# Install Docker

Follow the procedure here:

https://docs.docker.com/engine/install/ubuntu/

Axelera recommends using the method under "Install using the apt repository" heading on the linked page.


# Install Voyager SDK

The latest procedure for this is detailed at the Axelera link:

https://github.com/axelera-ai-hub/voyager-sdk/blob/HEAD/docs/tutorials/install.md

Very importantly, I changed directory to /axelera before cloning the SDK so that all data is downloaded to the SD card instead of EMMC.

```
cd /axelera
<proceed with git clone & installation commands>
```

After this, I proceeded with following the commands in the Axelera link.

Important observation: All packages did not get installed the first time I executed the ./install.sh script in Voyager SDK. There were errors about "holding broken packages" (answer "y" when it asks). My solution was to keep re-executing the install script, and every time it would go further, until finally everything was installed correctly.

Note that any time we need to use the Voyager SDK, we need to open a terminal, and then execute the following commands to activate the virtual environment:

```
cd /axelera/voyager-sdk/
source venv/bin/activate
```

# Upgrade M.2 Metis module firmware (if needed)

The Metis M.2 device runs its own "firmware" and "board controller firmware". The version on the installed M.2 Metis module can be checked by executing command `axdevice` in a terminal, AFTER activating the voyager venv:

```
cd /axelera/voyager-sdk/
source venv/bin/activate

axdevice
```

Example output:

>Device 0: metis-0:1:0 1GiB m2 flver=1.3.2 bcver=1.4 clock=800MHz(0-3:800MHz) mvm=0-3:100%

"flver" displays the firmware version, and "bcver" displays the board controller firmware version.

Important Note: Due to some risk of bricking the device in case of interruption or power failure while updating, only update the firmwares if your device is running an older firmware.

To update navigate to `Github -> axelera-ai-hub -> voyager-sdk -> docs -> tutorials -> firmware_flash_update.md` and follow the procedure detailed there. Since the exact commands/procedure may change in later releases, I am not replicating the commands here. 



# Configure for remote access

Remote access is very important for embedded computers, and different tools work better/worse on different platforms so I prefer to document this. In my case my remote access tool of choice at the time of this writing is "NoMachine". To install, go to `www.nomachine.com` and navigate to the download page. Among platofrms, choose "NoMachine for ARM". Among the options under "ARM 64-bit", choose "NoMachine for ARM DEB (arm64)". It's very important to select this particular option, since the others did not get installed on the RK3588 board.

Once downloaded, go to Downloads folder and then install (adjust name of .deb file in command below as per actual)

```
cd ~/Downloads/
sudo dpkg -i nomachine_XXXXXXXXXXXXXXXX.deb
```

One issue I saw was that after every reboot, the NoMachine services needed to be manually restarted, otherwise it would not allow remote connections. This is solved by typing the following command in a terminal

`sudo /etc/NX/nxserver --startmode nxd automatic`

Now it should be possible to connect to the board after booting without any manual intervention.
