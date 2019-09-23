# Re-image and configure SASC Vehicle ODROID
This page explains how to copy over the image from an existing SASC ODROID, install it on a new eMMC card for use on another ODROID, and then configure the vehicle ID numbers. NOTE: this only works as is with older RED v3 EMMC Cards and with SD cards.  It does not work with the newer v4 ORANGE EMMC. 

### Create ODROID Image Copy from Existing SASC Vehicle (using ODROID DiskImager)

1. Download Win32DiskImager for Odroid from `https://forum.odroid.com/viewtopic.php?t=947`
2. Acquire known working SASC configured vehicle
  * Can check functionality in SASC Flight Tech Interface (see SASC documentation)
  * Note that vehicle's tail # (the copied image will also be initially configured to that #)
3. Carefully remove EMMC (or SD) card from ODROID
4. Connect EMMC card to laptop
  * Insert EMMC card into EMMC to mirco-SD card adapter  
  * Insert that micro-SD card adapter into a micro-SD to USB adapter
  * Insert the USB adapter into a USB port on the computer
  * NOTE: Do not use a micro-SD to SD card adapter, this must go through USB
5. Use DiskImager tool to "READ" the image file (.img) and download it to the computer  
6. Copy the image to a new EMMC or SD card  
  * Remove the copied EMMC card from the laptop  
  * Insert the new EMMC card into the laptop using the same steps as above  
  * In DiskImager: select the image file (.img) to write to card
  * In DiskImager: carefully select the USB device to which you will write the image
  * In DiskImager: click "WRITE", and then "VERIFY"
7. Check functionality of new EMMC card by plugging it into ODROID of copied vehicle  
  * Functionality should be identical in SASC Flight Tech Interface with original card  
  
### Configure new ODROID vehicle ID and Team Number

1. Insert the imaged EMMC card into the ODROID of the vehicle to reconfigure
  * Make note of the tail number from the image.
  
2. Power up the UAS with a DC power supply.

3. Configure Alfa Wi-Fi dongle on SASC Linux Laptop
  * Attach an Alfa USB Wi-Fi adapter to the computer
  * `ifconfig` - Identify the Alfa Wi-Fi adapter (usually the last one) (ex: wlx00c0ca904414, or wlan2)
  * `wifi_config.sh -T 11 wlx00c0ca904414 201` - Note: 11 is the Team # (11 = Army), 201 is the selected network address for the computer (201-209 are recommended, make sure no other SASC computers are set to that). 
    - "11" is the team # (11 = Army)
    - "201" is the # w/i the team (201-209 recommended for dongles, must be unique from other active SASC computers)
    - This will set this computers the Alfa Wifi IP address to 192.168.11.201

4. Ping the powered up UAS to ensure you can talk to it: `ping 192.168.11.X` (X = tail number, ex: 112)

5. Connect to the UAS via secure shell (SSH)
  * `ssh odroid@192.168.11.X` (X = tail number, ex: `ssh odroid@192.168.11.112`)
  * When prompted for password enter `odroid`
  
6. Run ID reconfiguration script in SSH terminal
  * `./odroid-installer.sh -c`
  * At "Please enter a unique ID for this aircraft: " enter the new tail number desired (planes are 2 digits, quads 3 digits)
  * At "Please enter the team: " enter 11 for Army
  * At "Please enter a hostname for this aircraft: " enter planeXX or copterXXX (X = tail, ex: plane10 or copter110)
7. Power cycle the UAS, then you should be able to ping the new IP address (`ping 192.168.11.X`) 
8. Label the new tail number on the aircraft

### Create ODROID Image Copy from Existing SASC Vehicle (OLDER METHOD using DD command)

Note: this only works with RED EMMC cards (not orange EMMC, and not SD cards)

1. Acquire known working SASC configured vehicle
  * Can check functionality in SASC Flight Tech Interface (see SASC documentation)
  * Note that vehicle's tail # (the copied image will also be initially configured to that #)
2. Carefully remove EMMC card from ODROID
3. Connect EMMC card to laptop
  * Insert EMMC card into EMMC to mirco-SD card adapter  
  * Insert that micro-SD card adapter into a micro-SD to USB adapter
  * Insert the USB adapter into a USB port on the computer
  * NOTE: Do not use a micro-SD to SD card adapter, this must go through USB
4. Verify Designation of the USB Disk on the Laptop  
  * `df -h`   
  * Copy which drive has the "BOOT" and "trusty" drive partitions (ex: /dev/sdb1 BOOT, /dev/sdb2 trusty; means the USB disk is /dev/sdb)
  * Confirm this by removing the USB, rerunning `df -h`, and confirming that disk has disappeared
5. Copy only the part of the image we care about onto computer (instead of whole EMMC card)  
  * NOTE: these disks typically have 4 partitions: (0) Unallocated Space (~1.6MB), (1) BOOT (~135MB), (2) trusty (~5.8GB), (3) Unallocated (rest of drive space). We only care about parts 0-2.  
  * `sudo fdisk -l /dev/sdb` (replace "sdb" with the USB drive designation)  
  * Copy "End" number for /dev/sdb2 (ex: 10915839)  
  * Multiply that number by `*(512/(4*1024*1024))` and round up (ex: `10915839*(512/(4*1024*1024)) = 1333`)  
  * Create directory /home/user1/odroid_images
  * CAUTION: by very careful using the next command `dd`, particularly with the input `if` and output `of` file arguments. Misuse can corrupt your computer's system.  
  * `sudo dd if=/dev/sdb of=/home/user1/odroid_images/backup.img bs=4M count=1333 status=progress`(replace 1333 with your solution)  
  * More information available at: \sasc_docs\html\deploy under "Creating an image for cloning"
6. Copy the image to a new EMMC card  
  * Remove the copied EMMC card from the laptop  
  * Insert the new EMMC card into the laptop using the same steps as above  
  * Verify designation of the USB Disk using `df -h` and the same procedure as above (ex /dev/sdb)  
  * `sudo dd if=/home/user1/odroid_images/backup.img of=/dev/sdb status=progress` (CAUTION: see above caution using `dd` command, make sure `if` and `of` arguments are correct)
7. Check functionality of new EMMC card by plugging it into ODROID of copied vehicle  
  * Functionality should be identical in SASC Flight Tech Interface with original card  
