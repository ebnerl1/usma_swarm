# Vehicle Configuration (ODROID / PIXHAWK)

### Create ODROID Image Copy from Existing SASC Vehicle
1. Acquire known working SASC configured vehicle
  * Can check functionality in SASC Flight Tech Interface (see SASC documentation)
2. Carefully remove EMMC card from ODROID
3. Connect EMMC card to laptop
  * Insert EMMC card into EMMC to mirco-SD card adapter  
  * Insert that micro-SD card adapter into a micro-SD to USB adapter (NOTE: Do not use a micro-SD to SD card adapter, this must go through USB)  
  * Insert the USB adapter into a USB port on the computer
4. Verify Designation of the USB Disk on the Laptop  
  * `dh -h`   
  * Copy which drive has the "BOOT" and "trusty" drive partitions (ex: /dev/sdb1 BOOT, /dev/sdb2 trusty; means the USB disk is /dev/sdb)  
  * Confirm this by removing the USB, rerunning `dh -h`, and confirming that disk has disappeared
5. Copy only the part of the image we care about onto computer (instead of whole EMMC card)  
  * NOTE: these disks typically have 4 partitions: (0) Unallocated Space (~1.6MB), (1) BOOT (~135MB), (2) trusty (~5.8GB), (3) Unallocated (rest of drive space). We only care about parts 0-2.  
  * `sudo fdisk -l /dev/sdb` (replace "sdb" with the USB drive designation)  
  * Copy "End" number for /dev/sdb2 (ex: 10915839)  
  * Multiply that number by *(512/(4*1024*1024)) and round up (ex: 10915839*(512/(4*1024*1024)) = 1333)  
  * CAUTION: by very careful using the next command `dd`, particularly with the input `if` and output `of` file arguments. Misuse can corrupt your computer's system.  
  * `sudo dd if=/dev/sdb of=/home/user1/odroid_images/backup.img status=progress`(may need to create odroid_images directory)  
  * More information available at: \sasc_docs\html\deploy under "Creating an image for cloning"
6. Copy the image to a new EMMC card  
  * Remove the copied EMMC card from the laptop  
  * Insert the new EMMC card into the laptop using the same steps as above  
  * Verify designation of the USB Disk using `dh -h` and the same procedure as above (ex /dev/sdb)  
  * `sudo dd if=/home/user1/odroid_images/backup.img if=/dev/sdb status=progress` (CAUTION: see above caution using `dd` command)
7. Check functionality of new EMMC card by plugging it into ODROID of copied vehicle  
  * Functionality should be identical in SASC Flight Tech Interface with original card  
  

