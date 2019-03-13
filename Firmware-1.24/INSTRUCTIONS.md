

# Maslow Firmware Setup

Installing new firmware on your machine is important. We come out with a new firmware version every other week so be prepared to do this regularly. This process will also install the proper drivers to connect to your Arduino on some older computers so if you have trouble connecting it can be helpful to do this process on the same computer you will control the machine with.

### Step 1: Connect Your Arduino
Connect your Arduino to your computer using the provided USB cable.

### Step 2: Download The Arduino IDE
Download and install the last Arduino IDE from [https://www.arduino.cc/en/Main/Software](https://www.arduino.cc/en/Main/Software). Older versions of Arduino IDE have problems with libraries when compiling the firmware, so make sure you have the latest version.

Note - For Windows there are three options: "Windows Installer", "Windows Zip", and "Windows App".
       Some users have reported problems with the "Windows App" version. 
![Download IDE](https://raw.githubusercontent.com/MaslowCNC/Firmware/master/Documentation/Download%20IDE.jpg)

### Step 3: Download The Latest Maslow Firmware
You can do this at http://github.com/MaslowCNC/Firmware/releases/  Click the zip file for the most recent release to download it. Extract the files from the zip folder.
![Download Firmware](https://raw.githubusercontent.com/MaslowCNC/Firmware/master/Documentation/Download%20Firmware.jpg)

### Step 4: Open Firmware
Click **File -> Open** and then open the firmware by selecting cnc_ctrl_v1.ino
![Open Firmware](https://raw.githubusercontent.com/MaslowCNC/Firmware/master/Documentation/Open%20Firmware.jpg)

### Step 5: Select The Board Type
Select the board type by clicking **Tools -> Board -> Arduino/Genuino Mega or Mega 2560**
![Select Board Type](https://raw.githubusercontent.com/MaslowCNC/Firmware/master/Documentation/Select%20Board.jpg)

### Step 6: Select The Serial Port 
Select the correct port to connect to by clicking **Tools -> Port -> Your Port**. On Windows this will be something like COM3, on Mac and Linux computers it will be something like dev/tty/. You can find the right one by plugging and unplugging your Arduino compatible board and checking which option disappears. 
![Select Serial Port](https://raw.githubusercontent.com/MaslowCNC/Firmware/master/Documentation/Select%20COM%20Port.jpg)

### Step 7: Upload The Firmware
Upload the newest firmware to your machine by clicking the upload button in the top left corner. The arrow looks disabled until you hover over it!  _Linux users_: if you are getting timeout or permissions errors, you may need to add your username to the `dialout` group and then logout and back in.  [Instructions here.](https://askubuntu.com/questions/112568/how-do-i-allow-a-non-default-user-to-use-serial-device-ttyusb0)
![Upload Firmware](https://raw.githubusercontent.com/MaslowCNC/Firmware/master/Documentation/Upload.jpg)

### Step 8: Finish
You are now running the latest firmware. *Great Job!* Make sure you close the Arduino IDE before proceeding.

### Step 9: Proceed
You have finished setting up the Maslow firmware. Proceed to the [next step](http://maslowcommunitygarden.org/GroundControl.html?instructions=true) to install Ground Control on your OS.
