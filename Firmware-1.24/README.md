# Maslow Firmware

This is the firmware which controls the Maslow CNC machine

[![Build Status](https://travis-ci.org/MaslowCNC/Firmware.svg?branch=master)](https://travis-ci.org/MaslowCNC/Firmware)

This is the firmware for the Maslow CNC Router


## Steps to setup the Firmware development environment

First clone the Firmware repository, then install and setup the IDE of your choice.

### Using Arduino IDE
1. Download [Arduino IDE](https://www.arduino.cc/en/main/software) 1.8.1 or higher
2. Install Arduino IDE and run Arduino IDE
3. Navigate menus: File, Open
4. In the file chooser navigate to the cloned repository and choose the "cnc_ctrl_v1.ino" file to open
5. Navigate menu: Tools, Board, change to "Arduino/Genuino Mega or Mega 2560"
6. Navigate menu: Sketch -> Upload

This should compile the project without errors, and possibly some warnings.

### Using PlatformIO
1. Download package for [Atom](https://atom.io/)
2. Follow directions for [installing PlatformIO within Atom](http://docs.platformio.org/en/latest/ide/atom.html#ide-installation)
3. Within Atom navigate menus: PlatformIO, Open Project
4. Select "Firmware" directory
5. Click "Open Firmware" 

### Using Eclipse Neon C/C++ with Sloeber plugin

1. Download [Eclipse C/C++](https://eclipse.org/downloads/) Neon or higher
2. Install Eclipse C/C++ and run Eclipse
3. Install Sloeber Arduino plugin
   * Navigate menus: Help, Install New Software...
   * Copy this URL in the "Work With" field: http://eclipse.baeyens.it/update/V4/stable
   * Select "Add" button
   * Select "Sloeber Arduino IDE" check box
   * Select "Finish" button
   * Accept defaults and accept licenses, the plugin will restart Eclipse, and configure the plugin
4. Change to Arduino perspective, navigate menus: Window, Perspective, Open Perspective, Other...
   * Choose the "Arduino" perspective and select "Ok" button
5. Create an Arduino project
   * Navigate menus: File, New, New Arduino Sketch
   * Project Name: cnc_ctrl_v1
   * Select "Next" button
   * Select appropriate item from "Platform folder" drop down listing
   * Select Board: Arduino/Genuino Mega or Mega 2560
   * Select Upload Protocol: Default
   * Select Processor: ATmega2560 (Mega 2560)
   * Select "Finish" button
6. Import project source code
   * Select project folder, navigate menus: File, Import...
   * Expand "General" and select "File system"
   * Select "Next" button
   * Select the "Browse" button to select the source location (location of the cloned repository cnc_ctrl_v1 directory)
   * Select whole source directory in the left pane
   * Open Advanced Settings by klicking on "Advanced>>" button
   * Select 'Create Links in Workspace' and 'Create virtual folders' leave other settings untouched
   * Select 'Finish' button
   * Eclipse asks if overwriting the original cnc_ctrl_v1.ino file is ok. Confirm with 'yes'.
7. Update eclipse project include paths
   * Select the project folder in the project explorer and click Project->Properties in the menu.
   * In the Project properties left Pane select C/C++ Build->Settings
   * In the right Pane select the 'Tool Settings' Tab and add the path to the source location to the include paths of all compilers/linkers of the toolchain.
   * The last two steps may differ between toolchains.
   
   
### Using NotePad++ in Conjunction with the Arduino IDE
1. Download NotePad++ (Windows only) [link](https://notepad-plus-plus.org/)
2. Download the Arduino IDE          [link](https://www.arduino.cc/en/main/software)
3. Set that you would like use an external editor from within the Arduino IDE by clicking File -> Preferences -> Use External Editor
   *The Arduino editor will no longer allow you to edit the files, but instead will only work to compile and upload your code. 
   *The code can be edited from within NotePad++
   *This method can be used on other platforms with editing programs other than NP++

