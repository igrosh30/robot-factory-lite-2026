# Preparing your PC

## Software to download

* Visual Studio Code: [VSCode](https://code.visualstudio.com/download)
* Distributed version control system: [Git](https://git-scm.com/downloads)
* USB driver installation made easy: [Zadig](https://github.com/pbatard/libwdi/releases/download/v1.5.0/zadig-2.8.exe)

## Installation Sequence

* Open *Regedit*
  * Go to HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Control\FileSystem in regedit and then set LongPathsEnabled to 1.

* Install Git
   * Open *Git Bash* as administrator
   * Run **git config --global core.longpaths true**
   * Reboot

* Install VSCode

* Open VSCode and click the extensions Icon

  * Search for "PlatformIO"
  * Install PlatformIO
  * VSCode will ask to restart

## Compiling the Project

* In VSCode choose **File->Open Folder...** and select the folder where the project is.
* If it is the first time, the next step can take a long time because PlatformIO wil have to download the compiler, the tools and the libraries. It will happen just for the first time if the process completes without errors. When everything is installed the icon with a checkmark in the bottom must be clicked to build the firmware.