# 3D Tracker compatible with HTC Vive
This little device is capable of tracking its position in the world. The tracked position is then sent over BLE to the smartphone. The [mobile app](https://github.com/kacer/3D-Tracker-Vive-Android) was created for this purpose and renders the positions of all connected 3D trackers. For tracking 3D Tracker uses two HTC Base Stations which are part of the HTC Vive set for VR. For now, the tracker uses HTC Base Station 1.0 but HW is ready for HTC Base Station 2.0. This project came to the world within my master's thesis. The idea of position calculation is based on repo [vive-diy-position-sensor](https://github.com/ashtuchkin/vive-diy-position-sensor). 

The dimensions of the 3D tracker are 6.3x4.7 cm and can be worn like a watch. The 3D tracker itself looks: 
| ![Tracker1](https://user-images.githubusercontent.com/22794640/83662220-8237ee80-a5c7-11ea-870d-51db626de94d.jpg) | ![Tracker](https://user-images.githubusercontent.com/22794640/83663033-97f9e380-a5c8-11ea-9039-e5c1b7763fb1.jpg) |
| --- | --- |

The demo of position tracking by two 3D trackers on YouTube:
[![The demo of position tracking by two 3D trackers](https://img.youtube.com/vi/TOYSWDCKwdk/0.jpg)](https://www.youtube.com/watch?v=TOYSWDCKwdk)

## Hardware
The 3D tracker is based on **nRF52840** and uses as a sensor of IR light **TS4231 + photodiode BPW34S**. For now, only one sensor is used and thus calibration of positions of HTC Base Station must be taken from SteamVR through this utility [vive-diy-position-sensor-geometry-getter](https://github.com/ashtuchkin/vive-diy-position-sensor-geometry-getter). This utility prints to console the geometry of HTC Base Stations which is consists of position and rotation matrix. This data are then copied and save to txt file. After that, the txt file with geometry is copied to the smartphone and through the [mobile app](https://github.com/kacer/3D-Tracker-Vive-Android) is sent to 3D trackers. Because nRF52840 was used additional sensors are possible to add.

## Getting started
The firmware of 3D Tracker was written in [PlatformIO](https://platformio.org/) with Arduino framework in C++. I recommend using the Visual Studio Code for developing. The installation process of PlatfromIO is pretty simple. Just install it like an extension of the VS Code. Then clone this repository, open it in VS Code, build and upload to your board.

### Uploading firmware into board
I used a [development kit with nRF52840](https://www.nordicsemi.com/Software-and-Tools/Development-Kits/nRF52840-DK) from Nordic Semiconductor for burning firmware into a chip. Wiring was following: ![programming-custom-board](https://user-images.githubusercontent.com/22794640/83662332-9ed42680-a5c7-11ea-8593-a381d4ecfa29.jpg)
