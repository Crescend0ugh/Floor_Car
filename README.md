# FloorBot
The FloorBot is a miniature automobile that autonomously navigates and detects objects on the floor to pick up.

This repository contains the code used for the prototype.

## Libraries Used
- [Recast](https://github.com/recastnavigation/recastnavigation): Navigation mesh generation and pathfinding
- [raylib](https://www.raylib.com/): 3D rendering and visualization for testing
- [asio (standalone)](https://github.com/chriskohlhoff/asio): Socket networking and asynchronous I/O
- [zpp_bits](https://github.com/eyalz800/zpp_bits): Binary serialization for networking
- [OpenCV](https://opencv.org/): Computer vision, object detection and camera calibration
- [Ultralytics YOLO11](https://docs.ultralytics.com/models/yolo11/): Object detection model
- [ncnn](https://github.com/Tencent/ncnn): Lightweight neural network inference with good performance on Raspberry Pi
- [PCL](https://pointclouds.org/): Point cloud searching and filtering, mesh reconstruction
- [Serialib](https://github.com/imabot2/serialib?tab=readme-ov-file): Serial communication between the Arduino UNO and Raspberry Pi
- [SerialTransfer](https://github.com/PowerBroker2/SerialTransfer): Arduino library ported over so the receiving machine can read the packetized data
- [whisper.cpp](https://github.com/ggml-org/whisper.cpp): Voice command detection
- [libgpiod](https://github.com/brgl/libgpiod): GPIO utilities (only required on Raspberry Pi)

## Installing OpenCV (Windows)
OpenCV is not included directly in the repository. Building from its source increases build times dramatically, 
and since GitHub doesn't like sending large files over the network, including the static/dynamic libraries is not an option either.

Unfortunately, the only option left is to install OpenCV yourself.

To do this, head to https://opencv.org/releases/ and download the Windows installer for version 4.12.0.
Run the installer and extract OpenCV to `C:`.

Then, edit your system PATH by adding `C:\opencv\build\x64\vc16\bin` and `C:\opencv\build\x64\vc16\lib`.

You should now be able to build the project. If not, restart your IDE. If that doesn't work (or you aren't using one), restart your computer.

If your OpenCV 4.12.0 lives elsewhere, you must edit both the root and tests `CMakeLists.txt` by changing
the path in the line `set(OpenCV_DIR "C:/opencv/build")` to your OpenCV path.