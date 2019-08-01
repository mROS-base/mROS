# mROS

A lightweight runtime environment of ROS1 nodes onto embedded systems

| *master* | *develop* |
|----------|-----------|
|[![Build Status](https://travis-ci.org/tlk-emb/mROS.svg?branch=master)](https://travis-ci.org/tlk-emb/mROS)|[![Build Status](https://travis-ci.org/tlk-emb/mROS.svg?branch=develop)](https://travis-ci.org/tlk-emb/mROS)|

## Supported Platform

- Embedded board
  - [Renesas GR-PEACH](http://gadget.renesas.com/en/product/peach.html)
- Host devices
  - ROS Kinetic with Ubuntu 16.04

### Development Platform/Tools for Host PC

- IDE: [Atollic TrueSTUDIO](https://atollic.com/truestudio/)
  - Windows 10 Pro
  - Ubuntu 16.04.5
    - Currently we tested v.8.0.0, v.9.0.1 and v.9.1.0
- CUI
  - macOS High Sierra 10.13.6 / arm-none-eabi version 5.4.1 20160609 (Launchpad distribution)
  - Ubuntu 16.04 LTS / gcc-arm-none-eabi version 4.9.3 20150529 (apt package)
    - $ sudo apt install gcc-arm-none-eabi
  - Ubuntu 14.04.5 LTS / gcc-arm-none-eabi version 4.9.3 20150529 ([Launchpad distribution](https://launchpad.net/gcc-arm-embedded/4.9/4.9-2015-q3-update))

Please let us know if you could develop and build another host OS.

### SW Components

- [asp-gr_peach_gcc-mbed](https://github.com/tlk-emb/asp-gr_peach_gcc-mbed)
  - Open-source Software Platform Based on TOPPERS/ASP Kernel, mbed and Arduino Library for Renesas GR-PEACH.
  - located at `asp_mbed/asp-gr_peach_gcc-mbed` as gitsubmodule
- [opencv-lib](https://github.com/d-kato/opencv-lib.git)
  - located at `opencv-lib` as gitsubmodule
- [TOPPERS configurator](http://toppers.jp/cfg-download.html)
  - located at `asp_mbed/cfg_binary`
  - (for Win) cfg-mingw-static-1_9_6.zip
  - (for Linux) cfg-linux-static-1_9_6.gz
    - $ sudo apt install libstdc++6 lib32stdc++6
  - (for macOS) cfg-osx-static-1_9_5.gz

## Features

- Topic based publish/subscribe communication with host devices (such as laptop)
- Automatic generation of header files for customized message types
  - We currently support [std_msgs](http://wiki.ros.org/std_msgs) (except for Time, Duration and Header) and [sensor_msgs/Image.msg](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html)
  - In addition, we support `MessageTypes` as customized message types for communication. See [mros_ws/custom_pubsub](mros_ws/custom_pubsub) as the examples.

## How to get

```bash
$ git clone --recursive https://github.com/tlk-emb/mROS
```

`--recursive` option is mandatory since we use git submodules for SW components

## Build

### For host device (ROS applications)

```bash
source /opt/ros/kinetic/setup.bash
cd ros_catkin_ws/
catkin_make
```

### For embedded device (mROS applications)

- (Optional) For generation of customized message
  - Describe `GEN_MSGS = true` on app's Makefile
  - Edit app's JSON file such as `mros_ws/custom_pubsub/msg_app.json` for the customization of message types if you prefer
    - Specify headers for message types that are used in your app such as follows
    ```
    "including_msgs": [
      "custom_pubsub/UserTypeTest.h"
    ]
    ```
    - Specify depending packages for message types such as 
    ```
    "depending_packages": [
      "std_msgs",
      "custom_pubsub"
    ]
    ```
  - `python2` and `jinja2` python package is needed to operate the message generation script
    - `pip install jinja2`

- For CUI (terminal)
  - cd to project dir such as `mros_ws/string_pubsub/`
  - Describe `USE_TRUESTUDIO = false` or comment-out such as `#USE_TRUESTUDIO = true` on Makefile
  - `$ make`
- For TrueSTUDIO
  - Specify and open `mros_ws` as workspace
  - Import `mros_ws/*` such as `string_pubsub`
  - Describe `USE_TRUESTUDIO = true` on Makefile
  - You can build and debug the project on GUI

## Examples

### string_pubsub

- pub/sub communication between host/ROS and embedded/mROS.
  - Each message is realized as `String` type.
- mROS publishes the distance to obstacle by ultrasonic sensors
  - We use SainSmart HC-SR04
  - Start/Stop of publication can be switched by USER push-SW
- mROS subscribes the command for blinking LED
  - `red` / `green` / `blue` can be published from host/ROS

### custom_pubsub

- pub/sub communication with customized MessageType
- custom_pub_sub/UserTypeTest
  - PersonName nameVal
    - string firstName
    - string lastName
  - LEDValues ledVal
    - float32 red
    - float32 green
    - float32 blue
- mROS subscribes the customized message
  - Print full name
  - Blink LED 
- mROS publishes the subscription data to host device

### image_publisher

- mROS publishes image data from camera
  - [GR-PEACH AUDIO CAMERA Shield](https://www.core.co.jp/product/m2m/gr-peach/audio-camera.html#audio_camera) is needed
  - sensor_msgs/Image is used as the communication type
- Host can subscribe the image with `$ rosrun image_view image_view /image:=/image_raw`

## Limitation & TODO

- Currently we cannot support following primitive types
  - Time
  - Duration
  - Header
- Support the edge detection example application

## References

- Research papers
  - [mROS: A Lightweight Runtime Environment for Robot Software Components onto Embedded Devices](https://dl.acm.org/citation.cfm?id=3337815)
    - Authors: Hideki Takase, Tomoya Mori, Kazuyoshi Takagi and Naofumi Takagi
    - The 10th International Symposium on Highly-Efficient Accelerators and Reconfigurable Technologies (HEART 2019), Nagasaki, Jun 2019.
  - [Work-in-Progress: Design Concept of a Lightweight Runtime Environment for Robot Software Components Onto Embedded Devices
](https://ieeexplore.ieee.org/document/8537199)
    - Authors: Hideki Takase, Tomoya Mori, Kazuyoshi Takagi and Naofumi Takagi
    - 2018 International Conference on Embedded Software (EMSOFT), pp. 1-3, Turin, Italy, Sep 2018.
    - doi: 10.1109/EMSOFT.2018.8537199

- Qiita article (in Japanese)
  - https://qiita.com/takasehideki/items/7d783ecd605dcee29ee0

