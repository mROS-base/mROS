# mROS

A lightweight runtime environment of ROS1 nodes onto embedded systems

## Current Platform (embedded board)

- Renesas GR-PEACH

## How to get

- $ git clone --recursive https://github.com/tlk-emb/mROS

## Development Platform and Tools for Host PC

- IDE: [Atollic TrueSTUDIO](https://atollic.com/truestudio/)
  - (under review) Windows 10 Pro
  - Ubuntu 16.04.5
    - Currently we tested v.8.0.0 and v.9.0.1
- CUI
  - macOS High Sierra 10.13.6 / arm-none-eabi version 5.4.1 20160609
  - Ubuntu 16.04.5 / gcc-arm-none-eabi version 4.9.3 20150529

Please let us know if you could develop build anothoer host OS.

## SW Components

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

## Build

- For TrueSTUDIO
  - Specify and open `workspace` as workspace
  - Import `workspace/*` such as `camera_app/`
    - Currently `app/` cannot be built
  - Describe `USE_TRUESTUDIO = true` on Makefile
  - You can bulid and debug the project
- For CUI (terminal)
  - cd to project dir such as `workspace/asp_sample1/`
  - Comment-out such as `#USE_TRUESTUDIO = true` or describe `USE_TRUESTUDIO = false` on Makefile
  - `$ make` or `$ make depend && make`

## TODO

- Testing `make` on Windows
- Build `examples/app/`


## References

