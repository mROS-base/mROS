# mROS

A lightweight runtime environment of ROS1 nodes onto embedded systems

## Current Platform (embedded board)

- Renesas GR-PEACH

## How to get

- $ git clone --recursive https://github.com/tlk-emb/mROS

## Environment & Build

- Build on TrueSTUDIO (we tested v.8.0.0)
  - Open TrueSTUDIO and specify the workspace to `<git_clone_dir>/examples/truestudio`
  - Import `<git_clone_dir>/examples/truestudio/camera_app` to target project and build it
  - Select `camera_app` as target project and build it

- Build on Terminal (we tested on Mac terminal and arm-none-eabi 5.4.1 20160609)
  - Move to `<git_clone_dir>/examples/camera_app`
  - Build with `$ make depend && make`
    - locate `cfg` to `<git_clone_dir>/asp_mbed/asp-gr_peach_gcc-mbed/asp-1.9.2-utf8/cfg/cfg/`

## TODO


## References

