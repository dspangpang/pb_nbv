# turntable v0.0.7

## Overview

This package provides urdf for icr turntable

## update
v0.0.1
- 添加了turntable_description
  
v0.0.2
- 添加了turntable_ros_controller 支持ros控制实物和moveit_terminal控制

v0.0.3
- 修正了配置文件中的bug

v0.0.4
- 添加turntable_ros_controller中的报错退出功能，以配合launch文件中的重启功能

v0.0.5
- 添加了在转盘usb崩溃之后，使用ioctl来重置USB

v0.0.6
- 使用libusb来重置USB
- 添加转盘程序崩溃前对转盘发送停止信号

v0.0.7
- 修正了除驱动器错误外的异常退出问题


