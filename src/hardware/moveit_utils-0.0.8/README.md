# moveit_utils v0.0.8

## Overview

This package provides some useful function for MoveIt!.

update:
- move_line_middle_interpolation_force 有限制的解决了奇异点的问题
- 修改了 get_current_link_pose 中 (move_group.getCurrentPose)不能同步修改坐标的问题
- 修改了 使用 tf::TransformListener listener 来获取相对位姿
- 添加了对 movejoint pose excute 执行结果的错误判断

## Dependencies

```bash
apt install ros-noetic-trac-ik

apt install libnlopt-cxx-dev
```
