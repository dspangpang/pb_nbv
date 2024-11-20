# gw_scanner

## Overview

This package provides urdf for gw scanner

## update
v0.0.1
- 添加了gw_scanner_description使用kinect插件

## keypoints
- kinect的gazebo插件,rgb和点云如果使用同一个frame会对不齐，rgb应参考x为正前方，z轴为正上方的右手坐标系，点云应该参考z轴为正前方，y轴为正下方的右手坐标系
