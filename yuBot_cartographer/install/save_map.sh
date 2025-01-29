#!/bin/bash

# 获取当前的日期和时间，格式为 YYYY-MM-DD_HH-MM-SS
current_datetime=$(date '+%Y-%m-%d_%H-%M-%S')

# 构建地图文件名
map_filename="map_$current_datetime"

# 切换到相应的目录并运行map_saver_cli命令
cd src/qingzhou_cartographer/maps
ros2 run nav2_map_server map_saver_cli -t map -f "$map_filename"
