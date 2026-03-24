#! /bin/zsh

map_dir="/home/user/agrobot_ws/src/robot_nav/maps" # 地图保存路径
map_name=$(date +%m%d)

# 检查文件夹是否存在, 如果不存在就创建文件夹
if [ ! -d "$map_dir" ];then
  echo "文件夹不存在, 正在创建文件夹"
  mkdir -p $map_dir
fi

# finish slam 序列化
ros2 service call finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"

# 等待1秒，确保轨迹已经完成
sleep 1

# make pbstream
ros2 service call write_state cartographer_ros_msgs/srv/WriteState "{filename: '$map_dir/$map_name.pbstream'}"

# 等待2秒，确保pbstream文件已经写入
sleep 2

echo "正在尝试转换地图..."
# pbstream to map 第一种方法
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
-pbstream_filename=$map_dir/$map_name.pbstream \
-map_filestem=$map_dir/$map_name -resolution=0.03 

echo "建图完成！"

