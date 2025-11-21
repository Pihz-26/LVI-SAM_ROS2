source /home/ubt2204/CODE/Location/install/setup.bash &&\
ros2 run pcl_ros pcd_to_pointcloud --ros-args \
  -p file_name:="/home/ubt2204/CODE/Datasets/save/0918/Map.pcd" \
  -p publish_rate:=1 \
  -p tf_frame:="/map"