
```sh
# ビルド
colcon build --symlink-install --cmake-args -DBUILD_TESTING=true -DCMAKE_CXX_FLAGS='-fprofile-arcs -ftest-coverage' -DCMAKE_C_FLAGS='-fprofile-arcs -ftest-coverage' --packages-select autoware_default_adapi_poc

# 自作relayノードを単体で立ち上げる
ros2 run autoware_default_adapi_poc adapi_node --ros-args -r /input_topic_1:=/new_topic  -p num_relay_topics:=5

# 自作relayノードをstandaloneで指定した数立ち上げる
ros2 launch autoware_default_adapi_poc poc_relay_standalone_nodes.launch.py total_nodes:=10
```
