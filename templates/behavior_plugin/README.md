# Behavior plugin template

A minimal package with one simport behavior plugin: what a device answers, decided inside the
device's process. Use it for fast control loops (motor buses: thousands of packets per
second); for slower devices, a node serving the receive's service
([templates/device_node.py](../device_node.py)) is simpler.

    cp -r templates/behavior_plugin ~/my_ws/src/my_simport_behavior
    cd ~/my_ws && colcon build --packages-select my_simport_behavior && source install/setup.bash
    ros2 run simport_ros simport_check src/my_simport_behavior/example.yaml

Then rename the package (package.xml, CMakeLists.txt, plugins.xml), the class, and the
messages and fields it binds. `simport_examples/devices/damiao/damiao_bus.cpp` is a complete example
for a motor bus.
