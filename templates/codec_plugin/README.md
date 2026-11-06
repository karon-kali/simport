# Codec plugin template

A minimal package with one simport codec plugin, for packet formats the YAML `fields:` cannot
describe (here: a proprietary check byte).

    cp -r templates/codec_plugin ~/my_ws/src/my_simport_codec
    cd ~/my_ws && colcon build --packages-select my_simport_codec && source install/setup.bash
    ros2 run simport_ros simport_check src/my_simport_codec/example.yaml

Then rename the package (package.xml, CMakeLists.txt, plugins.xml), the class and the fields.
See docs/writing_plugins.md for the other plugin interfaces.
