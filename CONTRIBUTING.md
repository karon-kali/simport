# Contributing to simport

Thanks for your interest! Bug reports, new example devices, plugins and documentation fixes
are all welcome.

## How the repository is organised

| Package | What goes there |
|---|---|
| `simport_core` | ROS-agnostic core: data model, configuration, channels, plugin interfaces |
| `simport_plugins` | Generic plugins: ports (serial, UDP, TCP, CAN), codecs, framers |
| `simport_ros` | The device node, ROS sources, `simport_check`, the `Behavior` plugin interface and the `simport` Python module |
| `simport_msgs` | `Record`, `Frame` and the `Exchange` service |
| `simport_examples` | Real example devices, one folder each in `devices/` |
| `templates/` | Starting points for users: a device file, a behaviour node, codec and behaviour plugin packages (not built by colcon) |
| `docs/` | The configuration reference and the plugin guide |

## Ground rules

- **simport translates, it does not decide.** simport turns ROS messages into packets and
  packets into ROS messages. What a device decides (answers, modes, registers, state) lives in
  a behaviour: a node serving a receive's service, or a `behavior:` plugin. Never in the core
  packages.
- **The core stays generic.** No device-specific code in `simport_core`, `simport_plugins` or
  `simport_ros`. Device knowledge belongs in device files, behaviours, or separate plugin
  packages.
- **`simport_core` and `simport_plugins` stay ROS-free.** They depend on `simport_core` only,
  so they can be unit-tested without a ROS graph.
- **Errors in device files point at the problem**: file, line, column and path, with a "did you
  mean" for unknown keys. Report them with `Config::error()`.
- **No allocation or name lookup per packet** where practical: bind names to indices in
  `configure()`.

## Adding an example device

An example is a folder in `simport_examples/devices/<name>/`:

```
devices/<name>/
  <name>.yaml        the device file (several, if the device has several ports)
  behavior.py        optional: the node answering the driver (defaults match the yaml)
  *.cpp, *.hpp       optional: an in-process behavior plugin, built by CMakeLists.txt
  data/              optional: files the behaviour serves
  test_*.py / .cpp   unit tests of the behaviour's protocol logic
```

`ros2 launch simport_examples example.launch.py example:=<name>` picks it up without further
registration. Examples are real devices: add one only together with its unmodified driver, and
say in the pull request how you checked the driver's output against the simulator (for
instance in Gazebo: scan points against the simulated world, joint states against the
simulated joints).

## Workflow

1. For anything beyond a small fix, open an issue first, so the approach can be agreed.
2. Build and test:
   ```bash
   colcon build --packages-up-to <pkg>
   colcon test --packages-select <pkg> && colcon test-result --verbose
   ```
   Check every device file you touch with `ros2 run simport_ros simport_check <file.yaml>`.
3. Format C++ with uncrustify (ROS style): `ament_uncrustify --reformat <files>`. Python
   follows flake8 and pep257 (`ament_flake8`, `ament_pep257`); `colcon test` runs all linters.
4. Every source file carries the Apache-2.0 header used in the existing files.
5. New behaviour comes with tests. Codecs need golden-byte tests (expected packets byte by
   byte, ideally from the device's documentation).
6. Update the docs (`README.md`, `docs/`) and the package's `CHANGELOG.rst` with the change.

## Writing a plugin

See [docs/writing_plugins.md](docs/writing_plugins.md), and the packages in
[templates/codec_plugin](templates/codec_plugin) and
[templates/behavior_plugin](templates/behavior_plugin).

## Releases

Versions follow [semantic versioning](https://semver.org). All packages share one version:
bump it in every `package.xml`, turn each `CHANGELOG.rst`'s unreleased entries into a dated
section, and tag the release `vX.Y.Z`.
