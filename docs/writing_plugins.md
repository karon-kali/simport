# Writing a simport plugin

Most devices need no code: a device file describes the packets, and a small ROS node decides
what the device does (see [Device nodes](configuration.md#device-nodes)). Write a plugin when
the YAML cannot describe something: a packet format with a proprietary check, a new kind of
port, a different way to cut a byte stream into packets. Plugins live in your own package and
are loaded by name with `pluginlib`; simport does not need to change.

| Interface | Header | Base package | Used in a device file as |
|---|---|---|---|
| `simport::Codec` | `simport_core/codec.hpp` | `simport_core` | `codec: { plugin: ... }` in a `send` or `receive` |
| `simport::Transport` | `simport_core/transport.hpp` | `simport_core` | `port: { plugin: ... }` |
| `simport::Framer` | `simport_core/framer.hpp` | `simport_core` | `framer: { plugin: ... }` |
| `simport::Clock` | `simport_core/clock.hpp` | `simport_core` | `clock: { plugin: ... }` |
| `simport::Source` | `simport_ros/source.hpp` | `simport_ros` | `from: { plugin: ... }` in a `send` |
| `simport::Behavior` | `simport_ros/behavior.hpp` | `simport_ros` | `behavior: { plugin: ... }`, with `to: behavior` in a `receive` |

Plugins on `simport_core` interfaces have no ROS dependency and can be unit-tested without a
ROS graph. The header comments are the reference for each interface's lifecycle and threading
contract.

## Start from a template

- [templates/codec_plugin](../templates/codec_plugin): a codec, for a 4-byte packet with a
  proprietary check byte.
- [templates/behavior_plugin](../templates/behavior_plugin): a behavior that answers
  `templates/device.yaml`'s commands in process.
  `simport_examples/devices/damiao/damiao_bus.cpp` is a complete one for a motor bus.

Both build as is:

```bash
cp -r templates/codec_plugin ~/my_ws/src/my_simport_codec
cd ~/my_ws && colcon build --packages-select my_simport_codec && source install/setup.bash
ros2 run simport_ros simport_check src/my_simport_codec/example.yaml
```

Then rename the package (in `package.xml`, `CMakeLists.txt` and `plugins.xml`), the class and
the fields, and replace the bytes or the logic with your device's.

## The parts of a behavior

```cpp
class MyBehavior : public simport::Behavior
{
public:
  // Read your keys; bind message and field names to indices with messages.send(...),
  // messages.receive(...) and schema->find(...). Create ROS entities on `node` if needed.
  void configure(const simport::Config & config, simport::Context & context,
    rclcpp_lifecycle::LifecycleNode & node, const simport::DeviceMessages & messages) override;

  // A decoded packet of a receive with `to: behavior`. Append the packets to send back.
  // Runs on the port's receive thread; guard state shared with ROS callbacks.
  void handle(std::size_t receive, const simport::Record & packet,
    std::vector<simport::Answer> & answers) override;
};
```

Register it with `pluginlib_export_plugin_description_file(simport_ros plugins.xml)` and link
`simport_ros::simport_ros`.

## The parts of a codec

```cpp
class MyCodec : public simport::Codec
{
public:
  // `message` is the `codec:` block; `envelope` the device's `packet:` (usually undefined
  // for a plugin codec). Validate every key and build the schema here.
  void configure(const simport::Config & message, const simport::Config & envelope,
    simport::Context & context) override;

  // Where input fields get their values: (field name, description with `from:`, `scale:`...).
  // simport builds the mapping from ROS messages from these.
  std::vector<std::pair<std::string, simport::Config>> sources() const override;

  // The packet's fields: kInput fields are filled from ROS (or a service's answer);
  // kComputed fields (constants, checksums) are the codec's own.
  simport::SchemaPtr schema() const override;

  void encode(const simport::Record & record, std::vector<simport::Frame> & out) override;

  // Optional: to use the codec in a `receive`.
  bool can_decode() const override {return true;}
  simport::DecodeResult decode(const simport::Frame & frame, simport::Record & out) override;
};

PLUGINLIB_EXPORT_CLASS(my_simport_codec::MyCodec, simport::Codec)
```

`decode` reports why a packet is not its own: `kBadLength`, `kNoMatch` (a constant differs,
probably another packet type), `kBadChecksum`. simport then offers the packet to the next
`receive`.

**plugins.xml**: the `name` is what device files use.

```xml
<library path="my_simport_codec">
  <class name="my_simport_codec/MyCodec" type="my_simport_codec::MyCodec"
         base_class_type="simport::Codec">
    <description>...</description>
  </class>
</library>
```

**CMakeLists.txt**: link `simport_core::simport_core` and `pluginlib::pluginlib`, and register
the plugins with the base package:
`pluginlib_export_plugin_description_file(simport_core plugins.xml)`.

## Guidelines

- **Configuration.** Read only your own keys and call `config.expect_keys({...})` first.
  Report problems with `config.error(...)` or `config["key"].error(...)`, which adds the file,
  line, column and path. Use `config.resolve_path(...)` for file references.
- **Hot path.** Bind names to indices in `configure()`. Avoid lookups by name and logging per
  packet; count instead, and log the first occurrence.
- **Values.** Records may hold any numeric kind; convert with `simport::saturate_cast<T>()`.
- **Threading.** Follow the header contract: `Transport::write()` is called from several
  threads; a codec is called by one thread at a time and may keep state (reset it in
  `reset()`).
- **Logging.** Use `context.logger()`; messages are prefixed with the plugin's place in the
  device (`imu0/imu_data/codec`).
- **Time.** Use `context.clock()`, never the system clock directly, so the plugin follows
  simulation time.
- **Tests.** See `simport_plugins/test` for golden-byte codec tests and real PTY, UDP and TCP
  transport tests.
