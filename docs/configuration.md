# Device configuration reference

A simport device is one YAML file: an emulated port, and the packets that cross it. Check a
file, and see the exact bytes it produces, with:

```bash
ros2 run simport_ros simport_check my_device.yaml
```

Errors point at the problem as `file:line:column: path: message`, and unknown keys get a
"did you mean" suggestion. Relative paths resolve against the file that mentions them;
`package://<pkg>/<path>` and `~/` work too. Numbers may be written in hex (`0x1F`).

The quickest start is the commented template, [templates/device.yaml](../templates/device.yaml).

- [How a device works](#how-a-device-works)
- [File structure](#file-structure)
- [port](#port)
- [packet: the envelope](#packet-the-envelope) and [framer](#framer)
- [send](#send) and [receive](#receive)
- [Fields](#fields): types, constants, counters, checksums, lengths, bits, arrays
- [Where values come from](#where-values-come-from): `from`, builtins, transforms
- [Text packets](#text-packets) and [plugin codecs](#plugin-codecs)
- [Checksums](#checksums)
- [Device behavior](#device-behavior): answering the driver, from a node or in process
- [Clock](#clock) and [running a device](#running-a-device)

## How a device works

simport translates between ROS and the bytes on a port. It does not decide anything:

```
ROS topic ──▶ send ──▶ [ port ] ──▶ driver              ROS data becomes packets
driver ──▶ [ port ] ──▶ receive ──▶ ROS topic            packets become ROS messages
                                └─▶ ROS service ──▶ its answer becomes packets
```

A `send` turns messages from a topic into packets: it maps message fields into the packet,
adds the constants, counters, lengths and checksums, and paces the packets to the port's line
rate. A `receive` decodes the driver's packets and publishes them, or calls a service and
sends back whatever the service answers. Anything a device decides, such as which reply to
give, when to start streaming or what a register holds, belongs in a ROS node that serves that
service, or in a plugin loaded into the device (see [Device behavior](#device-behavior)).

## File structure

```yaml
device: imu0          # name: letters, digits and _ (logs, diagnostics, node name)
port: { ... }         # the emulated port
packet: { ... }       # optional: the envelope every packet shares
framer: { ... }       # optional: how to cut a byte stream into received packets
behavior: { ... }     # optional: a plugin answering receives in process
send: [ ... ]         # packets to the driver
receive: [ ... ]      # packets from the driver
clock: { ... }        # optional; default: the node's ROS clock
```

A device needs at least one `send` or `receive`. Message names are unique within a device.

## port

The key names the kind of port.

### Serial: `serial`

```yaml
port: { serial: /tmp/simport/imu0, baud: 115200 }
```

| Key | Default | Meaning |
|---|---|---|
| `serial` | required | The path the driver opens: a symlink to a pseudo-terminal. Parent directories are created. |
| `baud` | `115200` | Line rate. Sent packets are paced to it, and dropped when they don't fit. |
| `framing` | `8N1` | Data bits (5–8), parity (`N`/`E`/`O`), stop bits (1/2). |
| `throttle` | `true` | `false`: ignore the line rate. |
| `max_delay` | `0.1` | Seconds a packet may wait for the line before it is dropped. |

Like real hardware, packets sent while no driver has the port open are dropped (a driver never
receives stale data), and a driver that sets a different baud rate or framing gets a warning.

### UDP: `udp`

```yaml
port: { udp: 127.0.0.2:2369, to: 127.0.0.1:2368 }
```

| Key | Default | Meaning |
|---|---|---|
| `udp` | `0.0.0.0:0` | The device's own address. Any `127.x.y.z` works on Linux without setup, which helps drivers that check the sensor's IP. |
| `to` | none | Where sent packets go: `host:port`, `{ host, port }`, or `sender` (whoever sent the last datagram). Without it, only replies are sent. |
| `broadcast` | `false` | Allow a broadcast `to` address. |
| `bps`, `max_delay` | none, `0.1` | Pace sent packets at this line rate (e.g. a lidar's 640 packets/s). |

Each datagram is one packet. Replies go back to the sender of the request.

### TCP: `tcp`

```yaml
port: { tcp: 127.0.0.5:8080 }
```

| Key | Default | Meaning |
|---|---|---|
| `tcp` | required | The device's address and port. |
| `max_clients` | `8` | Further connections are refused. |
| `bps`, `max_delay` | none, `0.1` | Optional line-rate model. |

Each client is a separate byte stream, framed separately; replies go to the client that asked,
other packets to every client. Ports below 1024 need privileges: prefer a higher port when the
driver accepts `host:port`, or allow them once with
`sudo sysctl net.ipv4.ip_unprivileged_port_start=80`.

### CAN: `can`

```yaml
port: { can: vcan0, fd: true }
```

| Key | Default | Meaning |
|---|---|---|
| `can` | required | SocketCAN interface, normally virtual: `sudo modprobe vcan && sudo ip link add dev vcan0 type vcan mtu 72 && sudo ip link set up vcan0` (mtu 72 allows CAN-FD). |
| `fd` | `false` | Also receive and send CAN-FD frames. |
| `bps`, `max_delay` | none, `0.1` | Optional bus-rate model. |

Each CAN frame is one packet. Its identifier is the field `can_id: meta`; replies are sent in
the request's format (CAN or CAN-FD).

### Your own: `plugin`

`port: { plugin: my_pkg/MyPort, ... }` loads a [Transport plugin](writing_plugins.md).

A device with several ports, such as a lidar with an HTTP API and separate UDP data streams,
is several device files at the same address (see `simport_examples/devices/ouster_os1_64/`).

## packet: the envelope

Many protocols wrap every message the same way: start bytes, an id, a length, a checksum.
Describe that once in `packet:`; each message's `fields` are then only its body.

```yaml
packet:
  endian: little
  start: [0xFF, 0xFF, 0xFD, 0x00]                  # every packet starts with these bytes
  fields:
    - id: u8
    - length: { u16, length_of: [body, crc] }      # filled in
    - body                                         # the message's fields go here
    - crc: { u16, checksum: crc16_buypass }        # filled in
  stuffing: { after: [0xFF, 0xFF, 0xFD], insert: 0xFD }   # optional, applied to the body
```

Envelope fields such as `id` are fields of every message: a service answer sets them, and a
received packet reports them. `packet:` may also name a file (`packet: dynamixel.yaml`), to
share an envelope between devices.

When sending, simport builds the body, stuffs it, adds the envelope, then fills in the lengths
and the checksums. When receiving, it checks the checksums and the lengths, unstuffs the body,
and decodes it.

## framer

On a serial or TCP port, the driver's bytes arrive as a stream. With an envelope that has
`start` bytes and a length before the body, simport cuts the stream into packets by itself.
Otherwise, give a framer:

| Framer | Keys | Packets are... |
|---|---|---|
| `simport/SyncLength` | `sync: [bytes]`, `length: { offset, type, endian }`, `adjust`; or `size` | `sync`, then `length + adjust` bytes in all (or a fixed `size`) |
| | `cases: [{ when: { offset, mask, equals }, size \| length + adjust }]` | the first case whose header byte matches decides |
| `simport/FixedLength` | `size` | always `size` bytes |
| `simport/Delimiter` | `delimiter`, `include_delimiter` (true), `max_size` | text lines and the like |
| `simport/Http` | `max_size` | HTTP requests |

UDP and CAN deliver whole packets and need no framer.

## send

Packets to the driver.

```yaml
send:
  - name: imu_data
    from: { topic: /imu, type: sensor_msgs/msg/Imu }
    rate: 200
    fields:
      - sync: { u16: 0xAA55, endian: big }
      - gx:   { i16, from: angular_velocity.x, scale: 1000.0 }
      - crc:  { u16, checksum: crc16_ccitt_false }
```

| Key | Meaning |
|---|---|
| `name` | Required. |
| `from` | Where the data comes from: `{ topic, type }` (see below). Without it, the message is only sent as a service's answer. |
| `rate` | Hz: send the latest data at the device's own rate, like a sensor's internal timer. Without it, one packet per message. |
| `stale_after` | With `rate`: seconds without new data before the device goes silent (default `1.0`; `0` = never). |
| `stream` | `false`: keep the latest data from `from`, but send it only as a service's answer (a polled device). |
| `fields`, `endian`, `frames` | The packet, see [Fields](#fields). |
| `format`, `checksum` | Or a [text packet](#text-packets). |
| `codec` | Or a [plugin codec](#plugin-codecs). |

### `from`

| Key | Default | Meaning |
|---|---|---|
| `topic` | required | Topic name. |
| `type` | required | e.g. `sensor_msgs/msg/Imu`. Any message type works, without code generation. |
| `reliability` | `best_effort` | `best_effort` (matches any publisher) or `reliable`. |
| `depth` | `10` | Queue depth. |
| `stamp` | `auto` | The data timestamp (`$stamp_*`): `header`, `receive`, or `auto` (the header's if there is one). |

With `type: sensor_msgs/msg/PointCloud2`, the cloud is read as arrays, one element per point
in storage order (row by row for organized clouds), for 3-D lidars. Paths are point field
names (`x`, `intensity`, `ring`, ...), or `range`, `azimuth` and `elevation` computed from x,
y and z. Gazebo's `gpu_lidar` publishes such a cloud; bridge it rather than its LaserScan,
which `ros_gz_bridge` reduces to one row.

`from: { plugin: my_pkg/MySource, ... }` loads a [Source plugin](writing_plugins.md).

## receive

Packets from the driver. Each packet goes to the first `receive` whose fields decode it (the
length fits, constants and checksums match) and whose `match` holds; packets nothing claims
are counted in the diagnostics.

```yaml
receive:
  - name: command
    fields: [cmd: u8, arg: u8]
    match: { cmd: [0x10, 0x11] }
    to: { service: /my_sensor/command }
    reply: ack
```

| Key | Meaning |
|---|---|
| `name` | Required. |
| `fields`, `endian` / `format`, `checksum` / `codec` | The packet, as for `send`. |
| `match` | `{ field: value }` or `{ field: [values] }`: decoded fields must have one of the values. |
| `to` | `{ topic: name }`: publish each packet as a `simport_msgs/msg/Record` (default `~/<name>`). `{ service: name }`: call a `simport_msgs/srv/Exchange` service. `behavior`: hand it to the device's `behavior:` plugin. |
| `reply` | With a service: the `send` message, or list of them, the service's answers are sent as. |

## Fields

A message's `fields` are listed in wire order, one per line:

```yaml
fields:
  - seq: u8                                     # name: type
  - version: { u8: 0x02 }                       # a constant
  - gyro_z: { i16, from: angular_velocity.z, scale: 1000 }   # type and options
```

Types: `u8 u16 u32 u64 i8 i16 i32 i64 f32 f64 bool`, and `bytes`. Values that do not fit
saturate (clip), unless the field has `overflow: wrap`.

| Option | Meaning |
|---|---|
| `{ u8: 7 }` or `{ u8, value: 7 }` | A constant. Received packets must have it. |
| `endian: big \| little` | Byte order of this field (default: the message's `endian`, then the envelope's, then `little`). |
| `overflow: wrap` | Keep the low bits instead of clipping (timestamps that wrap). |
| `counter: true` or `{ start, step, per: packet \| record }` | Counts up by itself, wraps. |
| `checksum: <name>` | Filled in; see [Checksums](#checksums). Covers everything before it, or `over: first` (from `first` to the field before the checksum), or `over: [first, last]`. |
| `length_of: name` or `[names]` | Filled in: the byte size of those fields (`body`: the whole message). |
| `bits: [name: width, ...]` | Bit fields, least significant first. Each is `name: width` or `name: { width, value, overflow, from, ..., length_of }`. |
| `from`, `scale`, ... | Where the value comes from; see [below](#where-values-come-from). |

Other entries:

| Entry | Meaning |
|---|---|
| `pad: 2` | Two bytes, sent as zero and ignored when received. |
| `data: bytes` | The rest of the body, any length (one per message). |
| `magic: { bytes: [0xFF, 0xFF] }` | Constant bytes. |
| `ids: { u8, repeat: rest }` | Repeated until the body ends: an array. |
| `can_id: meta` | Not on the wire: the CAN identifier of the packet. |
| `group: { count: N, fields: [...] }` | Fields repeated N times: their values are arrays. Groups nest. |

### Arrays and several packets per message

Fields inside a `group` are arrays: element *k* fills repetition *k*. With `frames: N` a
message becomes N packets, the arrays continuing from one packet to the next; with
`frames: auto`, as many packets as the longest array needs. Computed values help:

| Option | Value |
|---|---|
| `frame_index: true` or `{ scale, offset }` | The packet's number within the message. |
| `element_index: true` or `{ scale, offset }` | The element's index in the array (inside a group). |
| `first_element: [a, b]` | `a` on the first element of the message, `b` on the others. |

```yaml
# RPLIDAR samples: one 5-byte packet per element of the ranges.
frames: auto
fields:
  - group:
      count: 1
      fields:
        - flags: { u8, bits: [start: { width: 1, first_element: [1, 0] },
                              not_start: { width: 1, first_element: [0, 1] },
                              quality: { width: 6, value: 47 }] }
        - angle: { u16, bits: [check: { width: 1, value: 1 },
                               angle_q6: { width: 15, element_index: { scale: 64.178273 } }] }
        - dist_q2: { u16, from: ranges, reverse: true, resize: 360, scale: 4000.0, nonfinite: 0 }
```

See `simport_examples/devices/ouster_os1_64/lidar.yaml` for nested groups: columns of beams,
16 columns per packet.

## Where values come from

In a `send`, a field with `from:` takes its value from the message on the topic:

| `from` | Value |
|---|---|
| `angular_velocity.x`, `ranges`, `ranges[3]`, `status.status` | A message field (a path). |
| `$time_ns`, `$time_us`, `$time_ms`, `$time_s` | The device clock (simulation time with `use_sim_time`). |
| `$stamp_ns`, ..., `$stamp_s` | The data's timestamp (see `stamp` under `from`). |
| `$wall_time_ns`, ..., `$wall_time_s` | The computer's real time (e.g. GPS UTC). |

Transforms, applied in this order:

| Key | Meaning |
|---|---|
| `transpose: { rows: N }` | Arrays: read a row-major N-row image column by column. |
| `reverse: true` | Arrays: reverse the order. |
| `step: N` | Arrays: keep every N-th element. |
| `resize: N` | Arrays: pick N evenly spaced elements. |
| `scale`, `offset` | `value * scale + offset`. |
| `nonfinite: x` | Replace inf and NaN with `x`. |
| `ramp: { step, count }` | Make an array from a number: `v, v + step, ...` |

Fields without `from` keep their default (0) unless a service's answer sets them; `simport_check`
shows where each field gets its value.

## Text packets

For ASCII protocols, give a `format` instead of `fields`:

```yaml
- name: gga
  from: { topic: /gps/fix, type: sensor_msgs/msg/NavSatFix }
  rate: 5
  format: "${$begin}GPGGA,{utc:hhmmss.ss},{lat:nmea_lat.5},{lon:nmea_lon.5},1,12,0.9,\
    {alt:%.1f},M,0.0,M,,{$end}*{$checksum}\r\n"
  checksum: nmea
  fields:
    utc: $wall_time_s
    lat: latitude
    lon: longitude
    alt: altitude
```

`fields` says where each placeholder's value comes from: `name: path` or
`name: { from: ..., scale: ... }`.

| Placeholder | Meaning |
|---|---|
| `{name}` / `{name:format}` | A value (default format `%g`). A name may appear more than once. |
| `{$begin}` / `{$end}` | Bounds of the checksummed text (defaults: start of the packet, and the checksum). |
| `{$checksum}` / `{$checksum:format}` | The checksum (default `%02X`). Needs `checksum:`. |
| `{{` / `}}` | Literal braces. |

| Format | Value |
|---|---|
| `%d %i %u %x %X %o %f %e %g %s` with flags, width, precision | printf-style |
| `nmea_lat`, `nmea_lon` (`.N`: N decimals of minutes, default 4) | degrees → `ddmm.mmmm,N` / `dddmm.mmmm,E` |
| `hhmmss`, `hhmmss.ss`, ... | seconds (Unix, or since midnight) → UTC time of day |
| `ddmmyy` | Unix seconds → UTC date |

A `receive` with a format checks the literal text and splits values at the next literal; empty
values (NMEA `,,`) count as absent.

## Plugin codecs

When neither `fields` nor `format` can describe a packet, a codec plugin can:

```yaml
codec: { plugin: my_pkg/MyCodec, ... }
```

simport ships `simport/HttpCodec`, for web APIs on a `tcp` port:

```yaml
framer: { plugin: simport/Http }
send:
  - name: response                  # status and body come from the service's answer
    codec: { plugin: simport/HttpCodec, content_type: application/json }
receive:
  - name: request                   # method, path, query and body
    codec: { plugin: simport/HttpCodec }
    to: { service: /my_sensor/http }
    reply: response
```

`HttpCodec` keys: `status` (default 200), `content_type`, `body` or `body_file` (defaults for
answers that set none), `headers`. To write one, see
[templates/codec_plugin](../templates/codec_plugin) and [Writing a plugin](writing_plugins.md).

## Checksums

`checksum: <name>`, or a custom CRC: `checksum: { algo: crc, width, poly, init, refin, refout,
xorout }` (width 8–64, a multiple of 8).

| Name | Notes |
|---|---|
| `xor8` / `nmea` | XOR of the bytes |
| `sum8`, `sum16` | Sum, truncated |
| `lrc8` | Two's complement of the 8-bit sum (Modbus ASCII) |
| `fletcher8` | u-blox UBX: CK_A in the high byte, CK_B in the low byte (a big-endian u16) |
| `crc8`, `crc8_maxim` | |
| `crc16_ccitt_false`, `crc16_xmodem`, `crc16_kermit`, `crc16_modbus`, `crc16_arc`, `crc16_buypass` | `crc16_buypass`: Dynamixel Protocol 2.0 |
| `crc32`, `crc32c`, `crc64_xz` | `crc64_xz`: CRC-64/ECMA-182, as in Ouster lidar packets |

## Device behavior

What a device decides, simport leaves to you, in one of two places:

| | A node serving a service | A behavior plugin |
|---|---|---|
| Receive | `to: { service: /my_sensor/command }` | `to: behavior` |
| Runs | in its own process, any language | inside the device's process, C++ |
| Per packet | a service round trip: about 1–3 ms | a function call: microseconds |
| Use for | sensors: start/stop, polls, registers, web APIs | fast control loops, e.g. a motor bus answering thousands of packets per second |

Both see the same thing: the decoded packet in, the packets to send back out. An answer goes
back the way the request came: to the same UDP sender or TCP client, in the same kind of CAN
frame.

### A node serving a service

A `receive` with `to: { service: ... }` calls a `simport_msgs/srv/Exchange` service:

```
simport_msgs/Record request     # the decoded packet
---
simport_msgs/Record[] replies   # packets to send back: none, one or several
```

A `Record` is the packet's fields by name (`name` says which `receive` it came from). Each
reply is encoded with the receive's `reply` message named by the record's `name` (the first
one if the name is empty); its fields override the message's defaults or latest data, and
constants, counters, lengths and checksums are filled in.

In Python, the `simport` module (installed with `simport_ros`) reads and writes records:

```python
from simport import fields_of, make_record, run_node, serve

serve(self, '/my_sensor/command', self.on_command)   # in the node's __init__

def on_command(self, request, response):
    command = fields_of(request.request)               # {'cmd': 1, 'arg': 42, ...}
    response.replies.append(make_record('ack', cmd=command['cmd'], arg=command['arg']))
    return response
```

Use `serve()` rather than `create_service()`: its request queue is deep enough for a driver
that sends many packets at once. [templates/device_node.py](../templates/device_node.py) is a
complete node to start from. The `behavior.py` of each example in `simport_examples/devices/`
shows a common pattern: streaming only between SCAN and STOP by publishing the topic a `send`
reads (`rplidar_a1`), a register map (`bno055`), a web API (`ouster_os1_64`), and motors with
control tables driving simulated joints (`dynamixel`).

### A behavior plugin

```yaml
behavior:
  plugin: simport_examples/DamiaoBus      # the plugin's own keys follow
  motors:
    - { id: 0x01, master_id: 0x11, type: DM8009, joint: openarm_joint1 }
receive:
  - name: mit
    fields: [...]
    to: behavior
```

The plugin (a `simport::Behavior`, see [Writing a plugin](writing_plugins.md)) gets every
packet of the receives with `to: behavior` and appends its answers: a send message and the
field values to set. It may use the device's ROS node, e.g. to read and command simulated
joints. `simport_examples/DamiaoBus` drives Damiao motors (see
`simport_examples/devices/damiao/`; it keeps up with OpenArm's 750 Hz loop, 10,500 packets per
second, with replies in under a millisecond). [templates/behavior_plugin](../templates/behavior_plugin) is a package to start
from.

## Clock

| `clock:` | Time |
|---|---|
| (default) | The node's ROS clock: simulation time with `use_sim_time:=true`. `/clock` steps are smoothed using the measured real-time factor, so fixed-rate sends stay even. |
| `{ plugin: simport/SystemClock }` | Wall-clock time. |

## Running a device

```bash
ros2 launch simport_ros device.launch.py config_file:=my_device.yaml name:=imu0 use_sim_time:=true
ros2 run simport_ros simport_device --ros-args -p config_file:=my_device.yaml
```

| Parameter | Default | Meaning |
|---|---|---|
| `config_file` | required | The device file. |
| `autostart` | `true` | Configure and activate on startup; otherwise drive the lifecycle yourself. |
| `debug_frames` | `false` | Publish every packet, both directions, on `~/frames` (`simport_msgs/Frame`). |
| `use_sim_time` | `false` | Use simulation time. |

The device is a lifecycle node: **configure** creates the port, **activate** starts data flow,
**deactivate** silences the device while the port stays (like a sensor fault), and **cleanup**
removes the port, as if the device were unplugged. Status of the port and of each `send` and
`receive` is published on `/diagnostics`.
