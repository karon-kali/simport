^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package simport_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-11-06)
------------------
* Device format: ``port``, an optional ``packet`` envelope, ``send`` and ``receive``
  messages. A receive publishes to a topic, calls a service whose answers are sent back
  (``reply``: one send message or a list), or hands packets to an in-process ``behavior``.
  ``stream: false`` sends a topic's latest data only as an answer; ``match`` takes lists
* simport translates and does not decide: removed input actions, output enable/disable and
  the ``map`` block (values now come from the fields' ``from``)
* Answers go back the way the request came: to its sender, in the same kind of frame
* Mapping: array selection (``reverse``, ``step``, ``resize``), ``transpose``, ``ramp``,
  ``nonfinite``, and single values filling array fields
* Inbound router: one framer per peer of a multi-client stream port
* Checksums: ``crc16_buypass`` (Dynamixel Protocol 2.0), ``crc64_xz`` (Ouster)
* Optimized builds by default

0.1.0 (2026-11-05)
------------------
* First release.
* Data model: Value, Schema, Record, Frame
* YAML configuration with file:line:column errors and key suggestions
* Plugin interfaces: Transport, Codec, Framer, Trigger, FrameFilter, Clock
* Device configuration: outputs, inputs, match conditions and actions
* Field mapping with message paths, constants, scale/offset and time builtins
* OutputChannel (gating, on-demand replies), InputChannel and InboundRouter
* Checksum library: CRC presets and custom CRCs, sums, XOR, Fletcher
* Simulation time extrapolation
