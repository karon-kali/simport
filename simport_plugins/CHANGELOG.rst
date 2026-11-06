^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package simport_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-11-06)
------------------
* BinaryCodec rewritten for the field syntax (``name: type`` or ``name: { type, ... }``): packet
  envelopes with ``body``, ``length_of`` (also in bit fields), byte stuffing, variable-length
  ``bytes`` and ``repeat: rest``, ``can_id: meta``, checksum ranges with ``over``, bit fields,
  nested groups of array fields, several packets per message (``frames: N | auto``), computed
  frame and element indices, per-record counters
* SyncLength: framing derived from a packet envelope, and ``cases`` by header byte
* New ports: TcpServer, SocketCan (CAN and CAN-FD); ports named by kind (``serial``,
  ``udp``/``to``, ``tcp``, ``can``)
* Http framer and HttpCodec for device web APIs
* TextCodec takes placeholder sources from ``fields``
* Optimized builds by default

0.1.0 (2026-11-05)
------------------
* First release.
* Transports: SerialPty (line-rate model, hot-plug, baud mismatch warning), Udp
* Codecs: BinaryCodec (counters, checksums, wrap/saturate), TextCodec (printf and NMEA formats)
* Framers: FixedLength, Delimiter, SyncLength
* Triggers: OnMessage, FixedRate, OnRequest
* Filters: Bandwidth
* Clocks: SystemClock
