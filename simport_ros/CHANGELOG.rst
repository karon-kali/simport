^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package simport_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-11-06)
------------------
* Device for the send/receive format: receives publish ``simport_msgs/Record``, call
  ``simport_msgs/srv/Exchange`` and send the answers, or hand packets to a behavior plugin
* Behavior plugin interface (``simport::Behavior``): a device's answers computed in process
* ``simport`` Python module for behaviour nodes: ``serve``, ``fields_of``, ``make_record``,
  ``run_node``
* PointCloud2Source: point fields, range, azimuth and elevation as arrays
* RosTopicSource: array sources and transforms
* simport_check: sample packets for every send message
* Removed the Sink interface and RecordTopicSink (receives publish records themselves)
* Optimized builds by default

0.1.0 (2026-11-05)
------------------
* First release.
* Device lifecycle node and component, with diagnostics and debug frame topics
* RosTopicSource for any message type via rosidl introspection
* RecordTopicSink, RosClock
* simport_check: validate a device file and print its wire layout
