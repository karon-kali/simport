# A raw frame as seen on an emulated port. Published for debugging/sniffing.

uint8 DIRECTION_OUT = 0  # simulator -> driver
uint8 DIRECTION_IN = 1   # driver -> simulator

std_msgs/Header header   # stamp: when the frame was written/received (device clock)
string name              # the send message that wrote it, or "received"
uint8 direction
uint8[] data
diagnostic_msgs/KeyValue[] meta  # transport metadata, e.g. endpoint, can_id
