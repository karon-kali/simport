# The fields of one packet: decoded from the driver (receive), or to encode (send / reply).

std_msgs/Header header
string name           # the receive or send message's name (may be empty when sending)
RecordField[] fields
