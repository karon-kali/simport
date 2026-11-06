# One field of a decoded Record.

uint8 KIND_BOOL = 0
uint8 KIND_INT = 1
uint8 KIND_UINT = 2
uint8 KIND_FLOAT = 3
uint8 KIND_STRING = 4
uint8 KIND_BYTES = 5
uint8 KIND_FLOAT_ARRAY = 6

string name
uint8 kind
float64 number      # set for BOOL/INT/UINT/FLOAT (may lose precision for 64-bit integers)
string text         # STRING value; for other kinds a human-readable rendering
uint8[] bytes       # BYTES value
float64[] array     # FLOAT_ARRAY value
