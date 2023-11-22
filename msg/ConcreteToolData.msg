# Messages from the lama tool
uint64 timestamp

# uint64 timestamp_distance
# uint64 timestamp_force

# Load sensors
float32[3] force
float32[3] torque

# ToF sensors
# sensor indeces
uint8 TOP_LEFT     = 0
uint8 TOP_RIGHT    = 1
uint8 BOTTOM_LEFT  = 2
uint8 BOTTOM_RIGHT = 3

float32[4] distance

# Tool mountig position
# tool position values
uint8 TOOL_FRONT = 0
uint8 TOOL_TOP   = 1

uint8 tool_pos
