# State machine and switch flags for lama flight mode

uint64 timestamp

# states values
uint8 APPROACH	   = 0
uint8 INTERACTION  = 1
uint8 DISAPPROACH  = 2

uint8 state
bool interaction_enable
bool interaction_output

