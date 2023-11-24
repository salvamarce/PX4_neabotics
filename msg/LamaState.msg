# State machine and switch flags for lama flight mode

uint64 timestamp

# states values
uint8 IDLE         = 0
uint8 APPROACH	   = 1
uint8 INTERACTION  = 2
uint8 DISAPPROACH  = 3

uint8 state
bool engage_approach
bool engage_interaction

