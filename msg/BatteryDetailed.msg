# A more detailed message for battery state

# Power status consts
uint8 POWER_SUPPLY_STATUS_UNKNOWN      = 0
uint8 POWER_SUPPLY_STATUS_CHARGING     = 1
uint8 POWER_SUPPLY_STATUS_DISCHARGING  = 2
uint8 POWER_SUPPLY_STATUS_NOT_CHARGING = 3
uint8 POWER_SUPPLY_STATUS_FULL         = 4

# Battery Chemistry
uint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0
uint8 POWER_SUPPLY_TECHNOLOGY_NIMH    = 1
uint8 POWER_SUPPLY_TECHNOLOGY_LION    = 2
uint8 POWER_SUPPLY_TECHNOLOGY_LIPO    = 3
uint8 POWER_SUPPLY_TECHNOLOGY_LIFE    = 4
uint8 POWER_SUPPLY_TECHNOLOGY_NICD    = 5
uint8 POWER_SUPPLY_TECHNOLOGY_LIMN    = 6


# Message Variables
float32 voltage
float32 current
float32 charge
float32 capacity
time    runtime
float32 percentage
uint8   status
uint8   chemistry
float32 temperature
string  name
bool    alive
float32 voltageInput
float32 currentInput
