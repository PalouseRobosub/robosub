# A more detailed message for battery state

# Power status consts
uint8 POWER_SUPPLY_STATUS_UNKNOWN      = 0
uint8 POWER_SUPPLY_STATUS_CHARGING     = 1
uint8 POWER_SUPPLY_STATUS_DISCHARGING  = 2
uint8 POWER_SUPPLY_STATUS_NOT_CHARGING = 3
uint8 POWER_SUPPLY_STATUS_FULL         = 4
uint8 POWER_SUPPLY_STATUS_OVERLOAD     = 5
uint8 POWER_SUPPLY_STATUS_ON_LINE      = 6
uint8 POWER_SUPPLY_STATUS_ON_BATTERY   = 7
uint8 POWER_SUPPLY_STATUS_LOW_BATTERY  = 8
uint8 POWER_SUPPLY_STATUS_HIGH_BATTERY = 9
uint8 POWER_SUPPLY_STATUS_REPLACE      = 10

# Message Variables
float32   voltageBattery
float32   currentBattery
float32   charge
float32   capacity
time      runtime
float32   percentage
int32[]   status
float32   temperature
bool      alive
float32   voltageInput
float32   currentInput
float32   voltageOutput
float32   currentOutput
float32[] cellVoltages
