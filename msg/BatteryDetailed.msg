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
bool      alive          # Is the battery attached?
int32[]   status         # Space Seperated Information
float32   voltageBattery # Voltage of the Battery
float32   currentBattery # Current out of the Battery
float32   voltageInput   # Voltage of the Input
float32   currentInput   # Current of the Input
float32   voltageOutput  # Voltage of the Output
float32   currentOutput  # Current of the Output
float32   temperature    # Temperature Reading near the Battery
float32   charge         # Percent charged
float32   capacity       # Ah left in the battery
time      runtime        # Time left on the battery
float32[] cellVoltages   # Voltages of individual cells (if available)
