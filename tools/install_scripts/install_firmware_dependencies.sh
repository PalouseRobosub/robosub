# Sets up firmware projects with PlatformIO.
#
# Note: You must previously have installed Platformio through:
#       sudo pip install platformio

robosub_base="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../../" && pwd )"
cd $robosub_base/firmware/depth

### Depth sensor project
# Install rosserial_arduino and MS5837 sensor libraries.
platformio lib install 345 808

# Finally, make the microcontroller header files.
rsmake firmware
