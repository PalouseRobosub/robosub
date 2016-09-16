#include "Serial.hpp"

add_executable(thruster thruster.cpp)
target_link_libraries(thruster serial)
