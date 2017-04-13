add_library(feature_detectors CentroidDetector.cpp
                              StartGateDetector.cpp
                              BuoyDetector.cpp
                              ChannelDetector.cpp
                              MarkerBinDetector.cpp
                              TorpedoTargetDetector.cpp
                              OctagonDetector.cpp)

add_dependencies(feature_detectors ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                   ${catkin_EXPORTED_TARGETS})
