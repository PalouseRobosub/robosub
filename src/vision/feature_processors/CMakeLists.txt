add_library(feature_processors CentroidDetector.cpp
                               StartGateDetector.cpp
                               BuoyDetector.cpp
                               ChannelDetector.cpp
                               MarkerBinDetector.cpp
                               TorpedoTargetDetector.cpp
                               OctagonDetector.cpp)

add_dependencies(feature_processors ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                    ${catkin_EXPORTED_TARGETS})
