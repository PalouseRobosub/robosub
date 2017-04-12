add_library(vision_filters CloseFilter.cpp
                           ConvertFilter.cpp
                           InRangeFilter.cpp
                           MedianBlurFilter.cpp
                           OpenFilter.cpp
                           OrFilter.cpp

                           FilterSet.cpp
)

add_dependencies(vision_filters ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                ${catkin_EXPORTED_TARGETS})
