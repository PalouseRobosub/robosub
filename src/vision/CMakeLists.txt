add_library(shared_image_reader SharedImageReader.cpp)
target_link_libraries(shared_image_reader ${OpenCV_LIBRARIES})

add_ros_node(image_publisher image_publisher.cpp)
target_link_libraries(image_publisher shared_image_reader)
target_link_libraries(image_publisher ${OpenCV_LIBRARIES})

add_ros_node(vision VisionProcessor.cpp Vision.cpp)
target_link_libraries(vision ${OpenCV_LIBRARIES})
