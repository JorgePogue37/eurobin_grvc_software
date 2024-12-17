cmake_minimum_required(VERSION 2.8)

project( VisualFeedback_Display_Detect )

find_package( OpenCV REQUIRED )

add_executable( VisualFeedback_Display_Detect Main.cpp )

target_link_libraries( VisualFeedback_Display_Detect ${OpenCV_LIBS} -pthread )
