cmake_minimum_required(VERSION 2.8)
ADD_DEFINITIONS(
  -Wall
  -O2
  -DdDOUBLE
  -DdTRIMESH_ENABLED
)
project( nachi_apple_catch )
find_package( OpenCV REQUIRED )

include_directories("/usr/include/X11/")
include_directories("/usr/local/include/ode/")
include_directories("/usr/local/include/drawstuff/")

link_directories("/usr/lib/X11/")

add_executable( nachi_apple_catch nachi.cpp draw_arms.cpp)
target_link_libraries( nachi_apple_catch ${OpenCV_LIBS} ode drawstuff GL GLU Xmu X11 stdc++ pthread m)
