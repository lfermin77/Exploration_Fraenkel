cmake_minimum_required(VERSION 2.8.3)
project(exp_fraenkel)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  nav_msgs
  cv_bridge
  image_transport
)


find_package(CGAL REQUIRED COMPONENTS Core)

find_package(MPFR)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dude_ros
#  CATKIN_DEPENDS tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/Third_Party/dude_final  
  ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/src  
  ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/mathtool
  ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/src/skeleton
  ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/src/draw
  ${CGAL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include  
)
include(${CGAL_USE_FILE})





######## exploration
add_executable(exp_fraenkel src/exp_fraenkel.cpp  include/wrapper.cpp include/inc_decomp.cpp include/graph.cpp )

target_link_libraries(exp_fraenkel
   ${catkin_LIBRARIES}
   ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/libdude2d.so
   ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/FIST/FIST.lib
   ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/FIST/libFIST.linux64.a
   ${CGAL_LIBRARIES} 
   ${CGAL_3RD_PARTY_LIBRARIES}
   ${MPFR_LIBRARIES}
 )


add_dependencies(exp_fraenkel ${catkin_LIBRARIES} ${CGAL_LIBRARIES})






######## tf2pose
add_executable(tf2pose src/tf2position.cpp )

target_link_libraries(tf2pose
   ${catkin_LIBRARIES}
   
 )





