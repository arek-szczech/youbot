cmake_minimum_required(VERSION 2.8)

ADD_EXECUTABLE(displayIpAddress
  DisplayIpAddress.cpp

)

target_link_libraries(displayIpAddress stdc++)

INSTALL(TARGETS displayIpAddress
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

