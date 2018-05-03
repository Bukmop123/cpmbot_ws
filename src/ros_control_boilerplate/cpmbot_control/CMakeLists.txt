include_directories(
  include/
)

# CpmBot Hardware Interface
add_library(cpmbot_hw_interface
  src/cpmbot_hw_interface.cpp
)
target_link_libraries(cpmbot_hw_interface
  generic_hw_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(cpmbot_hw_main src/cpmbot_hw_main.cpp)
target_link_libraries(cpmbot_hw_main
  cpmbot_hw_interface
  generic_hw_control_loop
  ${catkin_LIBRARIES}
)

## Install ------------------------------------------------------------

# Install libraries
install(TARGETS
    cpmbot_hw_interface
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executables
install(TARGETS
    cpmbot_hw_main
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/cpmbot_control/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
