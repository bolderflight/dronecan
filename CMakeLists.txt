cmake_minimum_required(VERSION 3.14)
if (DEFINED MCU)
  include(FetchContent)
  FetchContent_Declare(
    mcu_support
    GIT_REPOSITORY https://github.com/bolderflight/mcu-support.git
    GIT_TAG v1.0.0
  )
  FetchContent_MakeAvailable(mcu_support)
  # Setting up the toolchain
  set(CMAKE_TOOLCHAIN_FILE "${mcu_support_SOURCE_DIR}/cmake/cortex.cmake")
  # Project information
  project(DroneCan
    VERSION 1.0.0
    DESCRIPTION "DroneCAN driver"
    LANGUAGES C CXX
  )
  # Grab the processor and set up definitions and compile options
  include(${mcu_support_SOURCE_DIR}/cmake/config_mcu.cmake)
  configMcu(${MCU} ${mcu_support_SOURCE_DIR})
  # Fetch core
  FetchContent_Declare(
    core
    GIT_REPOSITORY https://github.com/bolderflight/core.git
    GIT_TAG v3.0.2
  )
  FetchContent_MakeAvailable(core)
  # Fetch easycan
  FetchContent_Declare(
    easycan
    GIT_REPOSITORY https://github.com/bolderflight/easycan.git
    GIT_TAG v1.0.0
  )
  FetchContent_MakeAvailable(easycan)
  # Add the library target
  file(GLOB_RECURSE dronecan_SRC CONFIGURE_DEPENDS "src/*.h" "src/*.cpp" "src/*.hpp")
  add_library(dronecan ${dronecan_SRC})
  # Link libraries
  target_link_libraries(dronecan
    PUBLIC
      core
      easycan
  )
  # Setup include directories 
  target_include_directories(dronecan PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
  )
  # Example and test if this project is built separately
  if (PROJECT_NAME STREQUAL CMAKE_PROJECT_NAME)
    # Add the publisher example target
    add_executable(pub examples/pub.cc)
    # Add the includes
    target_include_directories(pub PUBLIC 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    # Link libraries to the example target
    target_link_libraries(pub
      PRIVATE 
        dronecan
    )
    # Add hex and upload targets
    include(${mcu_support_SOURCE_DIR}/cmake/flash_mcu.cmake)
    FlashMcu(pub ${MCU} ${mcu_support_SOURCE_DIR})

    # Add the subscriber example target
    add_executable(sub examples/sub.cc)
    # Add the includes
    target_include_directories(sub PUBLIC 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    # Link libraries to the example target
    target_link_libraries(sub
      PRIVATE 
        dronecan
    )
    # Add hex and upload targets
    include(${mcu_support_SOURCE_DIR}/cmake/flash_mcu.cmake)
    FlashMcu(sub ${MCU} ${mcu_support_SOURCE_DIR})
  endif()
endif()
