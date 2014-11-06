CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(ubx_rsg_fmpc)

set(CMAKE_CXX_FLAGS "-Wall -Werror -fvisibility=hidden")
set(CMAKE_CXX_COMPILER clang++ )
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake") ## temporary resources, for UBX

# Global
set(INSTALL_LIB_TYPES_DIR lib/ubx/types CACHE PATH "Installation directory for types libraries")
set(INSTALL_LIB_BLOCKS_DIR lib/ubx/blocks CACHE PATH "Installation directory for blocks libraries")
set(INSTALL_BIN_APPS_DIR bin CACHE PATH "Installation directory for bin")
set(INSTALL_INCLUDE_DIR include/ubx CACHE PATH "Installation directory for header files (types)")
set(INSTALL_CMAKE_DIR  share/ubx/cmake CACHE PATH  "Installation directory for CMake files") # default ${DEF_INSTALL_CMAKE_DIR}

# Make relative paths absolute
foreach(p LIB_TYPES LIB_BLOCKS BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

##
# Add uninstall target.
##
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)
    
add_custom_target(uninstall
    "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake"
)

##
# Generate config package
##
set(CONF_INCLUDE_DIRS "${INSTALL_INCLUDE_DIR}")
set(CONF_CMAKE_DIR "${INSTALL_CMAKE_DIR}")
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/ubx-rsg-fmpc-config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake/ubx-rsg-fmpc-config.cmake" @ONLY
)

set (UBX_ROOT $ENV{UBX_ROOT} )
if (NOT UBX_ROOT)
 message( FATAL_ERROR "UBX_ROOT is not defined. CMake install willl abort." )
endif()

find_package(Ubx REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)
find_package(BRICS_3D REQUIRED)
ADD_DEFINITIONS(-DEIGEN3)

include_directories(
  $ENV{FBX_MODULES}
  ${UBX_INCLUDE_DIR}
  ${BRICS_3D_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
)

LINK_DIRECTORIES(${BRICS_3D_LINK_DIRECTORIES})

# Compile library fmpcwmlib
add_library(fmpcwmlib SHARED 
    src/fmpc_world_model.cpp
    modules/fmpcwmlib_module.c
)
set_target_properties(fmpcwmlib PROPERTIES PREFIX "")
target_link_libraries(fmpcwmlib ${BRICS_3D_LIBRARIES} ${UBX_LIBRARIES})

# Install fmpcwmlib
install(TARGETS fmpcwmlib DESTINATION ${INSTALL_LIB_BLOCKS_DIR} EXPORT fmpcwmlib-block)
set_property(TARGET fmpcwmlib PROPERTY INSTALL_RPATH_USE_LINK_PATH TRUE)
install(EXPORT fmpcwmlib-block DESTINATION ${INSTALL_CMAKE_DIR})


# Compile library fmpcmockuplib
add_library(fmpcmockuplib SHARED 
    src/fmpc_mockup.cpp
)
set_target_properties(fmpcmockuplib PROPERTIES PREFIX "")
target_link_libraries(fmpcmockuplib ${UBX_LIBRARIES})

# Install fmpcmockuplib
install(TARGETS fmpcmockuplib DESTINATION ${INSTALL_LIB_BLOCKS_DIR} EXPORT fmpcmockuplib-block)
set_property(TARGET fmpcmockuplib PROPERTY INSTALL_RPATH_USE_LINK_PATH TRUE)
install(EXPORT fmpcmockuplib-block DESTINATION ${INSTALL_CMAKE_DIR})

