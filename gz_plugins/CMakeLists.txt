# =============================================================================
# robomini_sim plugin (Gazebo Harmonic / gz-sim 8) build file.
#
# Build & install (no sudo, everything stays under gz_plugins/):
#
#   cd $SIM/gz_plugins
#   cmake -B build -S . -DCMAKE_BUILD_TYPE=RelWithDebInfo
#   cmake --build build -j"$(nproc)"
#   cmake --install build            # -> $SIM/gz_plugins/install/lib/librobomini_sim_system.so
#
# Run with:
#   export GZ_SIM_SYSTEM_PLUGIN_PATH=$SIM/gz_plugins/install/lib
#   gz sim -s -r --headless-rendering <world.sdf>
# =============================================================================
cmake_minimum_required(VERSION 3.16.3)
project(robomini_sim_plugin VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Keep the plugin self-contained inside gz_plugins/ (never install system-wide).
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/install"
      CACHE PATH "Local install prefix (do not use /usr)" FORCE)
endif()

find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-common5 REQUIRED)

add_library(robomini_sim_system SHARED
  src/RobominiSim.cc
  src/RobominiSim.hh
  src/json_lite.hh
)

target_include_directories(robomini_sim_system PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_link_libraries(robomini_sim_system PUBLIC
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-math7::gz-math7
  gz-common5::gz-common5
)

target_compile_options(robomini_sim_system PRIVATE -Wall -Wextra)

install(TARGETS robomini_sim_system
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

message(STATUS "robomini_sim plugin will install to: ${CMAKE_INSTALL_PREFIX}")
