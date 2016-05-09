cmake_minimum_required(VERSION 2.8.11)
project(tank)

if(CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake_modules)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_FLAGS " -std=c++14 ")
#-g -pedantic-errors -Werror -Wextra -Wpedantic -Winit-self -Wmissing-declarations -Wuninitialized -Woverloaded-virtual -Wfatal-errors -O3 -Wno-overloaded-virtual")
add_definitions(${CMAKE_CXX_FLAGS})

file(GLOB SOURCE irrBullet/src/*.cpp irrBullet/src/*.h src/*.h src/*.cpp)

find_package(Irrlicht REQUIRED)
find_package(Bullet REQUIRED)

include_directories(
  ${IRRLICHT_INCLUDE_DIR}
  ${BULLET_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/irrBullet/include/
  ${CMAKE_SOURCE_DIR}/framework/
  )

add_executable(${CMAKE_PROJECT_NAME} ${SOURCE})

target_link_libraries(${CMAKE_PROJECT_NAME}
  ${IRRLICHT_LIBRARY}
  ${BULLET_LIBRARIES}
  )
