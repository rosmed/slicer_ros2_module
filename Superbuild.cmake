# Superbuild.cmake
# Builds minimal_ros2 as an external project and then builds slicer_ros2_module against it.

include(ExternalProject)

if(NOT Python3_EXECUTABLE OR Python3_EXECUTABLE STREQUAL "/usr/bin/python3")
  if(Slicer_DIR AND EXISTS "${Slicer_DIR}/../python-install/bin/python3")
    get_filename_component(Python3_EXECUTABLE "${Slicer_DIR}/../python-install/bin/python3" ABSOLUTE)
    set(Python3_EXECUTABLE "${Python3_EXECUTABLE}" CACHE FILEPATH "Path to Python executable" FORCE)
  elseif(NOT Python3_EXECUTABLE)
    find_package(Python3 COMPONENTS Interpreter QUIET)
  endif()
endif()


set(MINIMAL_ROS2_INSTALL_DIR "${CMAKE_BINARY_DIR}/minimal_ros2-install")

set(MINIMAL_ROS_SOURCE_DIR "" CACHE PATH "Path to local minimal_ros2 source directory (optional)")

if(MINIMAL_ROS_SOURCE_DIR AND EXISTS "${MINIMAL_ROS_SOURCE_DIR}/CMakeLists.txt")
  message(STATUS "[Superbuild] Using local minimal_ros2 source: ${MINIMAL_ROS_SOURCE_DIR}")
  set(MINIMAL_ROS2_DOWNLOAD_ARGS SOURCE_DIR "${MINIMAL_ROS_SOURCE_DIR}")
else()
  message(STATUS "[Superbuild] Fetching minimal_ros2 from GitHub (main)...")
  set(MINIMAL_ROS2_DOWNLOAD_ARGS
    GIT_REPOSITORY "https://github.com/adeguet1/minimal_ros2.git"
    GIT_TAG "main"
  )
endif()

message(STATUS "[Superbuild] minimal_ros2 install prefix: ${MINIMAL_ROS2_INSTALL_DIR}")

include(ProcessorCount)
ProcessorCount(NCORES)
if(NCORES EQUAL 0)
  set(NCORES 4)
endif()

ExternalProject_Add(minimal_ros2
  ${MINIMAL_ROS2_DOWNLOAD_ARGS}
  BINARY_DIR "${CMAKE_BINARY_DIR}/minimal_ros2-build"
  INSTALL_DIR "${MINIMAL_ROS2_INSTALL_DIR}"
  BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --parallel ${NCORES}
  INSTALL_COMMAND ""
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX:PATH=${MINIMAL_ROS2_INSTALL_DIR}
    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
    -DPython3_EXECUTABLE:FILEPATH=${Python3_EXECUTABLE}
    -DFETCH_SOURCES:BOOL=ON
    -DBUILD_TESTING:BOOL=OFF
  BUILD_ALWAYS 0
)

# Assimp external project (built as static library to embed into module)
set(ASSIMP_INSTALL_DIR "${CMAKE_BINARY_DIR}/assimp-install")

set(ASSIMP_SOURCE_DIR "" CACHE PATH "Path to local assimp source directory (optional)")
if(ASSIMP_SOURCE_DIR AND EXISTS "${ASSIMP_SOURCE_DIR}/CMakeLists.txt")
  message(STATUS "[Superbuild] Using local assimp source: ${ASSIMP_SOURCE_DIR}")
  set(ASSIMP_DOWNLOAD_ARGS SOURCE_DIR "${ASSIMP_SOURCE_DIR}")
else()
  message(STATUS "[Superbuild] Fetching assimp from GitHub (v5.4.3)...")
  set(ASSIMP_DOWNLOAD_ARGS
    GIT_REPOSITORY "https://github.com/assimp/assimp.git"
    GIT_TAG "v5.4.3"
  )
endif()

ExternalProject_Add(assimp
  ${ASSIMP_DOWNLOAD_ARGS}
  BINARY_DIR "${CMAKE_BINARY_DIR}/assimp-build"
  INSTALL_DIR "${ASSIMP_INSTALL_DIR}"
  BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --parallel ${NCORES}
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX:PATH=${ASSIMP_INSTALL_DIR}
    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
    -DBUILD_SHARED_LIBS:BOOL=OFF
    -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON
    -DASSIMP_BUILD_TESTS:BOOL=OFF
    -DASSIMP_BUILD_ASSIMP_TOOLS:BOOL=OFF
    -DASSIMP_BUILD_DOCS:BOOL=OFF
    -DASSIMP_INSTALL_PDB:BOOL=OFF
    -DASSIMP_INJECT_DEBUG_POSTFIX:BOOL=OFF
    -DASSIMP_NO_EXPORT:BOOL=ON
    -DASSIMP_BUILD_ZLIB:BOOL=OFF
    -DASSIMP_WARNINGS_AS_ERRORS:BOOL=OFF
  BUILD_ALWAYS 0
)

# Inner build of slicer_ros2_module
set(INNER_BUILD_DIR "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-build")

set(INNER_EXTRA_ARGS)
if(MINIMAL_ROS_SOURCE_DIR)
  list(APPEND INNER_EXTRA_ARGS "-DMINIMAL_ROS_SOURCE_DIR:PATH=${MINIMAL_ROS_SOURCE_DIR}")
  if(EXISTS "${MINIMAL_ROS_SOURCE_DIR}/.venv")
    list(APPEND INNER_EXTRA_ARGS "-DMINIMAL_ROS_VENV_DIR:PATH=${MINIMAL_ROS_SOURCE_DIR}/.venv")
  endif()
elseif(EXISTS "${CMAKE_BINARY_DIR}/minimal_ros2-prefix/src/minimal_ros2/.venv")
  list(APPEND INNER_EXTRA_ARGS "-DMINIMAL_ROS_VENV_DIR:PATH=${CMAKE_BINARY_DIR}/minimal_ros2-prefix/src/minimal_ros2/.venv")
endif()

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT OR CMAKE_INSTALL_PREFIX STREQUAL "/usr/local")
  set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-install" CACHE PATH "Install path prefix" FORCE)
endif()

ExternalProject_Add(${PROJECT_NAME}_inner
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}"
  BINARY_DIR "${INNER_BUILD_DIR}"
  INSTALL_DIR "${CMAKE_INSTALL_PREFIX}"
  DEPENDS minimal_ros2 assimp
  BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --parallel ${NCORES}
  INSTALL_COMMAND ""
  CMAKE_ARGS
    -DSlicerROS2_SUPERBUILD_BUILDING_INNER:BOOL=ON
    -DSlicerROS2_USE_MINIMAL_ROS:BOOL=ON
    -DMINIMAL_ROS_DIR:PATH=${MINIMAL_ROS2_INSTALL_DIR}
    -DASSIMP_DIR:PATH=${ASSIMP_INSTALL_DIR}
    -Dassimp_DIR:PATH=${ASSIMP_INSTALL_DIR}/lib/cmake/assimp-5.4
    -DSlicer_DIR:PATH=${Slicer_DIR}
    -DPython3_EXECUTABLE:FILEPATH=${Python3_EXECUTABLE}
    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
    ${INNER_EXTRA_ARGS}
)

add_custom_target(${PROJECT_NAME} ALL DEPENDS ${PROJECT_NAME}_inner)
