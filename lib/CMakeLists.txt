cmake_minimum_required(VERSION 3.10)
project(Test)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable warnings and optimizations
add_compile_options(-Wall -Wextra -O3)

# ----------------------------------------------------------------------------
# 1. Find dependencies
# ----------------------------------------------------------------------------

# Find Eigen3
find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3 found")

# Find PCL
find_package(PCL 1.14 REQUIRED COMPONENTS common filters io visualization segmentation surface integral_image_normal region_growing features)
message(STATUS "PCL_VERSION: ${PCL_VERSION}")

# Find Open3D
find_package(Open3D REQUIRED)
message(STATUS "Open3D found")

# Find OctoMap
find_package(octomap REQUIRED)
message(STATUS "OctoMap found")

# Find CGAL
find_package(CGAL REQUIRED)
message(STATUS "CGAL found")

# Find grid_map_core
find_library(GRID_MAP_CORE_LIBRARY NAMES grid_map_core PATHS /usr/local/lib)
find_path(GRID_MAP_CORE_INCLUDE_DIR NAMES grid_map_core/GridMap.hpp PATHS /usr/local/include)
if(NOT GRID_MAP_CORE_LIBRARY OR NOT GRID_MAP_CORE_INCLUDE_DIR)
    message(FATAL_ERROR "grid_map_core not found")
endif()
message(STATUS "grid_map_core found")

# Find VTK
find_package(VTK REQUIRED)
message(STATUS "VTK found")

# Find OpenMP
find_package(OpenMP REQUIRED)
message(STATUS "OpenMP found")

# Find yaml-cpp
find_package(yaml-cpp QUIET)
if(yaml-cpp_FOUND)
    message(STATUS "yaml-cpp found")
else()
    message(WARNING "yaml-cpp not found, linking may fail if required")
endif()

# Find other libraries
find_package(OpenGL REQUIRED)
find_package(glfw3 QUIET)
find_package(GLEW QUIET)
find_package(assimp QUIET)
find_package(PNG QUIET)
find_package(JPEG QUIET)

# ----------------------------------------------------------------------------
# 2. Set include directories
# ----------------------------------------------------------------------------

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${Open3D_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${CGAL_INCLUDE_DIRS}
    ${GRID_MAP_CORE_INCLUDE_DIR}
    ${VTK_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIRS}
    ${yaml-cpp_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIRS}
    ${assimp_INCLUDE_DIRS}
    ${PNG_INCLUDE_DIRS}
    ${JPEG_INCLUDE_DIRS}
)

# ----------------------------------------------------------------------------
# 3. Set linker directories and definitions
# ----------------------------------------------------------------------------

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ----------------------------------------------------------------------------
# 4. Create executable
# ----------------------------------------------------------------------------

add_executable(main main.cpp)

# ----------------------------------------------------------------------------
# 5. Link libraries
# ----------------------------------------------------------------------------

target_link_libraries(main
    OpenMP::OpenMP_CXX
    yaml-cpp
    Open3D::Open3D
    ${PCL_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
    CGAL::CGAL
    ${GRID_MAP_CORE_LIBRARY}
    Eigen3::Eigen
    ${VTK_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${GLFW3_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${assimp_LIBRARIES}
    ${PNG_LIBRARIES}
    ${JPEG_LIBRARIES}
    z
    pthread
)

# ----------------------------------------------------------------------------
# 6. Optional linker flags
# ----------------------------------------------------------------------------

target_link_options(main PRIVATE -Wl,--copy-dt-needed-entries)