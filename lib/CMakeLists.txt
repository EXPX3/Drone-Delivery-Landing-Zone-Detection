cmake_minimum_required(VERSION 3.10)
project(Test)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ----------------------------------------------------------------------------
# 1. Find OPEN3D, PCL, and include the necessary modules
# ----------------------------------------------------------------------------

find_package(Open3D REQUIRED)

find_package(PCL 1.15 REQUIRED COMPONENTS common filters io visualization segmentation surface integral_image_normal region_growing features)
message(STATUS "PCL_VERSION: ${PCL_VERSION}")
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})


include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/

)

# ----------------------------------------------------------------------------
# 2. Optionally, find OpenMP if needed for parallelization
# ----------------------------------------------------------------------------
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    message(STATUS "OpenMP found. Linking to OpenMP libraries.")
endif()

# ----------------------------------------------------------------------------
# 3. Manually include other libraries' include paths
# ----------------------------------------------------------------------------
include_directories(
    /usr/local/include/eigen3
    /usr/local/include/pcl-1.15
)

# ----------------------------------------------------------------------------
# 4. Find VTK components (required by Open3D's third-party VTK modules)
# ----------------------------------------------------------------------------

find_package(VTK REQUIRED) 
# If you want to specify them, do:
# find_package(VTK REQUIRED COMPONENTS FiltersGeneral FiltersModeling FiltersCore CommonDataModel CommonExecutionModel)


set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -Wl,--copy-dt-needed-entries"
)
# ----------------------------------------------------------------------------
# 5. Create the executables
# ----------------------------------------------------------------------------
add_executable(main main.cpp)

# ----------------------------------------------------------------------------
# 6. Link libraries
# ----------------------------------------------------------------------------

# Link libraries.
target_link_libraries(main
  OpenMP::OpenMP_CXX
  yaml-cpp 
  Open3D::Open3D 
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}  
  GL                     # -lGL
  GLU                    # -lGLU
  glfw                   # -lglfw
  GLEW                   # -lGLEW
  assimp                 # -lassimp
  jsoncpp               # -ljsoncpp
  jpeg                   # -ljpeg
  png                    # -lpng
  z                      # -lz
  pthread                # -lpthread
)

# ----------------------------------------------------------------------------
# 7. Add OpenMP flags
# ----------------------------------------------------------------------------
if(OPENMP_FOUND)
    target_compile_options(main PRIVATE ${OpenMP_CXX_FLAGS})
endif()