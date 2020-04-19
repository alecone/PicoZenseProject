cmake_minimum_required(VERSION 3.5)
project(PicoZenseProject VERSION 1.0.0)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# include(CTest)
# enable_testing()

set(CMAKE_CXX_STANDARD 17)
find_package(OpenCV REQUIRED)
find_library(picozense_api NAMES libpicozense_api.so PATHS external/lib/)

add_executable(PicoZenseProject
    inc/PicoZense_api.h
    inc/PicoZense_define.h
    inc/PicoZense_enums.h
    inc/PicoZense_types.h
    inc/PicoZenseHandler.h
    inc/MultiCamWorker.h
    src/PicoZenseHandler.cpp
    src/MultiCamWorker.cpp
    src/main.cpp
)
include_directories(
    inc
    ${OpenCV_INCLUDE_DIRS}
    /usr/include/pcl-1.8
    /usr/include/eigen3
    /usr/include/vtk-6.3
)
# link_directories(
#     /usr/lib/x86_64-linux-gnu
# )

target_link_libraries(PicoZenseProject
    ${OpenCV_LIBRARIES}
    ${picozense_api}
    #Including only the necessary ones
    pcl_visualization
    boost_system
    pcl_common
    pcl_kdtree
    pcl_keypoints
    pcl_search
    pcl_features
    pcl_io
    pcl_filters
    pcl_surface
    vtkRenderingFreeType-6.3
    vtkRenderingCore-6.3
    vtkCommonExecutionModel-6.3
    vtkCommonDataModel-6.3
    vtkCommonMath-6.3
    vtkCommonCore-6.3
    vtkFiltersCore-6.3
    vtkFiltersGeometry-6.3
    # vtkRenderingLOD-6.3
    # vtkFiltersSources-6.3
    pthread
    boost_thread
)
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
