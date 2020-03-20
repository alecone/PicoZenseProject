cmake_minimum_required(VERSION 3.5)
project(PicoZenseProject VERSION 0.1.0)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# include(CTest)
# enable_testing()

set(CMAKE_CXX_STANDARD 17)

add_executable(PicoZenseProject
    main.cpp
    PicoZenseHandler.h
    PicoZenseHandler.cpp
    MultiCamWorker.cpp
)
include_directories(
    /usr/include/pcl-1.8
    /usr/include/eigen3
    /usr/include/vtk-6.3
    /opt/PicoZense/Thirdparty/opencv-3.4.1/include
    /opt/PicoZense/Include
)
link_directories(
    /usr/lib/x86_64-linux-gnu
    /opt/PicoZense/Thirdparty/opencv-3.4.1/lib/x64
    /opt/PicoZense/Lib/x64
)
# add_definitions(
#     ${PCL_DEFINITIONS}
# )

target_link_libraries(PicoZenseProject
    # ${PCL_LIBRARIES}
    opencv_core
    opencv_highgui
    opencv_imgproc
    picozense_api
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
