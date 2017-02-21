
add_library(LaserSlam SHARED
    ./ParameterReader.cpp
    ./LaserFrame.cpp
    ./LaserOdometry.cpp
    ./PoseVisual.cpp
    )

set(build_libs
    scanmatching
    ${OpenCV_LIBS}    
)

#add_library(odometry LaserOdometry.cpp)

#target_link_libraries(odometry 
    #${PROJECT_SOURCE_DIR}/Thirdparty/scanmatching/lib/libscanmatching.so
#)
#target_link_libraries(odometry scanmatching)


#target_link_libraries(odometry ${PROJECT_SOURCE_DIR}/Thirdparty/scanmatching/lib/scanmatching)


#target_link_libraries(LaserSlam
#${build_libs}
#)

target_link_libraries(LaserSlam ${build_libs})


