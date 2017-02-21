
add_executable( LaserFrameTest LaserFrameTest.cpp)
add_dependencies(LaserFrameTest
    LaserSlam
)
target_link_libraries(LaserFrameTest LaserSlam)

add_executable ( LaserOdometryTest LaserOdometryTest.cpp )
#add_dependencies(LaserOdometryTest
    #LaserSlam
#)

target_link_libraries(LaserOdometryTest LaserSlam)

add_executable(PoseVisualTest PoseVisualTest.cpp)

target_link_libraries(PoseVisualTest LaserSlam)


