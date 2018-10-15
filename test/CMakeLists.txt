set(GTEST_SHUFFLE 1)

find_package( OpenCV REQUIRED )

add_executable(
    cpp-test
    main.cpp
    test.cpp
    ../app/RRTStarPlanner/MapManager.cpp
    ../app/RRTStarPlanner/Node.cpp
    ../app/RRTStarPlanner/RRTStarPlanner.cpp
)

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories( ${OpenCV_INCLUDE_DIRS} )

target_include_directories(cpp-test PUBLIC ../vendor/googletest/googletest/include 
                                           ${CMAKE_SOURCE_DIR}/include)
                                           
target_link_libraries(cpp-test PUBLIC gtest ${OpenCV_LIBS})
