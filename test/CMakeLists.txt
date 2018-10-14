set(GTEST_SHUFFLE 1)

find_package(PythonLibs 2.7)

add_executable(
    cpp-test
    main.cpp
    test.cpp
)

target_include_directories(cpp-test PUBLIC ../vendor/googletest/googletest/include 
                                           ${CMAKE_SOURCE_DIR}/include ${PYTHON_INCLUDE_DIRS})
                                           
target_link_libraries(cpp-test PUBLIC gtest ${PYTHON_LIBRARIES})
