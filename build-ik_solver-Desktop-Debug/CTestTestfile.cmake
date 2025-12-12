# CMake generated Testfile for 
# Source directory: /home/galileo/projects/merlin2_ws/src/cnr/ik_solver/ik_solver
# Build directory: /home/galileo/projects/merlin2_ws/src/cnr/ik_solver/build-ik_solver-Desktop-Debug
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_node_ros2 "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/galileo/projects/merlin2_ws/src/cnr/ik_solver/build-ik_solver-Desktop-Debug/test_results/ik_solver/test_node_ros2.gtest.xml" "--package-name" "ik_solver" "--output-file" "/home/galileo/projects/merlin2_ws/src/cnr/ik_solver/build-ik_solver-Desktop-Debug/ament_cmake_gtest/test_node_ros2.txt" "--command" "/home/galileo/projects/merlin2_ws/src/cnr/ik_solver/build-ik_solver-Desktop-Debug/test_node_ros2" "--gtest_output=xml:/home/galileo/projects/merlin2_ws/src/cnr/ik_solver/build-ik_solver-Desktop-Debug/test_results/ik_solver/test_node_ros2.gtest.xml")
set_tests_properties(test_node_ros2 PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/galileo/projects/merlin2_ws/src/cnr/ik_solver/build-ik_solver-Desktop-Debug/test_node_ros2" TIMEOUT "60" WORKING_DIRECTORY "/home/galileo/projects/merlin2_ws/src/cnr/ik_solver/build-ik_solver-Desktop-Debug" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/galileo/projects/merlin2_ws/src/cnr/ik_solver/ik_solver/CMakeLists.txt;192;ament_add_gtest;/home/galileo/projects/merlin2_ws/src/cnr/ik_solver/ik_solver/CMakeLists.txt;0;")
subdirs("gtest")
