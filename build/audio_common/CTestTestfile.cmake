# CMake generated Testfile for 
# Source directory: /workspace/audio_common/audio_common
# Build directory: /workspace/build/audio_common
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(clang_format "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/workspace/build/audio_common/test_results/audio_common/clang_format.xunit.xml" "--package-name" "audio_common" "--output-file" "/workspace/build/audio_common/ament_clang_format/clang_format.txt" "--command" "/opt/ros/humble/bin/ament_clang_format" "--xunit-file" "/workspace/build/audio_common/test_results/audio_common/clang_format.xunit.xml" "--config" ".clang-format")
set_tests_properties(clang_format PROPERTIES  LABELS "clang_format;linter" TIMEOUT "60" WORKING_DIRECTORY "/workspace/audio_common/audio_common" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_clang_format/cmake/ament_clang_format.cmake;57;ament_add_test;/workspace/audio_common/audio_common/CMakeLists.txt;110;ament_clang_format;/workspace/audio_common/audio_common/CMakeLists.txt;0;")
