cmake_minimum_required(VERSION 3.16)
project(excavator_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf test/test_urdf.py)
  ament_add_pytest_test(test_launch test/test_launch.py)

  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_flake8()
  ament_pep257()
  ament_xmllint()
  ament_lint_cmake()
endif()

ament_package()
