cmake_minimum_required(VERSION 3.5)
project(irayple)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)

# Генерируем сообщения
set(msg_files
    "msg/Irayple.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)


ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(DIRECTORY ${PROJECT_NAME}
    DESTINATION ${PYTHON_INSTALL_DIR}
    FILES_MATCHING
    PATTERN "*.py"
    PATTERN "__pycache__" EXCLUDE
    PATTERN "*.pyc" EXCLUDE
)

# Устанавливаем исполняемые файлы
install(PROGRAMS
    ${PROJECT_NAME}/parser.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME irayple_parser
)
install(PROGRAMS
    centration/test/irayple_navigator.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME irayple_navigator
)
install(PROGRAMS
    centration/irayple_center.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME irayple_center
)
install(PROGRAMS
    centration/test/irayple_center_short.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME irayple_center_short
)
install(PROGRAMS
    centration/test/irayple_center_simple.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME irayple_center_simple
)
install(PROGRAMS
    pose/irayple_qr_pose_pub.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME irayple_pose_pub
)
install(PROGRAMS
    centration/irayple_navigator2.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME irayple_navigator2
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)
install(FILES pose/markers_config.txt
    DESTINATION share/${PROJECT_NAME}/pose
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
