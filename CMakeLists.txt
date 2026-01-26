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

# Устанавливаем Python пакет вручную
install(DIRECTORY ${PROJECT_NAME}
    DESTINATION lib/python3.12/site-packages
    FILES_MATCHING
    PATTERN "*.py"
    PATTERN "__pycache__" EXCLUDE
    PATTERN "*.pyc" EXCLUDE
)

# Устанавливаем исполняемый файл
install(PROGRAMS
    ${PROJECT_NAME}/parser.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME irayple_parser
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
