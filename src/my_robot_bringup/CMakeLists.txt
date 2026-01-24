cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config worlds scripts
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/camera_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/config.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/tsp_route_calculator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/tsp_solver_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/deliver_robot_tsp_client.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/random_box_spawner.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
