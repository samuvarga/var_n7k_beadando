cmake_minimum_required(VERSION 3.5)
project(var_n7k_beadando)

# Függőségek keresése
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

set(ament_dependencies
  rclcpp
  std_msgs
  geometry_msgs
  turtlesim
)

# Include könyvtárak (ha szükséges, hozzáadhatsz további könyvtárakat)
include_directories(include)

# Fordítási cél létrehozása (ha van ilyen)
add_executable(sierpinski_triangle_node var_n7k_beadando/sierpinski_triangle_node.py)

# Könyvtárak összekapcsolása
ament_target_dependencies(sierpinski_triangle_node rclcpp std_msgs)

# Célok telepítése
install(TARGETS
  sierpinski_triangle_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  sierpinski_triangle_node.py 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
