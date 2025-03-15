cmake_minimum_required(VERSION 3.5)
project(sierpinski_triangle)

# Függőségek keresése
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include könyvtárak (ha szükséges, hozzáadhatsz további könyvtárakat)
include_directories(include)

# Fordítási cél létrehozása (ha van ilyen)
add_executable(sierpinski_triangle_node src/sierpinski_triangle_node.cpp)

# Könyvtárak összekapcsolása
ament_target_dependencies(sierpinski_triangle_node rclcpp std_msgs)

# Célok telepítése
install(TARGETS
  sierpinski_triangle_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
  
ament_package()
