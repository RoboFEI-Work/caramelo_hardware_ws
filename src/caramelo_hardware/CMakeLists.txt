cmake_minimum_required(VERSION 3.8)
project(caramelo_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# O colcon nao define CMAKE_BUILD_TYPE, e sem ele o gcc compila com -O0.
# Isto NAO e' cosmetico neste pacote: medido na Pi 5 em 2026-09-01, o laco de
# amostragem do RIO faz 6.4 MHz com -O2 e 1.0 MHz com -O0 — fator 6. O mesmo
# vale para o laco de controle de 100 Hz sob SCHED_FIFO.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
  message(STATUS "caramelo_hardware: CMAKE_BUILD_TYPE nao definido -> RelWithDebInfo.")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pal_statistics REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)

find_package(Threads REQUIRED)

# Backend GPIO. Sem lgpio o driver COMPILA mas initialize() devolve false e o
# on_configure derruba o componente — falha em runtime por um problema de build.
# Por isso o default e' falhar aqui. Para build sem hardware (PC de dev, testes):
#   colcon build --packages-select caramelo_hardware --cmake-args -DCARAMELO_REQUIRE_LGPIO=OFF
option(CARAMELO_REQUIRE_LGPIO "Falha o build se o backend lgpio nao for encontrado" ON)

include(CheckIncludeFileCXX)
check_include_file_cxx(lgpio.h HAVE_LGPIO_H)
find_library(LGPIO_LIB lgpio)

include_directories(include)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/arm_hw_interface.cpp
  src/mobile_base_hw_interface.cpp
  src/maxon_motors_node.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pal_statistics
  pluginlib
  std_msgs
  dynamixel_sdk
)

if(HAVE_LGPIO_H AND LGPIO_LIB)
  target_compile_definitions(${PROJECT_NAME} PRIVATE CARAMELO_HAS_LGPIO=1)
  target_link_libraries(${PROJECT_NAME} ${LGPIO_LIB} Threads::Threads)
  message(STATUS "caramelo_hardware: lgpio backend enabled (${LGPIO_LIB}).")
elseif(CARAMELO_REQUIRE_LGPIO)
  message(FATAL_ERROR
    "caramelo_hardware: backend lgpio NAO encontrado (lgpio.h / liblgpio).\n"
    "  Na Raspberry:  sudo apt install liblgpio-dev\n"
    "  Build sem GPIO (PC de dev / so testes):\n"
    "    colcon build --packages-select caramelo_hardware --cmake-args -DCARAMELO_REQUIRE_LGPIO=OFF\n"
    "  Depois de instalar a lib, apague build/caramelo_hardware: o find_library e' cacheado.")
else()
  target_compile_definitions(${PROJECT_NAME} PRIVATE CARAMELO_HAS_LGPIO=0)
  target_link_libraries(${PROJECT_NAME} Threads::Threads)
  message(WARNING "caramelo_hardware: build SEM GPIO. on_configure vai FALHAR em runtime.")
endif()

# Ferramenta de bancada dos encoders. So' depende de mmap(/dev/gpiomem0) — nao
# usa lgpio nem ROS, e por isso roda com o stack de controle inteiro parado.
add_executable(encoder_probe tools/encoder_probe.cpp)
target_include_directories(encoder_probe PRIVATE include)
target_link_libraries(encoder_probe Threads::Threads)
if(HAVE_LGPIO_H AND LGPIO_LIB)
  # lgpio so' e' usado pelo modo --hold-neutral (PWM). A contagem de encoder e'
  # MMIO puro e funciona sem ele.
  target_compile_definitions(encoder_probe PRIVATE CARAMELO_HAS_LGPIO=1)
  target_link_libraries(encoder_probe ${LGPIO_LIB})
endif()

pluginlib_export_plugin_description_file(hardware_interface caramelo_hw_interface.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  TARGETS encoder_probe
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES caramelo_hw_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pal_statistics
  pluginlib
  std_msgs
  dynamixel_sdk
)

ament_package()
