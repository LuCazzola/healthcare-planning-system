cmake_minimum_required(VERSION 3.5)
project(p5)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# Define dependencies list using the correct format
set(dependencies
  rclcpp
  plansys2_executor
  plansys2_msgs
  lifecycle_msgs
)

# Create executables
add_executable(deliver_patient_node src/deliver_patient_node.cpp)
ament_target_dependencies(deliver_patient_node ${dependencies})

add_executable(deliver_supply_node src/deliver_supply_node.cpp)
ament_target_dependencies(deliver_supply_node ${dependencies})

add_executable(drop_down_1st_box_node src/drop_down_1st_box_node.cpp)
ament_target_dependencies(drop_down_1st_box_node ${dependencies})

add_executable(drop_down_2nd_box_node src/drop_down_2nd_box_node.cpp)
ament_target_dependencies(drop_down_2nd_box_node ${dependencies})

add_executable(drop_down_3rd_box_node src/drop_down_3rd_box_node.cpp)
ament_target_dependencies(drop_down_3rd_box_node ${dependencies})

add_executable(drop_down_patient_node src/drop_down_patient_node.cpp)
ament_target_dependencies(drop_down_patient_node ${dependencies})

add_executable(fill_box_node src/fill_box_node.cpp)
ament_target_dependencies(fill_box_node ${dependencies})

add_executable(move_worker_bot_alone_node src/move_worker_bot_alone_node.cpp)
ament_target_dependencies(move_worker_bot_alone_node ${dependencies})

add_executable(move_helper_bot_alone_node src/move_helper_bot_alone_node.cpp)
ament_target_dependencies(move_helper_bot_alone_node ${dependencies})

add_executable(move_helper_bot_patient_node src/move_helper_bot_patient_node.cpp)
ament_target_dependencies(move_helper_bot_patient_node ${dependencies})

add_executable(move_worker_bot_with_1_box_node src/move_worker_bot_with_1_box_node.cpp)
ament_target_dependencies(move_worker_bot_with_1_box_node ${dependencies})

add_executable(move_worker_bot_with_2_boxes_node src/move_worker_bot_with_2_boxes_node.cpp)
ament_target_dependencies(move_worker_bot_with_2_boxes_node ${dependencies})

add_executable(move_worker_bot_with_3_boxes_node src/move_worker_bot_with_3_boxes_node.cpp)
ament_target_dependencies(move_worker_bot_with_3_boxes_node ${dependencies})

add_executable(pick_up_1st_box_node src/pick_up_1st_box_node.cpp)
ament_target_dependencies(pick_up_1st_box_node ${dependencies})

add_executable(pick_up_2nd_box_node src/pick_up_2nd_box_node.cpp)
ament_target_dependencies(pick_up_2nd_box_node ${dependencies})

add_executable(pick_up_3rd_box_node src/pick_up_3rd_box_node.cpp)
ament_target_dependencies(pick_up_3rd_box_node ${dependencies})

add_executable(pick_up_patient_node src/pick_up_patient_node.cpp)
ament_target_dependencies(pick_up_patient_node ${dependencies})

install(DIRECTORY launch pddl DESTINATION share/${PROJECT_NAME})

install(TARGETS
    deliver_patient_node
    deliver_supply_node
    drop_down_1st_box_node
    drop_down_2nd_box_node
    drop_down_3rd_box_node
    drop_down_patient_node
    fill_box_node
    move_worker_bot_alone_node
    move_helper_bot_alone_node
    move_helper_bot_patient_node
    move_worker_bot_with_1_box_node
    move_worker_bot_with_2_boxes_node
    move_worker_bot_with_3_boxes_node
    pick_up_1st_box_node
    pick_up_2nd_box_node
    pick_up_3rd_box_node
    pick_up_patient_node
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_package()