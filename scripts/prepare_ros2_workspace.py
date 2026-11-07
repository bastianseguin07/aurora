"""Generate a build-only adaptation of the reference ROS driver for Lyrical."""

from pathlib import Path
import shutil
import sys

reference = Path(sys.argv[1]).resolve()
workspace = Path(sys.argv[2]).resolve()
original = reference / "unitree_lidar_ros2/src/unitree_lidar_ros2"
if not original.is_dir():
    raise SystemExit("Falta el paquete ROS 2 en el clon de referencia")
package = workspace / "src/unitree_lidar_ros2"
shutil.copytree(
    original, package, dirs_exist_ok=True, ignore=shutil.ignore_patterns("__pycache__")
)
shutil.copyfile(reference / "LICENSE", package / "LICENSE.reference")
header = package / "include/unitree_lidar_ros2.h"
source = header.read_text()
source = source.replace(
    'tf2_ros/transform_broadcaster.h"', 'tf2_ros/transform_broadcaster.hpp"'
)
source = source.replace(
    'tf2/LinearMath/Quaternion.h"', 'tf2/LinearMath/Quaternion.hpp"'
)
source = source.replace(
    'declare_parameter<std::string>("lidar_ip", "192.168.1.2")',
    'declare_parameter<std::string>("lidar_ip", "192.168.1.62")',
)
source = source.replace(
    'declare_parameter<std::string>("local_ip", "192.168.1.62")',
    'declare_parameter<std::string>("local_ip", "192.168.1.2")',
)
source = source.replace(
    'declare_parameter<double>("range_max", 50)',
    'declare_parameter<double>("range_max", 1000000.0)',
)
source = source.replace(
    "    lsdk_->setLidarWorkMode(work_mode_);",
    "    lsdk_->startLidarRotation();\n    lsdk_->setLidarWorkMode(work_mode_);",
)
source = source.replace(
    "    ~UnitreeLidarSDKNode() {};",
    "    ~UnitreeLidarSDKNode() { if (lsdk_) { lsdk_->stopLidarRotation(); lsdk_->closeUDP(); } };",
)
old_udp = "lsdk_->initializeUDP(lidar_port_, lidar_ip_, local_port_, local_ip_,\n                             cloud_scan_num_, use_system_timestamp_, range_min_, range_max_);"
new_udp = 'if (lsdk_->initializeUDP(lidar_port_, lidar_ip_, local_port_, local_ip_,\n                             cloud_scan_num_, use_system_timestamp_, range_min_, range_max_)) {\n            throw std::runtime_error("Unitree UDP: revisa IP local, puertos y otro driver abierto");\n        }'
source = source.replace(old_udp, new_udp)
source = source.replace("        lsdk_->getImuData(imu);\n", "")
header.write_text(source)
# Preserve the SDK's uint32 ring when publishing through PCL/ROS.
pcl_header = package / "include/unitree_lidar_sdk_pcl.h"
pcl_source = (
    reference / "unitree_lidar_sdk/include/unitree_lidar_sdk_pcl.h"
).read_text()
pcl_source = pcl_source.replace("std::uint16_t ring;", "std::uint32_t ring;").replace(
    "(std::uint16_t, ring, ring)", "(std::uint32_t, ring, ring)"
)
pcl_header.write_text(pcl_source)
(package / "CMakeLists.txt").write_text("""cmake_minimum_required(VERSION 3.16)
project(unitree_lidar_ros2)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
if(NOT UNITREE_SDK_DIR)
  message(FATAL_ERROR "Define UNITREE_SDK_DIR pointing to the external SDK")
endif()
add_executable(unitree_lidar_ros2_node src/unitree_lidar_ros2_node.cpp)
target_include_directories(unitree_lidar_ros2_node PRIVATE include ${PCL_INCLUDE_DIRS} ${UNITREE_SDK_DIR}/include)
target_link_libraries(unitree_lidar_ros2_node ${PCL_LIBRARIES} ${UNITREE_SDK_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}/libunilidar_sdk2.a)
target_link_libraries(unitree_lidar_ros2_node rclcpp::rclcpp std_msgs::std_msgs sensor_msgs::sensor_msgs geometry_msgs::geometry_msgs tf2_ros::tf2_ros pcl_conversions::pcl_conversions)
install(TARGETS unitree_lidar_ros2_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
ament_package()
""")
(package / "launch/aurora.launch.py").write_text("""from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    defaults = {
        "lidar_ip":"192.168.1.62", "local_ip":"192.168.1.2", "lidar_port":"6101",
        "local_port":"6201", "cloud_scan_num":"18", "rviz":"true",
    }
    arguments=[DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]
    params={
        "initialize_type":2, "work_mode":0, "use_system_timestamp":True,
        "range_min":0.0, "range_max":1000000.0,
        "cloud_frame":"unilidar_lidar", "cloud_topic":"/unilidar/cloud",
        "imu_frame":"unilidar_imu", "imu_topic":"/unilidar/imu",
    }
    for name in ("lidar_ip","local_ip"):
        params[name]=ParameterValue(LaunchConfiguration(name),value_type=str)
    for name in ("lidar_port","local_port","cloud_scan_num"):
        params[name]=ParameterValue(LaunchConfiguration(name),value_type=int)
    driver=Node(package="unitree_lidar_ros2",executable="unitree_lidar_ros2_node",parameters=[params],output="screen")
    config=Path(get_package_share_directory("unitree_lidar_ros2"))/"rviz/aurora.rviz"
    rviz=Node(package="rviz2",executable="rviz2",arguments=["-d",str(config)],condition=IfCondition(LaunchConfiguration("rviz")))
    return LaunchDescription(arguments+[driver,rviz])
""")
(package / "rviz/aurora.rviz").write_text("""Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Cell Size: 1
      Plane: XY
    - Class: rviz_default_plugins/PointCloud2
      Name: Unitree
      Enabled: true
      Topic:
        Value: /unilidar/cloud
        Reliability Policy: Best Effort
        Durability Policy: Volatile
        History Policy: Keep Last
        Depth: 10
      Queue Size: 10
      Decay Time: 1
      Style: Points
      Size (Pixels): 2
      Color Transformer: Intensity
      Channel Name: intensity
      Autocompute Intensity Bounds: true
      Position Transformer: XYZ
  Global Options:
    Fixed Frame: unilidar_lidar
    Frame Rate: 30
    Background Color: 48; 48; 48
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 5
      Pitch: 0.5
      Yaw: 0.8
Window Geometry:
  Width: 1200
  Height: 800
""")
print("Workspace generado:", workspace)
