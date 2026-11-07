"""Adapt the pinned external Point-LIO ROS 2 sources in a build workspace."""

from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import yaml

reference = Path(sys.argv[1]).resolve()
destination = Path(sys.argv[2]).resolve() / "src/point_lio"
staging = tempfile.TemporaryDirectory(prefix="aurora-point-lio-")
package = Path(staging.name) / "point_lio"
package.mkdir(parents=True, exist_ok=True)
for name in ("src", "include", "config", "launch", "rviz_cfg"):
    shutil.copytree(reference / name, package / name, dirs_exist_ok=True)
for name in ("LICENSE", "README.md", "package.xml"):
    shutil.copyfile(reference / name, package / name)
revision = subprocess.check_output(
    ["git", "-C", str(reference), "rev-parse", "HEAD"], text=True
).strip()
(package / "UPSTREAM_REVISION").write_text(revision + "\n")
for directory in (package / "src", package / "include"):
    for path in directory.rglob("*"):
        if path.suffix not in (".h", ".hpp", ".cpp"):
            continue
        source = path.read_text()
        source = source.replace(
            "tf2_ros/transform_broadcaster.h>", "tf2_ros/transform_broadcaster.hpp>"
        )
        source = source.replace("#include <Python.h>\n", "")
        path.write_text(source)
# Match our driver's uint32 ring exactly; only the Unitree point type changes.
path = package / "src/preprocess.h"
source = path.read_text()
start = source.index("namespace unilidar_ros")
end = source.index("namespace hesai_ros", start)
source = (
    source[:start]
    + source[start:end].replace("std::uint16_t", "std::uint32_t")
    + source[end:]
)
path.write_text(source)
# Bound ROS queues and trajectory memory for an interactive desktop session.
path = package / "src/laserMapping.cpp"
source = (
    path.read_text()
    .replace(
        "(imu_topic, 200000, imu_cbk)",
        "(imu_topic, rclcpp::SensorDataQoS().keep_last(2000), imu_cbk)",
    )
    .replace(", 100000);", ", 10);")
)
source = source.replace(
    "path.poses.emplace_back(msg_body_pose);",
    "path.poses.emplace_back(msg_body_pose);\n        if (path.poses.size() > 10000) path.poses.erase(path.poses.begin());",
)
source = source.replace(
    "crossmat_list.reserve(feats_down_size);", "crossmat_list.resize(feats_down_size);"
)
source = source.replace(
    "pbody_list.reserve(feats_down_size);", "pbody_list.resize(feats_down_size);"
)
source = source.replace(
    "/******* Publish odometry downsample *******/",
    'RCLCPP_INFO_THROTTLE(logger, *nh->get_clock(), 1000, "LIO: points=%d matches=%d", feats_down_size, effct_feat_num);\n            /******* Publish odometry downsample *******/',
)
source = source.replace(
    "    odomAftMapped.header.frame_id = odom_header_frame_id;",
    """    // Never present inertial-only drift as a usable pose to the GUI.
    static int good_frames = 0;
    const auto velocity = use_imu_as_input ? kf_input.x_.vel : kf_output.x_.vel;
    if (effct_feat_num < 30 || !velocity.allFinite() || velocity.norm() > 3.0) {
        good_frames = 0;
        return;
    }
    if (++good_frames < 15) return;
    odomAftMapped.header.frame_id = odom_header_frame_id;""",
)
path.write_text(source)
for name in ("Log", "PCD"):
    (package / name).mkdir(exist_ok=True)
(package / "CMakeLists.txt").write_text("""cmake_minimum_required(VERSION 3.16)
project(point_lio)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
add_executable(pointlio_mapping src/laserMapping.cpp include/ikd-Tree/ikd_Tree.cpp src/parameters.cpp src/preprocess.cpp src/Estimator.cpp)
target_include_directories(pointlio_mapping PRIVATE include ${PCL_INCLUDE_DIRS})
target_compile_options(pointlio_mapping PRIVATE -O2 -g0 -fexceptions)
target_compile_definitions(pointlio_mapping PRIVATE ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/" MP_PROC_NUM=2 MP_EN)
target_link_libraries(pointlio_mapping ${PCL_LIBRARIES} Eigen3::Eigen OpenMP::OpenMP_CXX rclcpp::rclcpp sensor_msgs::sensor_msgs geometry_msgs::geometry_msgs nav_msgs::nav_msgs visualization_msgs::visualization_msgs tf2_ros::tf2_ros pcl_conversions::pcl_conversions)
install(TARGETS pointlio_mapping DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz_cfg DESTINATION share/${PROJECT_NAME})
ament_package()
""")
params = yaml.safe_load((package / "config/unilidar_l2.yaml").read_text())["/**"][
    "ros__parameters"
]
params.update(
    {
        "odom_header_frame_id": "unitree_lio_origin",
        "odom_child_frame_id": "unitree_lio_body",
        "use_imu_as_input": False,
        "prop_at_freq_of_imu": True,
        "check_satu": True,
        "init_map_size": 10,
        "point_filter_num": 1,
        "space_down_sample": True,
        "filter_size_surf": 0.2,
        "filter_size_map": 0.2,
        "cube_side_length": 1000.0,
        "runtime_pos_log_enable": False,
    }
)
params["pcd_save"]["pcd_save_en"] = False
# Publish pose per cloud: IMU-rate output and unbounded map caching are unnecessary here.
params["odometry"]["publish_odometry_without_downsample"] = False
(package / "config/aurora_l2.yaml").write_text(
    yaml.safe_dump({"/**": {"ros__parameters": params}}, sort_keys=False)
)
(package / "launch/aurora_l2.launch.py").write_text(
    """from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

import uuid

def generate_launch_description():
    root=Path(get_package_share_directory("point_lio"))
    origin="unitree_lio_origin_"+uuid.uuid4().hex[:16]
    remaps=[(old,"/unitree_lio/"+new) for old,new in [
        ("/aft_mapped_to_init","odometry"),("/path","path"),
        ("/cloud_registered","cloud_registered"),("/cloud_registered_body","cloud_body"),
        ("/cloud_effected","cloud_effected"),("/Laser_map","map")]]
    return LaunchDescription([
        DeclareLaunchArgument("rviz",default_value="true"),
        Node(package="point_lio",executable="pointlio_mapping",name="unitree_point_lio",output="screen",parameters=[str(root/"config/aurora_l2.yaml"),{"odom_header_frame_id":origin}],remappings=remaps),
        Node(package="tf2_ros",executable="static_transform_publisher",arguments=["--frame-id","unitree_lio_origin","--child-frame-id",origin]),
        Node(package="rviz2",executable="rviz2",arguments=["-d",str(root/"rviz_cfg/aurora_lio.rviz")],condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
"""
)
rviz = {
    "Panels": [
        {"Class": "rviz_common/Displays", "Name": "Displays"},
        {"Class": "rviz_common/Views", "Name": "Views"},
    ],
    "Visualization Manager": {
        "Class": "",
        "Global Options": {
            "Fixed Frame": "unitree_lio_origin",
            "Frame Rate": 30,
            "Background Color": "48; 48; 48",
        },
        "Displays": [
            {
                "Class": "rviz_default_plugins/Grid",
                "Name": "Cuadricula 1 m",
                "Enabled": True,
                "Cell Size": 1,
                "Plane": "XY",
            },
            {
                "Class": "rviz_default_plugins/Axes",
                "Name": "INICIO (0,0,0)",
                "Enabled": True,
                "Reference Frame": "unitree_lio_origin",
                "Length": 0.5,
                "Radius": 0.015,
            },
            {
                "Class": "rviz_default_plugins/Axes",
                "Name": "Sensor IMU - posicion estimada",
                "Enabled": True,
                "Reference Frame": "unitree_lio_body",
                "Length": 0.3,
                "Radius": 0.012,
            },
            {
                "Class": "rviz_default_plugins/Path",
                "Name": "Recorrido estimado",
                "Enabled": True,
                "Topic": {
                    "Value": "/unitree_lio/path",
                    "Reliability Policy": "Reliable",
                    "Durability Policy": "Volatile",
                    "Depth": 10,
                },
                "Color": "0; 255; 100",
                "Line Style": "Billboards",
                "Line Width": 0.02,
            },
            {
                "Class": "rviz_default_plugins/PointCloud2",
                "Name": "Nube registrada LIO",
                "Enabled": True,
                "Topic": {
                    "Value": "/unitree_lio/cloud_registered",
                    "Reliability Policy": "Best Effort",
                    "Durability Policy": "Volatile",
                    "Depth": 10,
                },
                "Decay Time": 3,
                "Style": "Points",
                "Size (Pixels)": 2,
                "Position Transformer": "XYZ",
                "Color Transformer": "Intensity",
                "Channel Name": "intensity",
            },
        ],
        "Tools": [
            {"Class": "rviz_default_plugins/" + name}
            for name in ("MoveCamera", "Select", "FocusCamera")
        ],
        "Views": {
            "Current": {
                "Class": "rviz_default_plugins/Orbit",
                "Distance": 5,
                "Pitch": 0.5,
                "Yaw": 0.8,
                "Target Frame": "unitree_lio_origin",
            }
        },
    },
    "Window Geometry": {"Width": 1200, "Height": 800},
}
(package / "rviz_cfg/aurora_lio.rviz").write_text(yaml.safe_dump(rviz, sort_keys=False))
if params["cube_side_length"] <= 3 * params["mapping"]["det_range"]:
    raise ValueError(
        "El mapa debe superar 3 veces el alcance para conservar referencias."
    )
for source in package.rglob("*"):
    target = destination / source.relative_to(package)
    if source.is_dir():
        target.mkdir(parents=True, exist_ok=True)
    else:
        target.parent.mkdir(parents=True, exist_ok=True)
        if not target.exists() or target.read_bytes() != source.read_bytes():
            shutil.copy2(source, target)
staging.cleanup()
print("Point-LIO preparado:", destination, "revision", revision)
