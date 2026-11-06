"""Exercise actual bridge entry points using a synthetic SDK and ROS runtime."""

import io
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from unitree_protocol import read_frame

ROOT = Path(__file__).resolve().parents[1]

FAKE_SDK = r"""
#pragma once
#include <cstdint>
#include <vector>
#include <string>
#include <csignal>
#include <cstdio>
namespace unilidar_sdk2 {
const int LIDAR_POINT_DATA_PACKET_TYPE=2;
struct PointUnitree { float x,y,z,intensity,time; uint32_t ring; };
struct PointCloudUnitree { double stamp; uint32_t id; std::vector<PointUnitree> points; };
class UnitreeLidarReader {
 int step=0;
public:
 int initializeUDP(unsigned short, std::string, unsigned short, std::string, uint16_t scans, bool, float, float) {
  printf("SDK log to stdout scans=%u\n",scans); return 0;
 }
 void startLidarRotation() {} void stopLidarRotation() {} void closeUDP() {} void setLidarWorkMode(uint32_t) {}
 int runParse() { if(step<2) return LIDAR_POINT_DATA_PACKET_TYPE; raise(SIGTERM); return 0; }
 bool getPointCloud(PointCloudUnitree& c) { c.stamp=123.25+step; c.id=17+step;
   c.points={{1.25f,2.f,3.f,99.f,.125f,7},{4.f,5.f,6.f,100.f,.25f,8}}; step++; return true; }
};
inline UnitreeLidarReader* createUnitreeLidarReader() {return new UnitreeLidarReader;}
}
"""

ROS_STUB = r"""
import struct
from types import SimpleNamespace as N
calls=[]
def init_node(*a,**kw): print('ROS initialization log')
def Subscriber(topic, kind, callback, **kwargs): calls.append((kind,callback))
def spin():
    fields=[N(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z','intensity','time'))]
    fields.append(N(name='ring',offset=20,datatype=4,count=1))
    stamp=N(to_sec=lambda:25.5)
    cloud=N(fields=fields,width=2,height=1,point_step=24,row_step=48,is_bigendian=False,
      header=N(frame_id='unilidar_lidar',stamp=stamp),
      data=struct.pack('<fffffHxxfffffHxx',1,2,3,4,.125,7,5,6,7,8,.25,9))
    imu=N(header=N(frame_id='unilidar_imu',stamp=stamp),orientation=N(x=0,y=0,z=0,w=1),orientation_covariance=[0]*9)
    for kind,callback in calls:
        callback(imu if kind.__name__=='Imu' else cloud)
"""


class NativeBridgeTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which("g++"), "g++ no disponible")
    def test_cpp_output_preserves_full_sdk_clouds_and_redirects_logs(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            (d / "unitree_lidar_sdk.h").write_text(FAKE_SDK)
            binary = d / "bridge"
            subprocess.run(
                [
                    "g++",
                    "-std=c++17",
                    "-I" + str(d),
                    str(ROOT / "native/unitree_bridge.cpp"),
                    "-o",
                    str(binary),
                ],
                check=True,
                capture_output=True,
            )
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.bind(("127.0.0.1", 0))
                port = sock.getsockname()[1]
            result = subprocess.run(
                [
                    str(binary),
                    "--local-ip",
                    "127.0.0.1",
                    "--sensor-ip",
                    "127.0.0.2",
                    "--local-port",
                    str(port),
                    "--scans",
                    "23",
                ],
                capture_output=True,
                timeout=3,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn(b"SDK log to stdout scans=23", result.stderr)
            stream = io.BytesIO(result.stdout)
            for sequence in (17, 18):
                frame = read_frame(stream)
                self.assertEqual(frame.cloud_id, sequence)
                self.assertEqual(frame.stamp, 123.25 + sequence - 17)
                np.testing.assert_array_equal(frame.xyz, [[1.25, 2, 3], [4, 5, 6]])
                np.testing.assert_array_equal(frame.fields["ring"], [7, 8])
                np.testing.assert_array_equal(frame.fields["time"], [0.125, 0.25])
            self.assertEqual(stream.read(), b"")

    def test_real_bridge_rejects_port_already_owned_by_another_driver(self):
        binary = ROOT / "build/unitree/unitree_bridge"
        if not binary.is_file():
            self.skipTest("Compila primero ./install_unitree_sdk.sh")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind(("127.0.0.1", 0))
            result = subprocess.run(
                [
                    str(binary),
                    "--local-ip",
                    "127.0.0.1",
                    "--local-port",
                    str(sock.getsockname()[1]),
                ],
                capture_output=True,
                timeout=3,
            )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(b"Local UDP IP/port unavailable", result.stderr)


class RosBridgeTests(unittest.TestCase):
    def test_separate_ros_python_emits_complete_cloud_and_imu_diagnostic(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            (d / "rospy.py").write_text(ROS_STUB)
            (d / "rclpy").mkdir()
            (d / "rclpy/__init__.py").write_text(
                "import rospy\ndef init(): print('ROS initialization log')\ndef shutdown(): pass\nclass Node:\n def create_subscription(self,kind,topic,callback,qos): rospy.calls.append((kind,callback))\n def destroy_node(self): pass\ndef create_node(name): return Node()\ndef spin(node): rospy.spin()\n"
            )
            (d / "rclpy/qos.py").write_text("qos_profile_sensor_data=None\n")
            (d / "sensor_msgs").mkdir()
            (d / "sensor_msgs/__init__.py").write_text("")
            (d / "sensor_msgs/msg.py").write_text(
                "class PointCloud2: pass\nclass Imu: pass\n"
            )
            env = dict(os.environ, PYTHONPATH=str(d))
            for transport in ("ros1", "ros2"):
                with self.subTest(transport=transport):
                    result = subprocess.run(
                        [
                            sys.executable,
                            str(ROOT / "scripts/ros_unitree_bridge.py"),
                            "--ros",
                            transport,
                        ],
                        capture_output=True,
                        env=env,
                        timeout=3,
                    )
                    self.assertEqual(result.returncode, 0, result.stderr)
                    frame = read_frame(io.BytesIO(result.stdout))
                    self.assertEqual(frame.stamp, 25.5)
                    np.testing.assert_array_equal(frame.xyz, [[1, 2, 3], [5, 6, 7]])
                    np.testing.assert_array_equal(frame.fields["ring"], [7, 9])
                    self.assertIn(b"ROS initialization log", result.stderr)
                    self.assertIn(b"AURORA_IMU", result.stderr)


if __name__ == "__main__":
    unittest.main()
