// Aurora IPC bridge. SDK is obtained separately under its upstream BSD license.
#include "unitree_lidar_sdk.h"
#include <arpa/inet.h>
#include <sys/file.h>
#include <fcntl.h>
#include <csignal>
#include <limits>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>
#include <unistd.h>
using namespace unilidar_sdk2;

static volatile sig_atomic_t running = 1;
static void stop(int) { running = 0; }
template<class T> static void append(std::vector<char>& out, T value) {
    auto p = reinterpret_cast<const char*>(&value);
    out.insert(out.end(), p, p + sizeof(T));
}
int main(int argc, char** argv) {
    try {
        std::string ip = "192.168.1.62", pc = "192.168.1.2";
        int port = 6101, local_port = 6201, scans = 18;
        bool system_time = true;
        for (int i = 1; i < argc; i += 2) {
            if (i + 1 >= argc) throw std::runtime_error("Each option needs a value");
            std::string key = argv[i], value = argv[i+1];
            if (key == "--sensor-ip") ip = value;
            else if (key == "--local-ip") pc = value;
            else if (key == "--sensor-port") port = std::stoi(value);
            else if (key == "--local-port") local_port = std::stoi(value);
            else if (key == "--scans") scans = std::stoi(value);
            else if (key == "--system-time") system_time = std::stoi(value) != 0;
            else throw std::runtime_error("Unknown option: " + key);
        }
        in_addr addr;
        if (!inet_pton(AF_INET, ip.c_str(), &addr) || !inet_pton(AF_INET, pc.c_str(), &addr))
            throw std::runtime_error("Invalid IPv4 address");
        if (port < 1 || port > 65535 || local_port < 1 || local_port > 65535 || scans < 1 || scans > 65535)
            throw std::runtime_error("Invalid port or scan count");
        uint16_t endian = 1;
        if (*reinterpret_cast<char*>(&endian) != 1) throw std::runtime_error("Little-endian host required");
        // Prevent two Aurora bridges from claiming the same UDP port.
        std::string lock_path = "/tmp/aurora-unitree-udp-" + std::to_string(local_port) + ".lock";
        int lock_fd = open(lock_path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
        if (lock_fd < 0 || flock(lock_fd, LOCK_EX | LOCK_NB)) throw std::runtime_error("UDP port owned by another Aurora bridge");
        // Refuse a port already bound by ROS or another producer. Do not enable
        // SO_REUSEADDR: sharing this socket would split/corrupt acquisition.
        int check_fd = socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in endpoint{}; endpoint.sin_family = AF_INET;
        endpoint.sin_port = htons(local_port);
        inet_pton(AF_INET, pc.c_str(), &endpoint.sin_addr);
        if (check_fd < 0 || bind(check_fd, reinterpret_cast<sockaddr*>(&endpoint), sizeof(endpoint)) != 0)
            throw std::runtime_error("Local UDP IP/port unavailable (another driver or missing interface)");
        close(check_fd);
        // The SDK may log to stdout. Reserve the original stdout for IPC only.
        int ipc_fd = dup(STDOUT_FILENO);
        dup2(STDERR_FILENO, STDOUT_FILENO);
        UnitreeLidarReader* reader = createUnitreeLidarReader();
        if (!reader) throw std::runtime_error("SDK reader creation failed");
        if (reader->initializeUDP(port, ip, local_port, pc, scans, system_time, 0.f, std::numeric_limits<float>::max()))
            throw std::runtime_error("SDK UDP initialization failed (check IP/interface and port ownership)");
        reader->startLidarRotation();
        reader->setLidarWorkMode(0);
        signal(SIGINT, stop); signal(SIGTERM, stop); signal(SIGPIPE, SIG_IGN);
        // Retain SDK logging redirection; write payloads through ipc_fd.
        while (running) {
            int result = reader->runParse();
            PointCloudUnitree cloud;
            if (result != LIDAR_POINT_DATA_PACKET_TYPE || !reader->getPointCloud(cloud)) {
                usleep(100); continue;
            }
            if (cloud.points.size() > 4000000) throw std::runtime_error("Cloud exceeds IPC limit");
            std::vector<char> out{'U','T','L','2'};
            append<uint32_t>(out, 2); append<double>(out, cloud.stamp);
            append<uint64_t>(out, cloud.id); append<uint32_t>(out, cloud.points.size());
            append<int64_t>(out, -1); // SDK timestamp is natively double, not integer ns.
            const std::string frame_id = "unilidar_lidar";
            append<uint16_t>(out, frame_id.size());
            out.insert(out.end(), frame_id.begin(), frame_id.end());
            for (const auto& p : cloud.points) {
                append<double>(out, p.x); append<double>(out, p.y); append<double>(out, p.z);
                append<double>(out, p.intensity); append<double>(out, p.time); append<uint32_t>(out, p.ring);
            }
            size_t offset = 0;
            while (offset < out.size() && running) {
                ssize_t n = write(ipc_fd, out.data() + offset, out.size() - offset);
                if (n < 0 && errno == EINTR) continue;
                if (n <= 0) { running = 0; break; }
                offset += static_cast<size_t>(n);
            }
        }
        // Closing Aurora releases the PC-side UDP socket but deliberately does
        // not send stopLidarRotation(): the Unitree keeps scanning until power
        // is removed or its own control interface stops it.
        reader->closeUDP();
        close(ipc_fd); close(lock_fd);
        return 0;
    } catch (const std::exception& e) {
        fprintf(stderr, "Unitree bridge: %s\n", e.what()); return 1;
    }
}
