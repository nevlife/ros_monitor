#include <ros/ros.h>
#include <ros/master.h>
#include <xmlrpcpp/XmlRpc.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <unistd.h>

// 🛠 특정 PID의 CPU 사용 시간 가져오기
long long getProcCPUTime(int pid) {
    std::ifstream file("/proc/" + std::to_string(pid) + "/stat"); // 경로 수정
    std::string line;

    if (std::getline(file, line)) {
        std::istringstream iss(line);
        std::vector<std::string> tokens((std::istream_iterator<std::string>(iss)),
                                         std::istream_iterator<std::string>());

        if (tokens.size() > 14) {
            long long userTime = std::stoll(tokens[13]);
            long long kernelTime = std::stoll(tokens[14]);
            return userTime + kernelTime;
        }
    }
    return -1;
}

// 🛠 실행 중인 ROS 노드 목록 가져오기
std::vector<std::string> getRosNodes() {
    std::vector<std::string> nodes;
    ros::master::getNodes(nodes);  // ROS 마스터에서 실행 중인 노드 가져오기
    return nodes;
}

// 🛠 특정 노드의 API URI 가져오기 (`lookupNode` 사용)
std::string getNodeApiUri(const std::string& node_name) {
    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = ros::this_node::getName();  // 현재 노드 이름
    args[1] = node_name;  // 대상 노드

    // `lookupNode`는 **ROS 노드**에 대해서만 실행 가능 (서비스 X)
    if (!ros::master::execute("lookupNode", args, result, payload, true)) {
        ROS_WARN("Failed to lookupNode for %s (It might be a service, not a node)", node_name.c_str());
        return "";
    }
    return result[2];  // API URI 반환
}

// 🛠 특정 노드의 PID 가져오기 (XML-RPC `getPid` 호출)
int getNodePid(const std::string& node_api) {
    if (node_api.empty()) return -1;

    std::string host;
    int port;

    // API URI에서 호스트와 포트 추출
    size_t pos = node_api.find("http://");
    if (pos == 0) {
        size_t port_start = node_api.find(":", 7);
        size_t port_end = node_api.find("/", port_start);
        host = node_api.substr(7, port_start - 7);
        port = std::stoi(node_api.substr(port_start + 1, port_end - port_start - 1));
    } else {
        ROS_WARN("Invalid API URI format: %s", node_api.c_str());
        return -1;
    }

    // XML-RPC 클라이언트 생성
    XmlRpc::XmlRpcClient client(host.c_str(), port, "/");

    XmlRpc::XmlRpcValue request, response;
    request[0] = ros::this_node::getName();  // 호출자 이름 설정

    // `getPid` 요청 실행
    if (!client.execute("getPid", request, response)) {
        ROS_WARN("Failed to get PID from %s", node_api.c_str());
        return -1;
    }

    // 응답이 정상적으로 왔는지 확인 후 반환
    if (response.getType() == XmlRpc::XmlRpcValue::TypeArray && response.size() > 2) {
        return static_cast<int>(response[2]);
    }

    return -1;
}

// 🛠 특정 노드의 CPU 사용률 측정
double getNodeCPUUsage(int pid, int interval_ms = 1000) {
    long long prevCPUTime = getProcCPUTime(pid);
    usleep(interval_ms * 1000);  // 지정된 시간 동안 대기
    long long currentCPUTime = getProcCPUTime(pid);

    if (prevCPUTime < 0 || currentCPUTime < 0) {
        return -1.0;
    }

    double usage = 100.0 * (currentCPUTime - prevCPUTime) / (double)sysconf(_SC_CLK_TCK) / (interval_ms / 1000.0);
 
    return usage;
}

// 🛠 ROS 노드의 API URI 및 PID를 가져와 CPU 사용률 출력
void printRosNodesWithPidAndCPUUsage() {
    std::vector<std::string> nodes = getRosNodes();

    if (nodes.empty()) {
        ROS_WARN("No active ROS nodes found.");
    } else {
        ROS_INFO("Fetching API URI, PID, and CPU usage for active ROS nodes:");

        for (const auto& node : nodes) {
            std::string api_uri = getNodeApiUri(node);
            if (api_uri.empty()) {
                ROS_WARN("Node: %s - API URI not found (It might be a service, not a node)", node.c_str());
                continue;
            }

            int pid = getNodePid(api_uri);
            if (pid > 0) {
                double cpuUsage = getNodeCPUUsage(pid);
                if (cpuUsage >= 0) {
                    ROS_INFO("Node: %s - API URI: %s - PID: %d - CPU Usage: %f%%", 
                              node.c_str(), api_uri.c_str(), pid, cpuUsage);
                } else {
                    ROS_WARN("Node: %s - Failed to retrieve CPU usage", node.c_str());
                }
            } else {
                ROS_WARN("Node: %s - Failed to retrieve PID", node.c_str());
            }
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_nodes_pid_usage_monitor");
    ros::NodeHandle nh;

    ros::Duration(1.0).sleep();  // ROS Master 연결 대기

    printRosNodesWithPidAndCPUUsage();  // 현재 실행 중인 ROS 노드들의 API URI, PID 및 CPU 사용률 출력

    return 0;
}
