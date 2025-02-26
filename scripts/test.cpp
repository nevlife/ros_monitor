#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nvml.h>
#include <sstream>
#include <cstdlib>
#include <unistd.h>

int main(int argc, char** argv)
{
    // ROS 초기화
    ros::init(argc, argv, "gpu_monitor_node");
    ros::NodeHandle nh;
    ros::Publisher gpu_pub = nh.advertise<std_msgs::String>("gpu_usage", 10);
    ros::Rate loop_rate(1); // 1Hz 업데이트

    // NVML 초기화
    nvmlReturn_t result = nvmlInit_v2();
    if (result != NVML_SUCCESS) {
        ROS_ERROR("NVML 초기화 실패: %d", result);
        return 1;
    }

    // 사용 가능한 GPU 개수 가져오기
    unsigned int deviceCount = 0;
    result = nvmlDeviceGetCount_v2(&deviceCount);
    if (result != NVML_SUCCESS) {
        ROS_ERROR("GPU 개수 가져오기 실패: %d", result);
        nvmlShutdown();
        return 1;
    }

    // GPU 핸들 배열 할당 (C++에서는 malloc 시 캐스트 필요)
    nvmlDevice_t *deviceHandles = (nvmlDevice_t*)malloc(sizeof(nvmlDevice_t) * deviceCount);
    for (unsigned int i = 0; i < deviceCount; i++) {
        result = nvmlDeviceGetHandleByIndex_v2(i, &deviceHandles[i]);
        if (result != NVML_SUCCESS) {
            ROS_ERROR("GPU %u 핸들 가져오기 실패: %d", i, result);
        }
    }

    while (ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Detected " << deviceCount << " GPU(s).\n";

        for (unsigned int i = 0; i < deviceCount; i++)
        {
            char name[64];
            result = nvmlDeviceGetName(deviceHandles[i], name, sizeof(name));
            if (result != NVML_SUCCESS) {
                snprintf(name, sizeof(name), "Unknown");
            }

            nvmlUtilization_t utilization;
            result = nvmlDeviceGetUtilizationRates(deviceHandles[i], &utilization);
            if (result != NVML_SUCCESS) {
                ss << "GPU " << i << " (" << name << "): 사용률 가져오기 실패\n";
                continue;
            }

            ss << "GPU " << i << " (" << name << "):\n"
               << "  Core Utilization: " << utilization.gpu << "%\n"
               << "  Memory Utilization: " << utilization.memory << "%\n";

            // 프로세스 정보 가져오기 (NVML v2가 없으면 nvmlDeviceGetComputeRunningProcesses 사용)
            unsigned int infoCount = 64;
            nvmlProcessInfo_t infos[64];
            result = nvmlDeviceGetComputeRunningProcesses(deviceHandles[i], &infoCount, infos);
            if (result == NVML_SUCCESS && infoCount > 0) {
                ss << "  Running Processes:\n";
                ss << "    PID      GPU Memory (MiB)\n";
                for (unsigned int j = 0; j < infoCount; j++) {
                    unsigned int pid = infos[j].pid;
                    unsigned long long usedMem = infos[j].usedGpuMemory;
                    ss << "    " << pid << "      " << (usedMem / (1024 * 1024)) << "\n";
                }
            } else {
                ss << "  No running compute processes.\n";
            }
            ss << "\n";
        }

        msg.data = ss.str();
        gpu_pub.publish(msg);
        ROS_INFO_STREAM(msg.data);

        ros::spinOnce();
        loop_rate.sleep();
    }

    free(deviceHandles);
    nvmlShutdown();
    return 0;
}
