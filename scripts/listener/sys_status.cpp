#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <stdexcept>
#include <nvml.h>

class SystemStatusPublisher {
public:
    SystemStatusPublisher(ros::NodeHandle& nh)
        : sys_status_pub_(nh.advertise<std_msgs::Float32MultiArray>("/get_sys_status", 10)),
          rate_(1) {  // 1Hz
        // Initialize GPU monitoring (NVML)
        if (nvmlInit() != NVML_SUCCESS) {
            ROS_WARN("Failed to initialize NVML");
            nvml_initialized_ = false;
        } else {
            nvml_initialized_ = true;
        }
    }

    ~SystemStatusPublisher() {
        if (nvml_initialized_) {
            nvmlShutdown();
        }
    }

    void publishSystemStatus() {
        while (ros::ok()) {
            float cpu_usage = getCpuUsage();
            float cpu_frequency = getCPUFreq();
            float gpu_usage = getGPUUsage();
            float memory_usage = getMemoryUsage();

            ROS_INFO("CPU: %.2f%%, Freq: %.2f GHz, GPU: %.2f%%, Mem: %.2f%%", 
                     cpu_usage, cpu_frequency, gpu_usage, memory_usage);

            // Publish the system status
            std_msgs::Float32MultiArray msg;
            msg.data = {cpu_usage, cpu_frequency, gpu_usage, memory_usage};
            sys_status_pub_.publish(msg);

            rate_.sleep();
        }
    }

private:
    ros::Publisher sys_status_pub_;
    ros::Rate rate_;
    bool nvml_initialized_;

    float getCpuUsage() {
        static long prev_idle = 0;
        static long prev_total = 0;

        std::ifstream proc_status("/proc/stat");
        if (!proc_status.is_open()) {
            ROS_WARN("Failed to read /proc/stat");
            return 0.0;
        }

        std::string line;
        std::getline(proc_status, line);
        proc_status.close();

        long user, nice, sys, idle, iowait, irq, softirq, steal;
        std::istringstream ss(line);
        std::string cpu;
        ss >> cpu >> user >> nice >> sys >> idle >> iowait >> irq >> softirq >> steal;

        long curr_idle = idle + iowait;
        long curr_total = user + nice + sys + idle + iowait + irq + softirq + steal;

        long diff_idle = curr_idle - prev_idle;
        long diff_total = curr_total - prev_total;

        prev_idle = curr_idle;
        prev_total = curr_total;

        if (diff_total == 0) {
            return 0.0;
        }

        return (1.0 - static_cast<float>(diff_idle) / diff_total) * 100.0;
    }

    float getCPUFreq() {
        std::ifstream cpu_info("/proc/cpuinfo");
        if (!cpu_info.is_open()) {
            ROS_WARN("Failed to read /proc/cpuinfo");
            return 0.0;
        }

        std::string line;
        while (std::getline(cpu_info, line)) {
            if (line.find("cpu MHz") != std::string::npos) {
                std::istringstream ss(line);
                std::string label;
                float frequency;
                ss >> label >> label >> frequency;
                return frequency / 1000.0; // Convert MHz to GHz
            }
        }

        return 0.0;
    }

    float getGPUUsage() {
        if (!nvml_initialized_) {
            return 0.0;
        }

        nvmlDevice_t device;
        nvmlUtilization_t utilization;

        try {
            nvmlDeviceGetHandleByIndex(0, &device);
            nvmlDeviceGetUtilizationRates(device, &utilization);
            return utilization.gpu;
        } catch (...) {
            ROS_WARN("Failed to fetch GPU usage");
            return 0.0;
        }
    }

    float getMemoryUsage() {
        std::ifstream mem_info("/proc/meminfo", std::ios::in | std::ios::binary);
        if (!mem_info.is_open()) {
            ROS_WARN("Failed to read /proc/meminfo");
            return 0.0;
        }

        long total_memory = 0;
        long available_memory = 0;
        std::string label;

        while (mem_info >> label) {
            if (label == "MemTotal:") {
                mem_info >> total_memory;
            } else if (label == "MemAvailable:") {
                mem_info >> available_memory;
            }

            if (total_memory > 0 && available_memory > 0) {
                break;
            }
        }

        if (total_memory == 0) {
            return 0.0;
        }

        return (1.0 - static_cast<float>(available_memory) / total_memory) * 100.0;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "get_sys_status");
    ros::NodeHandle nh;

    SystemStatusPublisher system_status_publisher(nh);
    system_status_publisher.publishSystemStatus();

    return 0;
}
