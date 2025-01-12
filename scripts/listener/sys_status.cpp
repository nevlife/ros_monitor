#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <stdexcept>
#include <nvml.h>


float getCpuUsage() {
    /*cpu usage */
    std::ifstream procStatus("/proc/stat"); //read prev status

    static long prevIdle = 0;
    static long prevTotal = 0;

    long user, nice, sys, idle, iowait, irq, softirq, steal;

    if (!procStatus.is_open()) {
        ROS_WARN("failed to read /proc/stat");
        return 0.0;
    }

    std::string line;
    std::getline(procStatus, line);
    procStatus.close();

    std::istringstream ss(line);
    std::string cpu;
    ss >> cpu >> user >> nice >> sys >> idle >> iowait >> irq >> softirq >> steal;

    long curridleTime = idle + iowait;
    long currtotalTime = user + nice + sys + idle + iowait + irq + softirq + steal;

    long diffIdle = curridleTime - prevIdle;
    long diffTotal = currtotalTime - prevTotal;

    if (prevIdle == 0 && prevTotal == 0) {
        prevIdle = curridleTime;
        prevTotal = currtotalTime;
    }

    if (diffTotal == 0)
        return 0.0;

    return (1.0 - (float)diffIdle / diffTotal) * 100.0;
}

// Function to get CPU freq
float getCPUFreq() {
    std::ifstream cpuInfo("/proc/cpuinfo");

    if (!cpuInfo.is_open()) {
        ROS_WARN("failed to read /proc/cpuinfo");
        return 0.0;
    }

    std::string line;
    while (std::getline(cpuInfo, line)) {

        if (line.find("cpu MHz") != std::string::npos) {

            std::istringstream ss(line);
            std::string label;
            float frequency;

            ss >> label >> label >> frequency;

            return frequency / 1000.0; //GHz
        }
    }

    return 0.0;
}

//Fnc to get GPU usage
float getGPUUsage() {
    
    if (nvmlInit(); != NVML_SUCCESS) {
        ROS_WARN("failed to init NVML");
        return 0.0;
    }

    //gpu device headler
    nvmlDevice_t device;
    nvmlUtilization_t utilization;
    float gpuUsage = 0.0;

    try {
        nvmlDeviceGetHandleByIndex(0, &device);
        nvmlDeviceGetUtilizationRates(device, &utilization);
        gpuUsage = utilization.gpu;
    } catch (...) {
        ROS_WARN("Failed to fetch GPU usage");
    }

    nvmlShutdown();

    return gpuUsage;
}

//Fnc to get memory usage
float getMemoryUsage() {
    std::ifstream memInfo("/proc/meminfo", std::ios::in | std::ios::binary);

    if (!memInfo.is_open()) {
        ROS_WARN("failed to read /proc/meminfo");
        return 0.0;
    }

    long totalMemory = 0;
    long freeMemory = 0;
    std::string label;
    while (memInfo >> label) {
        if (label == "MemTotal:")
            memInfo >> totalMemory;
            
        else if
            (label == "MemAvailable:") memInfo >> availableMemory;

        if (totalMemory > 0 && availableMemory > 0)
            break;
    }

    if (totalMemory == 0)
        return 0.0;

    return (1.0 - static_cast<float>(availableMemory) / totalMemory) * 100.0;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "get_sys_status");
    ros::NodeHandle nh;

    ros::Publisher sysStatusPub = nh.advertise<std_msgs::Float32MultiArray>("/get_sys_status", 10);
    ros::Rate rate(1);  //1Hz

    while (ros::ok()) {
        float cpuUsage = getCpuUsage();
        float cpuFrequency = getCPUFreq();
        float gpuUsage = getGPUUsage();
        float memoryUsage = getMemoryUsage();

        //ROS_INFO("CPU: %.2f%%, Freq: %.2f GHz, GPU: %.2f%%, Mem: %.2f%%", cpuUsage, cpuFrequency, gpuUsage, memoryUsage);
        printf("CPU: %.2f%%, Freq: %.2f GHz, GPU: %.2f%%, Memory: %.2f%%\n", cpuUsage, cpuFrequency, gpuUsage, memoryUsage);

        //publish
        std_msgs::Float32MultiArray msg;
        msg.data = {cpuUsage, cpuFrequency, gpuUsage, memoryUsage};
        sysStatusPub.publish(msg);

        rate.sleep();
    }

    return 0;
}
