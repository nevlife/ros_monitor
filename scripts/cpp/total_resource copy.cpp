#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <nvml.h>
#include <sys/sysinfo.h>
#include <fstream>
#include <vector>
#include <string>
#include <iostream>
#include <numeric>

class SystemStatusMonitor {
public:
    SystemStatusMonitor() : rate(1.0), gpu_initialized(false) {
        total_resource_pub = nh.advertise<std_msgs::Float32MultiArray>("/total_resource", 100);

        // Initialize NVML for GPU monitoring
        nvmlReturn_t result = nvmlInit();
        if (result == NVML_SUCCESS) {
            nvmlDeviceGetHandleByIndex(0, &gpu_handle);
            char gpu_name[NVML_DEVICE_NAME_BUFFER_SIZE];
            nvmlDeviceGetName(gpu_handle, gpu_name, NVML_DEVICE_NAME_BUFFER_SIZE);
            gpu_initialized = true;
            ROS_INFO("NVML initialized GPU: %s", gpu_name);
        } else {
            ROS_WARN("GPU initialization failed: %s", nvmlErrorString(result));
        }
    }

    ~SystemStatusMonitor() {
        if (gpu_initialized) {
            nvmlShutdown();
            ROS_INFO("GPU resources released.");
        }
    }

    void run() {
        while (ros::ok()) {
            std_msgs::Float32MultiArray msg;

            // Collect system information
            std::vector<float> cpu_info = getCpuUsage();
            std::vector<float> mem_info = getMemoryUsage();
            std::vector<float> gpu_info = getGpuInfo();

            // Populate message
            msg.data.insert(msg.data.end(), cpu_info.begin(), cpu_info.end());
            msg.data.insert(msg.data.end(), mem_info.begin(), mem_info.end());
            msg.data.insert(msg.data.end(), gpu_info.begin(), gpu_info.end());

            total_resource_pub.publish(msg);

            // Log system information
            ROS_INFO("CPU: %.2f%%, Temp: %.2fC, Freq: %.2f GHz", cpu_info[0], cpu_info[2], cpu_info[1]);
            ROS_INFO("Mem: %.2f/%.2f MB (%.2f%%)", mem_info[0], mem_info[1], mem_info[2]);

            if (gpu_initialized) {
                ROS_INFO("GPU Usage: %.2f%%, Mem: %.2f/%.2f MB (%.2f%%), Temp: %.2fC",
                         gpu_info[0], gpu_info[1], gpu_info[2], gpu_info[3], gpu_info[4]);
            }

            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh;
    ros::Publisher total_resource_pub;
    ros::Rate rate;

    nvmlDevice_t gpu_handle;
    bool gpu_initialized;
    
    float getCpuTemperature() {
        std::ifstream temp_file("/sys/class/thermal/thermal_zone0/temp");
        float temp;
        if (temp_file >> temp) {
            return temp / 1000.0;  // 보통 단위가 milli-degree이므로 변환 필요
        }
        return 0.0;  // 읽기 실패 시 0 반환
    }

    float getCpuFrequency() {
        std::ifstream freq_file("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq");
        float freq;
        if (freq_file >> freq) {
            return freq / 1000000.0;  // kHz -> GHz 변환
        }
        return 0.0;
    }

    std::vector<float> getCpuUsage() {
        static std::vector<long> prev_cpu_times(7, 0);
        
        std::ifstream cpu_stat_file("/proc/stat");
        std::string line;
        std::getline(cpu_stat_file, line);
        cpu_stat_file.close();

        std::istringstream iss(line);
        std::string cpu_label;
        long user, nice, system, idle, iowait, irq, softirq;
        iss >> cpu_label >> user >> nice >> system >> idle >> iowait >> irq >> softirq;

        std::vector<long> cpu_times = {user, nice, system, idle, iowait, irq, softirq};

        // 이전과의 차이 계산
        long total = std::accumulate(cpu_times.begin(), cpu_times.end(), 0L);
        long prev_total = std::accumulate(prev_cpu_times.begin(), prev_cpu_times.end(), 0L);
        long total_diff = total - prev_total;

        long idle_total = idle + iowait;
        long prev_idle_total = prev_cpu_times[3] + prev_cpu_times[4];
        long idle_diff = idle_total - prev_idle_total;

        // CPU 사용률 계산
        float usage = (total_diff - idle_diff) * 100.0 / total_diff;

        // 이전 CPU 시간 업데이트
        prev_cpu_times = cpu_times;

        float freq = getCpuFrequency();
        float temp = getCpuTemperature();

        return {usage, freq, temp};
    }



    std::vector<float> getMemoryUsage() {
        struct sysinfo info;
        sysinfo(&info);

        float total_mem = info.totalram / (1024.0 * 1024.0);
        float used_mem = (info.totalram - info.freeram) / (1024.0 * 1024.0);
        float mem_usage = (used_mem / total_mem) * 100.0;

        return {used_mem, total_mem, mem_usage};
    }

    std::vector<float> getGpuInfo() {
        if (!gpu_initialized) {
            return {0, 0, 0, 0, 0};
        }

        nvmlUtilization_t utilization;
        nvmlMemory_t memory;
        unsigned int temperature;

        nvmlDeviceGetUtilizationRates(gpu_handle, &utilization);
        nvmlDeviceGetMemoryInfo(gpu_handle, &memory);
        nvmlDeviceGetTemperature(gpu_handle, NVML_TEMPERATURE_GPU, &temperature);

        float gpu_usage = utilization.gpu;
        float gpu_mem_used = memory.used / (1024.0 * 1024.0);
        float gpu_mem_total = memory.total / (1024.0 * 1024.0);
        float gpu_mem_usage = (gpu_mem_used / gpu_mem_total) * 100.0;
        float gpu_temp = temperature;

        return {gpu_usage, gpu_mem_used, gpu_mem_total, gpu_mem_usage, gpu_temp};
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "total_resource");

    SystemStatusMonitor monitor;

    try {
        monitor.run();
    } catch (const ros::Exception& e) {
        ROS_ERROR("ROS Exception: %s", e.what());
    }

    return 0;
}