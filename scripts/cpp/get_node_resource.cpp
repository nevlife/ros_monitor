#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rosgraph_msgs/Clock.h>
#include <topic_tools/shape_shifter.h>
#include <ros/master.h>
#include <map>
#include <vector>
#include <deque>
#include <mutex>
#include <fstream>
#include <jsoncpp/json/json.h>

class ROSTopicMetrics {
public:
    ROSTopicMetrics(const std::string& topic, int window_size = 10)
        : topic_(topic), window_size_(window_size), bytes_received_(0), start_time_(0), end_time_(0) {
        sub_ = nh_.subscribe(topic, 1000, &ROSTopicMetrics::callback, this);
    }

    void callback(const ros::MessageEvent<topic_tools::ShapeShifter const>& event) {
        ros::Time now = ros::Time::now();
        std::lock_guard<std::mutex> lock(hz_mutex_);
        times_.push_back(now.toSec());
        if (times_.size() > window_size_) times_.pop_front();
        
        std::lock_guard<std::mutex> lock_bw(bw_mutex_);
        if (start_time_ == 0) start_time_ = now.toSec();
        end_time_ = now.toSec();
        
        auto msg = event.getMessage();
        int msg_size = msg->serializationLength();
        bytes_received_ += msg_size;
    }

    double getHz() {
        std::lock_guard<std::mutex> lock(hz_mutex_);
        if (times_.size() < 2) return 0.0;
        double sum = 0.0;
        for (size_t i = 1; i < times_.size(); ++i) {
            sum += times_[i] - times_[i - 1];
        }
        return (sum > 0.0) ? (times_.size() - 1) / sum : 0.0;
    }

    double getBandwidth() {
        std::lock_guard<std::mutex> lock(bw_mutex_);
        double duration = end_time_ - start_time_;
        return (duration > 0) ? bytes_received_ / duration : 0.0;
    }

private:
    ros::NodeHandle nh_;
    std::string topic_;
    ros::Subscriber sub_;
    std::deque<double> times_;
    std::mutex hz_mutex_, bw_mutex_;
    int window_size_;
    double bytes_received_;
    double start_time_, end_time_;
};

class MetricsManager {
public:
    MetricsManager() {
        loadTopics();
    }

    void loadTopics() {
        ros::master::V_TopicInfo master_topics;
        ros::master::getTopics(master_topics);
        
        std::ifstream file("../cfg/topic_lst.yaml");
        std::vector<std::string> topic_list;
        if (file) {
            std::string line;
            while (std::getline(file, line)) {
                topic_list.push_back(line);
            }
        }
        file.close();

        for (const auto& topic : topic_list) {
            monitors_[topic] = std::make_shared<ROSTopicMetrics>(topic);
        }
    }

    Json::Value getMetrics() {
        Json::Value root;
        for (const auto& [topic, monitor] : monitors_) {
            Json::Value data;
            data["hz"] = monitor->getHz();
            data["bw"] = monitor->getBandwidth();
            root.append(data);
        }
        return root;
    }

private:
    std::map<std::string, std::shared_ptr<ROSTopicMetrics>> monitors_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "topic_hzbw");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("/topic_hzbw", 100);
    
    MetricsManager manager;
    ros::Rate rate(1);

    while (ros::ok()) {
        Json::Value metrics = manager.getMetrics();
        Json::StreamWriterBuilder writer;
        std::string json_data = Json::writeString(writer, metrics);
        
        std_msgs::String msg;
        msg.data = json_data;
        pub.publish(msg);
        
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
