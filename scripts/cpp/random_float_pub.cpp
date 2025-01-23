#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <cstdlib>
#include <ctime>

float generateRandomFloat(float min, float max) {
    float random = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    return min + random * (max - min);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "random_float_pub");
    ros::NodeHandle nh;

    ros::Publisher random_float_pub = nh.advertise<std_msgs::Float32>("random_float_topic", 10);

    srand(static_cast<unsigned int>(time(0)));

    ros::Rate loop_rate(10); //10Hz

    while (ros::ok()) {
        float random_value = generateRandomFloat(0.0, 10000.0); // 0.0에서 100.0 사이의 값

        std_msgs::Float32 msg;
        msg.data = random_value;

        ROS_INFO("Publishing random float: %f", random_value);
        random_float_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
