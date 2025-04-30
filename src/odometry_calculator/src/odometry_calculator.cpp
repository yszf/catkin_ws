#include <fstream>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <sstream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>

// 计算欧几里得距离
double euclidean_distance_2d(const nav_msgs::Odometry& odom1, const nav_msgs::Odometry& odom2) {
    double dx = odom2.pose.pose.position.x - odom1.pose.pose.position.x;
    double dy = odom2.pose.pose.position.y - odom1.pose.pose.position.y;
    return std::sqrt(dx * dx + dy * dy);
}

double euclidean_distance_3d(const nav_msgs::Odometry& odom1, const nav_msgs::Odometry& odom2) {
    double dx = odom2.pose.pose.position.x - odom1.pose.pose.position.x;
    double dy = odom2.pose.pose.position.y - odom1.pose.pose.position.y;
    double dz = odom2.pose.pose.position.z - odom1.pose.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// 获取当前日期和时间字符串
std::string get_current_datetime() {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S")
       << '.' << std::setfill('0') << std::setw(3) << now_ms.count();
    return ss.str();
}

class OdometerNode {
public:
    OdometerNode() {
        // 读取之前的总里程
        read_previous_distance();

        // 订阅 /rtk_gps_node/odom 话题
        odom_sub = nh.subscribe("/rtk_gps_node/odom", 10, &OdometerNode::odomCallback, this);

        // 发布 /APU/Nav/State/Odometer 话题
        odometer_pub = nh.advertise<std_msgs::Float32MultiArray>("/APU/Nav/State/Odometer", 10);

        // 创建定时器，每隔 10 秒调用一次 timerCallback
        timer = nh.createTimer(ros::Duration(10.0), &OdometerNode::timerCallback, this);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
    ros::Publisher odometer_pub;
    ros::Timer timer;

    // 类成员变量
    double total_distance_2d = 0.0;
    double current_distance_2d = 0.0;
    double total_distance_3d = 0.0;
    double current_distance_3d = 0.0;
    nav_msgs::Odometry previous_odom;
    bool first_odom_received = false;

    // 读取之前的总里程
    void read_previous_distance() {
        std::ifstream file("total_distance.txt");
        if (file.is_open()) {
            file >> total_distance_2d >> total_distance_3d;
            file.close();
        } else {
            ROS_INFO("[%s] total_distance.txt file not found.", get_current_datetime().c_str());
        }
    }

    // 保存里程信息到文件
    void save_distance() {
        std::ofstream file("total_distance.txt");
        if (file.is_open()) {
            file << total_distance_2d << " " << total_distance_3d << " " << current_distance_2d << " " << current_distance_3d;
            file.close();
            ROS_INFO("[%s] Distance saved: Total 2D: %.2f m, Total 3D: %.2f m, Current 2D: %.2f m, Current 3D: %.2f m",
                     get_current_datetime().c_str(), total_distance_2d, total_distance_3d, current_distance_2d, current_distance_3d);
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // 打印接收到数据的日志
        ROS_INFO("[%s] Received odometry data. Position: (%.2f, %.2f, %.2f)",
                 get_current_datetime().c_str(), msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

        if (first_odom_received) {
            double d_2d = euclidean_distance_2d(previous_odom, *msg);
            double d_3d = euclidean_distance_3d(previous_odom, *msg);
            current_distance_2d += d_2d;
            total_distance_2d += d_2d;
            current_distance_3d += d_3d;
            total_distance_3d += d_3d;
        } else {
            first_odom_received = true;
        }
        previous_odom = *msg;

        // 发布里程计信息
        std_msgs::Float32MultiArray odometer_msg;
        odometer_msg.data.resize(4);
        odometer_msg.data[0] = total_distance_2d;
        odometer_msg.data[1] = current_distance_2d;
        odometer_msg.data[2] = total_distance_3d;
        odometer_msg.data[3] = current_distance_3d;
        odometer_pub.publish(odometer_msg);
    }

    void timerCallback(const ros::TimerEvent&) {
        save_distance();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometer_node");
    OdometerNode node;
    ros::spin();
    return 0;
}